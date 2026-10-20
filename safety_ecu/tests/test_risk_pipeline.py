"""Initial risk policy and actual main-loop integration, without CAN/CARLA I/O.

Run: python -B -m unittest discover -s safety_ecu/tests -p test_risk_pipeline.py
Legacy script tests are run separately via their run_all_tests entrypoints.
"""
import copy
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from risk.bns import BNSCalculator
from risk.pms import PMSCalculator
from risk.risk_manager import RiskManager
from control.command_mapper import CommandMapper
from control.failsafe import FailSafeManager
from control.state import SystemState
from canbus.decoder import decode_pedal_status
from carla.receiver_v2 import CarlaReceiverV2
from hmi_sender import build_packet


class Pipeline:
    def __init__(self):
        self.b = BNSCalculator()
        self.p = PMSCalculator()
        self.r = RiskManager()

    def step(self, distance=50, ttc=10, acc=20, rate=50, brake=0,
             sensor=0, connected=True, closing=5):
        bs = self.b.calculate(None if distance is None else distance * 100,
                              distance is not None, 50, ttc,
                              environment_connected=connected, closing_speed_mps=closing)
        ps = self.p.calculate(acc, rate, True, sensor, brake=brake, brake_rate=0)
        conditions = {f'level{i}_conditions': (getattr(self.b, f'distance_l{i}'),
                       getattr(self.b, f'ttc_l{i}'), getattr(self.p, f'pedal_l{i}')) for i in (1, 2, 3)}
        return self.r.evaluate(bs, self.b.available, ps, self.p.available, **conditions)


class RiskPolicyTest(unittest.TestCase):
    def test_a_normal(self):
        p = Pipeline()
        for _ in range(10):
            self.assertEqual(p.step(), (0, 0))
        self.assertTrue(p.r.available)

    def test_b_distance_only_does_not_satisfy_two_of_three(self):
        p = Pipeline()
        for _ in range(10):
            total, level = p.step(distance=8)
            self.assertEqual(level, 0)
            self.assertGreaterEqual(total, .65)
        self.assertEqual(p.r.candidate_level, 0)

    def test_c_distance_and_ttc_confirm_level_two(self):
        p = Pipeline()
        for expected in (0, 0, 2):
            self.assertEqual(p.step(distance=8, ttc=2)[1], expected)
        self.assertEqual(p.r.candidate_level, 2)

    def test_l1_uses_two_conditions_and_three_confirmations(self):
        p = Pipeline()
        for expected in (0, 0, 1):
            self.assertEqual(p.step(distance=20, ttc=4)[1], expected)
        self.assertEqual(p.r.candidate_level, 1)

    def test_d_distance_and_pedal_with_absent_or_safe_ttc(self):
        for ttc in (None, 10):
            p = Pipeline()
            for _ in range(3):
                p.step(distance=8, ttc=ttc, acc=85, rate=500)
            self.assertTrue(p.b.available)
            self.assertEqual(p.r.risk_level, 2)

    def test_e_very_dangerous_is_l3_stop_without_estop(self):
        p = Pipeline()
        for expected in (0, 0, 3):
            self.assertEqual(p.step(distance=4, ttc=1, acc=95, rate=700)[1], expected)
        self.assertGreaterEqual(p.b.score, .9)
        self.assertGreaterEqual(p.p.score, .9)
        cmd = CommandMapper().map(p.r.risk_level, p.r.available)['motor']
        self.assertEqual(cmd, dict(control_mode=2, output_limit=0, command_flags=1))

    def test_f_recovery_five_confirmations_per_level(self):
        p = Pipeline()
        for _ in range(3):
            p.step(distance=4, ttc=1, acc=95, rate=700)
        for expected in (2, 1, 0):
            for _ in range(4):
                self.assertEqual(p.step()[1], expected + 1)
            self.assertEqual(p.step()[1], expected)

    def test_g_fresh_no_obstacle_is_available_and_stale_is_not(self):
        p = Pipeline()
        self.assertEqual(p.step(distance=None, ttc=None, closing=None), (0, 0))
        self.assertTrue(p.b.available)
        self.assertTrue(p.r.available)
        self.assertFalse(any(getattr(p.b, f'{kind}_l{i}') for kind in ('distance', 'ttc') for i in (1, 2, 3)))
        p.step(distance=None, ttc=None, connected=False, closing=None)
        self.assertFalse(p.b.available)
        self.assertFalse(p.r.available)

    def test_h_sensor_error_uses_existing_failsafe(self):
        p = Pipeline()
        p.step(sensor=4, acc=100, rate=1000)
        self.assertEqual(p.p.score, 0)
        self.assertFalse(p.p.available)
        state = SystemState()
        state.carla_connected = state.pedal_connected = True
        state.pedal_sensor_status = 4
        fs = FailSafeManager()
        self.assertTrue(fs.evaluate(state, 'CARLA'))
        self.assertEqual(fs.reason, 'PEDAL_SENSOR_ERROR')
        self.assertEqual(CommandMapper().map(p.r.risk_level, p.r.available, fs.active)['motor'],
                         dict(control_mode=2, output_limit=0, command_flags=3))

    def test_threshold_edges_monotonic_scores_and_negative_rates(self):
        b = BNSCalculator()
        scores = []
        for distance, minimum, level in ((50, 0, 0), (25, .35, 1), (15, .65, 2), (10, .9, 3), (0, 1, 3)):
            scores.append(b.calculate(distance*100, True, 50, None, environment_connected=True))
            self.assertGreaterEqual(scores[-1], minimum)
            for i in (1, 2, 3):
                self.assertEqual(getattr(b, f'distance_l{i}'), i <= level)
        self.assertEqual(scores, sorted(scores))
        for ttc, minimum in ((4, .35), (2.5, .65), (1.5, .9), (0, 1)):
            self.assertGreaterEqual(b.calculate(5000, True, 50, ttc, closing_speed_mps=5), minimum)
        p = PMSCalculator()
        for acc, rate, minimum, level in ((60, 0, .35, 1), (75, 0, .65, 2), (90, 0, .9, 3),
                                           (0, 250, .35, 1), (0, 450, .65, 2), (0, 650, .9, 3)):
            self.assertGreaterEqual(p.calculate(acc, rate, True, 0), minimum)
            self.assertTrue(getattr(p, f'pedal_l{level}'))
        self.assertEqual(p.calculate(20, -1000, True, 0), 0)
        scores = [p.calculate(0, rate, True, 0) for rate in (0, 250, 350, 450, 550, 650, 750, 850)]
        self.assertEqual(scores, sorted(scores))
        self.assertGreater(scores[2], scores[1])
        for acc in range(101):
            self.assertTrue(0 <= p.calculate(acc, acc*10, True, 0) <= 1)

    def test_brake_suppression_and_separate_conflict(self):
        p = PMSCalculator()
        self.assertEqual(p.calculate(95, 700, True, 0, brake=5, brake_rate=300), 0)
        self.assertFalse(p.pedal_l3)
        self.assertEqual(p.calculate(95, 700, True, 0, brake=20), .65)
        self.assertTrue(p.pedal_conflict)
        self.assertTrue(p.pedal_l1 and p.pedal_l2)
        self.assertFalse(p.pedal_l3)
        p.calculate(0, 0, True, 0)
        self.assertFalse(p.pedal_conflict)

    def test_invalid_values_clear_conditions_and_total_is_normalized(self):
        p = Pipeline()
        p.step(distance=4, ttc=1, acc=95, rate=700)
        p.step(acc=math.nan)
        self.assertFalse(p.p.available)
        self.assertFalse(p.p.pedal_l1)
        p.b.calculate(100, True, math.inf, 1, environment_connected=True)
        self.assertFalse(p.b.available)
        self.assertFalse(p.b.distance_l1)
        r = RiskManager()
        for bs, ps, expected in ((.4, .8, .8), (-5, 10, 1), (math.nan, .5, .5)):
            self.assertEqual(r.evaluate(bs, True, ps, True)[0], expected)

    def test_decoder_rate_units_and_carla_no_obstacle_timeout(self):
        raw = bytes([85, 0]) + struct.pack('<hh', 5000, -1200) + bytes([0, 7])
        decoded = decode_pedal_status(raw)
        self.assertEqual(decoded['accelerator_rate'], 500)
        self.assertEqual(decoded['brake_rate'], -120)
        receiver = CarlaReceiverV2()
        receiver.sock = Mock()
        wire = dict(source='CARLA', sequence=1, vehicle_speed_kmh=50,
                    distance_m=None, closing_speed_mps=None, ttc_s=None, obstacle_id=None)
        receiver.sock.recvfrom.return_value = (json.dumps(wire).encode(), ('192.168.0.57', 5006))
        with patch('carla.receiver_v2.time.monotonic', return_value=100):
            self.assertTrue(receiver.receive())
        state = SystemState()
        state.update_carla(receiver)
        b = BNSCalculator()
        self.assertEqual(b.calculate(state.carla_distance_cm, state.carla_distance_valid,
                         state.carla_vehicle_speed, state.carla_ttc,
                         closing_speed_mps=state.carla_closing_speed_mps,
                         environment_connected=state.carla_connected), 0)
        self.assertTrue(b.available)
        with patch('carla.receiver_v2.time.monotonic', return_value=100.5):
            self.assertFalse(receiver.check_freshness())
        state.invalidate_carla()
        self.assertTrue(FailSafeManager().evaluate(state, 'CARLA'))


class MainLoopIntegrationTest(unittest.TestCase):
    def test_actual_main_calls_conditions_maps_sends_and_publishes(self):
        # main() is exercised with in-memory receivers, mocked loggers and a fake CAN bus.
        spec = importlib.util.spec_from_file_location('safety_risk_main_mock', ROOT / 'main.py')
        main = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(main)
        rows = []
        for name, count, distance, ttc, acc, rate, status in (
            ('startup', 8, 50, 10, 0, 0, 0), ('A', 8, 50, 10, 20, 50, 0),
            ('B', 8, 8, 10, 20, 50, 0), ('C', 8, 8, 2, 20, 50, 0),
            ('D', 8, 8, None, 85, 500, 0), ('E', 8, 4, 1, 95, 700, 0),
            ('F', 30, 50, 10, 20, 50, 0), ('G', 8, None, None, 20, 50, 0),
            ('I', 8, None, None, 80, 50, 0),
            ('J', 8, None, None, 100, 700, 0),
            ('H', 8, 4, 1, 95, 700, 4)):
            rows += [dict(stage=name, distance=distance, ttc=ttc, acc=acc, rate=rate, status=status)] * count
        cursor = [0]
        alive = [0]
        rx = Mock()
        def recv(timeout):
            if cursor[0] >= len(rows):
                raise KeyboardInterrupt
            row = rows[cursor[0]]
            i = cursor[0]
            cursor[0] += 1
            if i % 3 == 0:
                return type('Frame', (), dict(arbitration_id=0x210,
                    data=bytes([0, 0, 255, 255, 0, 0, 0, i % 256]), timestamp=i*.01))()
            data = bytes([row['acc'], 0]) + struct.pack('<hh', row['rate']*10, 0) + bytes([row['status'], alive[0] % 256])
            alive[0] += 1
            return type('Frame', (), dict(arbitration_id=0x100, data=data, timestamp=i*.01))()
        rx.receive.side_effect = recv
        carla = Mock()
        def receive_carla():
            row = rows[min(cursor[0], len(rows)-1)]
            carla.connected, carla.vehicle_speed = True, 50
            carla.distance_m, carla.ttc = row['distance'], row['ttc']
            carla.closing_speed_mps = 5 if row['distance'] is not None else None
            carla.obstacle_id = 7 if row['distance'] is not None else None
            return True
        carla.receive.side_effect = receive_carla
        carla.check_freshness.return_value = True
        sender = main.CANSender()
        sender.bus = Mock()
        hmi = Mock()
        carla_control = Mock()
        snapshots = []
        def publish(state, **kwargs):
            packet = build_packet(state, seq=len(snapshots), **kwargs)
            snapshots.append((rows[cursor[0]-1]['stage'], copy.deepcopy(packet)))
        hmi.publish.side_effect = publish
        with patch.object(main, 'CANReceiver', return_value=rx), \
             patch.object(main, 'CANSender', return_value=sender), \
             patch.object(sender, 'open'), patch.object(sender, 'close'), \
             patch.object(main, 'CarlaReceiverV2', return_value=carla), \
             patch.object(main, 'HmiSender', return_value=hmi), \
             patch.object(main, 'CarlaControlSender', return_value=carla_control), \
             patch.object(main, 'PedalLogger'), patch.object(main, 'SystemLogger'), \
             patch.object(main, 'PedalDiagnostics'), \
             patch.object(main.time, 'monotonic', side_effect=lambda: 100 + cursor[0]*.01), \
             patch('socket.socket', side_effect=AssertionError('network forbidden')), \
             patch('builtins.print') as logs:
            main.main()
        by_stage = {name: [pkt for stage, pkt in snapshots if stage == name] for name in 'ABCDEFGHIJ'}
        self.assertTrue(all(by_stage.values()))
        self.assertEqual(by_stage['A'][-1]['risk_level'], 0)
        self.assertEqual(by_stage['A'][-1]['motor_output_limit'], 100)
        self.assertEqual(by_stage['B'][-1]['risk_level'], 0)
        self.assertGreater(by_stage['B'][-1]['bns'], 0)
        for stage in ('C', 'D'):
            self.assertEqual(by_stage[stage][-1]['risk_level'], 2)
        self.assertEqual(by_stage['E'][-1]['risk_level'], 3)
        self.assertEqual(by_stage['E'][-1]['motor_control_mode'], 2)
        self.assertEqual(by_stage['E'][-1]['motor_output_limit'], 0)
        self.assertEqual(by_stage['F'][-1]['risk_level'], 0)
        self.assertTrue(by_stage['G'][-1]['risk_available'])
        self.assertTrue(by_stage['G'][-1]['bns_available'])
        self.assertEqual(by_stage['G'][-1]['bns'], 0)
        for stage in ('I', 'J'):
            self.assertEqual(by_stage[stage][-1]['risk_level'], 0)
            self.assertEqual(by_stage[stage][-1]['motor_output_limit'], 100)
            self.assertGreater(by_stage[stage][-1]['pms'], 0)
            self.assertEqual(by_stage[stage][-1]['bns'], 0)
        self.assertTrue(by_stage['H'][-1]['failsafe_active'])
        self.assertEqual(by_stage['H'][-1]['failsafe_reason'], 'PEDAL_SENSOR_ERROR')
        self.assertEqual(by_stage['H'][-1]['motor_control_mode'], 2)
        motor_frames = [call.args[0] for call in sender.bus.send.call_args_list if call.args[0].arbitration_id == 0x120]
        self.assertEqual(len(carla_control.send.call_args_list), len(motor_frames))
        for call, frame in zip(carla_control.send.call_args_list, motor_frames):
            motor, risk, failsafe_active = call.args
            self.assertEqual([motor['control_mode'], motor['output_limit'], risk,
                              motor['command_flags']], list(frame.data[:4]))
        carla_control.close.assert_called_once()
        self.assertTrue(any(list(f.data[:4]) == [2, 0, 3, 1] for f in motor_frames))
        self.assertEqual(list(motor_frames[-1].data[:4]), [2, 0, 0, 3])
        self.assertTrue(all(len(f.data) == 8 and bytes(f.data[4:7]) == b'\0\0\0' for f in motor_frames))
        risk_logs = [call.args[0] for call in logs.call_args_list if call.args and str(call.args[0]).startswith('[RISK]')]
        self.assertGreaterEqual(len(risk_logs), 2)
        self.assertLess(len(risk_logs), 5)  # ~1 second of loop data, not one log per iteration.
        self.assertTrue(all('CANDIDATE=L' in line for line in risk_logs))
        debug_logs = [call.args[0] for call in logs.call_args_list
                      if call.args and str(call.args[0]).startswith('[RISKDBG]')]
        self.assertGreaterEqual(len(debug_logs), 2)
        self.assertLess(len(debug_logs), 5)
        self.assertTrue(all(all(field in line for field in
            ('OBS=', 'DIST=', 'TTC=', 'CLOSING=', 'D1=', 'D2=', 'D3=',
             'T1=', 'T2=', 'T3=', 'P1=', 'P2=', 'P3=', 'CAND=L',
             'LEVEL=L', 'CMD_MODE=', 'LIMIT=')) for line in debug_logs))
        sys.path.append(str(ROOT.parent / 'hmi_ecu'))
        from hmi import state_merge
        for _, packet in snapshots:
            self.assertEqual(packet['score_max'], 1.0)
            displayed = state_merge.merge({}, {'data': packet, 't': 100}, now=100)
            if packet['risk_available']:
                self.assertAlmostEqual(displayed['bns'], packet['bns'])
                self.assertAlmostEqual(displayed['pms'], packet['pms'])
                self.assertAlmostEqual(displayed['total_risk'], packet['total_risk'])
            self.assertTrue(0 <= packet['bns'] <= 1 and 0 <= packet['pms'] <= 1 and 0 <= packet['total_risk'] <= 1)
            json.dumps(packet, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
