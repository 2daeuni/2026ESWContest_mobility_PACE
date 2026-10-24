"""Collision -> absent obstacle -> pedal alone, through the real UDP parser."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from test_risk_pipeline import Pipeline, CarlaReceiverV2, SystemState, FailSafeManager, CommandMapper

SOURCE = Path(__file__).resolve().parents[2] / 'carla_sim/carla_pedal_control_v17_safety_json_5006.py'
TREE = ast.parse(SOURCE.read_text())
NODE = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'build_safety_telemetry')
ID_NODE = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'safety_candidate_id')


class ObstacleClearTests(unittest.TestCase):
    def test_accelerator_alone_never_counts_as_two_conditions(self):
        for acc, rate in ((80, 50), (100, 700)):
            with self.subTest(acc=acc, rate=rate):
                pipeline = Pipeline()
                for _ in range(20):
                    _, level = pipeline.step(distance=None, ttc=None,
                                             closing=0, acc=acc, rate=rate)
                    self.assertEqual(level, 0)
                    self.assertEqual(pipeline.r.candidate_level, 0)
                    self.assertTrue(pipeline.b.available)
                    self.assertEqual(pipeline.b.score, 0.0)
                    self.assertGreater(pipeline.p.score, 0.0)
                    for i in (1, 2, 3):
                        conditions = (getattr(pipeline.b, f'distance_l{i}'),
                                      getattr(pipeline.b, f'ttc_l{i}'),
                                      getattr(pipeline.p, f'pedal_l{i}'))
                        self.assertFalse(conditions[0])
                        self.assertFalse(conditions[1])
                        self.assertLessEqual(sum(conditions), 1)

    def setUp(self):
        self.detect = Mock(return_value=None)
        self.ns = dict(get_speed_kmh=lambda ego: 30., get_forward_speed_mps=lambda *args: 5.,
                       NIGHT_SCENARIO_OBSTACLE_ID=900001, find_front_vehicle_for_safety=self.detect,
                       SAFETY_FRONT_MAX_DISTANCE_M=50.0,
                       SAFETY_FRONT_MAX_LOOKAHEAD_M=120.0,
                       SAFETY_FRONT_LOOKAHEAD_S=4.5,
                       SAFETY_VEHICLE_ONLY=True)
        exec(compile(ast.Module(body=[ID_NODE, NODE], type_ignores=[]), str(SOURCE), 'exec'), self.ns)
        self.build = self.ns['build_safety_telemetry']
        self.ego = Mock()

    def test_virtual_preview_ignored_until_real_vehicle_exists(self):
        preview = self.build(None, self.ego, 1, True, 60.)
        self.assertIsNone(preview['obstacle_id'])
        real = dict(actor=SimpleNamespace(id=77), distance_m=0., closing_speed_mps=5., ttc_s=0.)
        self.detect.return_value = real
        collision = self.build(None, self.ego, 2, True, 10., True)
        self.assertEqual(collision['obstacle_id'], 77)
        self.assertEqual(collision['distance_m'], 0.)
        self.detect.return_value = None
        for distance, materialized in ((10., True), (0., True), (0., False)):
            clear = self.build(None, self.ego, 3, True, distance, materialized)
            self.assertTrue(all(clear[k] is None for k in
                            ('obstacle_id', 'distance_m', 'closing_speed_mps', 'ttc_s')))

    def test_full_collision_clear_recovery_and_accelerator_only(self):
        receiver = CarlaReceiverV2()
        receiver.sock = Mock()
        state = SystemState()
        state.pedal_connected = True
        state.pedal_sensor_status = 0
        pipeline = Pipeline()
        seq = 0

        def step(packet, acc=20, rate=50):
            nonlocal seq
            packet = dict(packet, sequence=seq)
            seq += 1
            receiver.sock.recvfrom.return_value = (json.dumps(packet).encode(), ('192.168.50.1', 5006))
            with patch('carla.receiver_v2.time.monotonic', return_value=100.):
                self.assertTrue(receiver.receive())
            state.update_carla(receiver)
            bs = pipeline.b.calculate(state.carla_distance_cm, state.carla_distance_valid,
                                      state.carla_vehicle_speed, state.carla_ttc,
                                      closing_speed_mps=state.carla_closing_speed_mps,
                                      environment_connected=state.carla_connected)
            ps = pipeline.p.calculate(acc, rate, True, 0)
            conditions = {f'level{i}_conditions': (getattr(pipeline.b, f'distance_l{i}'),
                          getattr(pipeline.b, f'ttc_l{i}'), getattr(pipeline.p, f'pedal_l{i}')) for i in (1, 2, 3)}
            return pipeline.r.evaluate(bs, pipeline.b.available, ps, pipeline.p.available, **conditions)[1]

        clear = self.build(None, self.ego, 0, False, None)
        self.assertEqual(clear['gear'], 'D')
        for _ in range(receiver.READY_CONFIRM_COUNT):
            self.assertEqual(step(clear), 0)
        collision = dict(clear, obstacle_id=77, distance_m=0., closing_speed_mps=5., ttc_s=0.)
        for expected in (0, 0, 3):
            self.assertEqual(step(collision), expected)
            self.assertEqual(pipeline.r.candidate_level, 3)
        self.assertEqual(CommandMapper().map(3, True)['motor']['output_limit'], 0)
        for expected in (2, 1, 0):
            for count in range(5):
                self.assertEqual(step(clear), expected if count == 4 else expected+1)
                self.assertEqual(pipeline.r.candidate_level, 0)
                self.assertFalse(state.carla_obstacle_present)
                self.assertIsNone(state.carla_distance_cm)
                self.assertIsNone(state.carla_ttc)
                self.assertEqual(state.carla_closing_speed_mps, 0)
                self.assertEqual(pipeline.b.score, 0)
                self.assertTrue(pipeline.b.available)
                self.assertFalse(any(getattr(pipeline.b, f'{kind}_l{i}')
                                     for kind in ('distance', 'ttc') for i in (1, 2, 3)))
        for acc, rate in ((80, 50), (100, 700)):
            for _ in range(20):
                self.assertEqual(step(clear, acc, rate), 0)
                self.assertEqual(pipeline.r.candidate_level, 0)
                self.assertTrue(pipeline.p.pedal_l1)
                self.assertGreater(pipeline.p.score, 0)
                self.assertEqual(CommandMapper().map(0, pipeline.r.available)['motor']['output_limit'], 100)
        with patch('carla.receiver_v2.time.monotonic', return_value=100.5):
            self.assertFalse(receiver.check_freshness())
        state.invalidate_carla()
        self.assertFalse(state.carla_obstacle_present)
        fs = FailSafeManager()
        self.assertTrue(fs.evaluate(state, 'CARLA'))
        self.assertEqual(fs.reason, 'CARLA_COMMUNICATION_ERROR')


if __name__ == '__main__':
    unittest.main()
