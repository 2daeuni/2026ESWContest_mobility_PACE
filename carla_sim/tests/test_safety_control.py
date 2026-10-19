"""Test real receiver code without Windows/CARLA dependencies."""
import ast
import contextlib
import io
import json
from pathlib import Path
import socket
import textwrap
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from safety_ecu.carla.control_sender import CarlaControlSender
from safety_ecu.control.command_mapper import CommandMapper

SOURCE = Path(__file__).resolve().parents[1] / 'carla_pedal_control_v17_safety_json_5006.py'
TREE = ast.parse(SOURCE.read_text())
NS = {'socket': socket, 'json': json}
exec(compile(ast.Module(body=[n for n in TREE.body if isinstance(n, ast.ClassDef)
     and n.name == 'SafetyControlReceiver'], type_ignores=[]), str(SOURCE), 'exec'), NS)


CREEP_NODES = [n for n in TREE.body if (
    isinstance(n, ast.Assign) and any(
        isinstance(t, ast.Name) and t.id.startswith('CREEP_') for t in n.targets
    )
) or (isinstance(n, ast.FunctionDef) and n.name in ('clamp', 'calculate_creep_throttle'))]
exec(compile(ast.Module(body=CREEP_NODES, type_ignores=[]), str(SOURCE), 'exec'), NS)


class SafetyControlTests(unittest.TestCase):
    def setUp(self):
        self.sock = Mock()
        self.sock.recvfrom.side_effect = BlockingIOError
        with patch.object(socket, 'socket', return_value=self.sock):
            self.receiver = NS['SafetyControlReceiver']()
        self.output = io.StringIO()

    def apply_packet(self, mode, limit, throttle, risk=0, now=1., failsafe=False):
        packet = dict(type='safety_control', sequence=123, mode=mode,
                      limit_percent=limit, risk_level=risk, failsafe=failsafe)
        self.sock.recvfrom.side_effect = [(json.dumps(packet).encode(), ('pi', 5008)), BlockingIOError]
        control = SimpleNamespace(throttle=throttle, brake=.37, steer=-.42)
        with contextlib.redirect_stdout(self.output):
            self.receiver.apply(control, now)
        self.assertEqual(control.brake, 1. if failsafe or mode == 2 else .37)
        self.assertEqual(control.steer, -.42)
        return control.throttle

    def test_creep_inputs_and_taper(self):
        creep = NS['calculate_creep_throttle']
        for speed, expected in ((0, .18), (3, .126), (5, .09), (8, .036),
                                (10, 0.), (15, 0.)):
            with self.subTest(speed=speed):
                self.assertAlmostEqual(creep('D', 0., 0., speed), expected)
        self.assertEqual(creep('D', 0., .03, 0), 0.)
        self.assertEqual(creep('D', .03, 0., 0), 0.)
        self.assertEqual(creep('R', 0., 0., 0), 0.)
        self.assertEqual(creep('P', 0., 0., 0), 0.)
        with patch.dict(NS, CREEP_ENABLED=False):
            self.assertEqual(creep('D', 0., 0., 0), 0.)

    def test_reverse_and_park_bypass_every_safety_state(self):
        for gear in ('R', 'P'):
            for mode, failsafe, last_rx in ((0, False, 1.), (1, False, 1.),
                    (2, False, 1.), (0, True, 1.), (2, True, None),
                    (2, False, 0.)):
                with self.subTest(gear=gear, mode=mode, failsafe=failsafe, last_rx=last_rx):
                    self.receiver.mode = mode
                    self.receiver.limit_percent = 30
                    self.receiver.failsafe = failsafe
                    self.receiver.last_rx = last_rx
                    self.receiver.last_log = None
                    control = SimpleNamespace(throttle=.7, brake=.12, steer=-.4, reverse=True)
                    with contextlib.redirect_stdout(self.output):
                        self.receiver.apply(control, 1., gear=gear)
                    self.assertEqual((control.throttle, control.brake, control.steer,
                                      control.reverse), (.7, .12, -.4, True))
        self.assertIn('GEAR=R', self.output.getvalue())
        self.assertIn('CREEP=0', self.output.getvalue())
        self.assertIn('SAFETY_BYPASS_FOR_REVERSE=1', self.output.getvalue())
        self.assertNotIn('STALE -> STOP', self.output.getvalue())

    def test_main_driver_priority_and_gear_transition(self):
        source = SOURCE.read_text()
        start = source.index('            keys = pygame.key.get_pressed()')
        end = source.index('            # REALISTIC LANE FOLLOWING ASSIST', start)
        code = compile(textwrap.dedent(source[start:end]), str(SOURCE), 'exec')
        for gear, accel, brake, speed, pending, expected_throttle in (
                ('D', 0., 0., 0., False, .18),
                ('D', 0., 0., 10., False, 0.),
                ('D', .65, 0., 0., False, .65),
                ('D', 0., .03, 0., False, 0.),
                ('R', .65, 0., 0., False, .65),
                ('R', 0., 0., 0., False, 0.),
                ('D', 0., 0., 5., True, 0.),
                ('D', 0., 0., 0., True, 0.)):
            with self.subTest(gear=gear, accel=accel, brake=brake, speed=speed, pending=pending):
                ns = dict(NS, pygame=SimpleNamespace(key=SimpleNamespace(get_pressed=lambda: [])),
                          get_speed_kmh=lambda vehicle: speed,
                          get_speed_steering_gain=lambda speed: .82,
                          STEERING_GAIN_LOW_SPEED=.82, BRAKE_INPUT_GAIN=1.2,
                          GEAR_CHANGE_SPEED_KMH=1., control_mode='KEYBOARD',
                          keyboard=SimpleNamespace(update=lambda *args: (accel, brake, 0.),
                                                   reset=lambda: None),
                          milliseconds=16, vehicle=object(), applied_gear=gear,
                          requested_gear='R' if pending else gear,
                          gear_change_pending=pending, smooth_throttle=0., smooth_brake=0.)
                with contextlib.redirect_stdout(self.output):
                    exec(code, ns)
                self.assertAlmostEqual(ns['target_throttle'], expected_throttle)
                if brake > .02 or (pending and speed > 1.):
                    self.assertEqual(ns['creep_throttle'], 0.)
                    self.assertGreater(ns['target_brake'], 0.)

    def test_safety_overrides_creep(self):
        creep = NS['calculate_creep_throttle']('D', 0., 0., 0.)
        for mode, failsafe, fresh, expected in (
                (0, False, True, (.18, 0.)), (1, False, True, (.18, 0.)),
                (2, False, True, (0., 1.)), (0, True, True, (0., 1.)),
                (0, False, False, (0., 1.))):
            self.receiver.mode = mode
            self.receiver.limit_percent = 30
            self.receiver.failsafe = failsafe
            self.receiver.last_rx = 1. if fresh else None
            self.receiver.last_log = None
            control = SimpleNamespace(throttle=creep, brake=0.)
            with contextlib.redirect_stdout(self.output):
                self.receiver.apply(control, 1., gear='D', speed_kmh=0.,
                                    creep_throttle=creep, driver_throttle=0.)
            self.assertEqual((control.throttle, control.brake), expected)
        for field in ('GEAR=D', 'SPEED_KMH=0.0', 'CREEP=1',
                      'CREEP_THROTTLE=0.180', 'RAW_THROTTLE=0.00'):
            self.assertIn(field, self.output.getvalue())

    def test_requested_limits(self):
        for mode, limit, raw, expected in [(0, 100, 1., 1.), (1, 30, 1., .3),
                (1, 30, .2, .2), (1, 50, .8, .5)]:
            with self.subTest(mode=mode, limit=limit, raw=raw):
                self.assertEqual(self.apply_packet(mode, limit, raw), expected)

    def test_mapper_levels(self):
        for risk in range(4):
            motor = CommandMapper().map(risk, True)['motor']
            self.assertEqual(self.apply_packet(motor['control_mode'], motor['output_limit'],
                                             1., risk), .3 if risk == 2 else 0. if risk == 3 else 1.)

    def test_stop_without_failsafe(self):
        self.assertEqual(self.apply_packet(2, 0, .8, risk=3), 0.)
        self.assertIn('MODE=STOP', self.output.getvalue())
        self.assertIn('FAILSAFE=0', self.output.getvalue())

    def test_failsafe_overrides_all_modes(self):
        for mode in (0, 1, 2):
            self.assertEqual(self.apply_packet(mode, 100, 1., failsafe=True), 0.)

    def test_stale_stops_at_timeout_boundary(self):
        for mode in (0, 1, 2):
            self.apply_packet(mode, 30, 1.)
            self.sock.recvfrom.side_effect = BlockingIOError
            for now in (1.5, 1.6):
                control = SimpleNamespace(throttle=.8, brake=.37, steer=-.42)
                with contextlib.redirect_stdout(self.output):
                    self.receiver.apply(control, now)
                self.assertEqual((control.throttle, control.brake, control.steer), (0., 1., -.42))
        self.assertIn('FRESH=0', self.output.getvalue())
        self.assertIn('[CARLA SAFETY] STALE -> STOP', self.output.getvalue())

    def test_startup_stops_and_fresh_command_recovers(self):
        control = SimpleNamespace(throttle=1., brake=0., steer=.61)
        with contextlib.redirect_stdout(self.output):
            self.receiver.apply(control, .1)
        self.assertEqual((control.throttle, control.brake, control.steer), (0., 1., .61))
        self.assertIn('WAITING FOR FIRST safety_control PACKET -> STOP', self.output.getvalue())
        self.assertEqual(self.apply_packet(0, 100, 1.), 1.)
        self.sock.recvfrom.side_effect = BlockingIOError
        with contextlib.redirect_stdout(self.output):
            self.receiver.apply(control, 1.5)
        self.assertEqual(control.brake, 1.)
        self.assertEqual(self.apply_packet(0, 100, .7, now=1.6), .7)

    def test_latest_valid_packet_and_log_values(self):
        packets = [dict(type='safety_control', sequence=i, mode=mode,
                        limit_percent=limit, risk_level=risk, failsafe=False)
                   for i, mode, limit, risk in ((1, 2, 0, 3), (2, 1, 30, 2))]
        self.sock.recvfrom.side_effect = [(json.dumps(p).encode(), ('pi', 5008))
                                        for p in packets] + [BlockingIOError]
        control = SimpleNamespace(throttle=1., brake=.2, steer=.4)
        with contextlib.redirect_stdout(self.output):
            self.receiver.apply(control, 1.)
        self.assertEqual((control.throttle, control.brake, control.steer), (.3, .2, .4))
        for field in ('RISK=L2', 'MODE=LIMIT', 'LIMIT=30', 'FAILSAFE=0', 'FRESH=1',
                      'RAW_THROTTLE=1.00', 'RAW_BRAKE=0.20',
                      'APPLIED_THROTTLE=0.300', 'APPLIED_BRAKE=0.20'):
            self.assertIn(field, self.output.getvalue())
        self.sock.recvfrom.side_effect = BlockingIOError
        with contextlib.redirect_stdout(self.output):
            self.receiver.apply(control, 1.1)
        self.assertEqual(self.output.getvalue().count('RISK=L2'), 1)

    def test_bad_packets_do_not_refresh(self):
        for data in [b'{', b'[]', b'{}', b'\xff', json.dumps(dict(type='safety_control',
                sequence=0, mode=1, limit_percent=float('nan'), risk_level=3, failsafe=False)).encode()]:
            self.sock.recvfrom.side_effect = [(data, ('pi', 5008)), BlockingIOError]
            self.receiver.receive(1.)
            self.assertIsNone(self.receiver.last_rx)

    def test_socket_and_cleanup(self):
        self.sock.bind.assert_called_once_with(('0.0.0.0', 5008))
        self.sock.setblocking.assert_called_once_with(False)
        self.receiver.close()
        self.sock.close.assert_called_once()

    def test_sender_rate_schema_and_final_command(self):
        tx = Mock()
        with patch.object(socket, 'socket', return_value=tx):
            sender = CarlaControlSender()
        motor = dict(control_mode=1, output_limit=30, command_flags=1)
        sender.send(motor, 3, False, now=1.)
        sender.send(motor, 3, False, now=1.01)
        self.assertEqual(tx.sendto.call_count, 1)
        data, dest = tx.sendto.call_args.args
        self.assertEqual(dest, ('192.168.50.1', 5008))
        self.assertEqual(json.loads(data), dict(type='safety_control', sequence=0,
                         mode=1, limit_percent=30, risk_level=3, failsafe=False))
        sender.send(dict(control_mode=2, output_limit=0), 3, True, now=1.06)
        self.assertEqual(json.loads(tx.sendto.call_args.args[0])['mode'], 2)
        tx.sendto.side_effect = OSError('unreachable')
        with contextlib.redirect_stdout(self.output):
            sender.send(motor, 3, False, now=2.)
        sender.close()
        tx.close.assert_called_once()

    def test_sender_continuous_rate_above_20hz(self):
        tx = Mock()
        with patch.object(socket, 'socket', return_value=tx):
            sender = CarlaControlSender()
        for tick in range(101):
            sender.send(dict(control_mode=0, output_limit=100), 0, False,
                        now=1. + tick * .01)
        self.assertGreaterEqual(tx.sendto.call_count, 20)
        self.assertEqual([json.loads(call.args[0])['sequence']
                          for call in tx.sendto.call_args_list],
                         list(range(tx.sendto.call_count)))
        sender.close()

    def test_main_applies_immediately_before_vehicle(self):
        main = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        for node in ast.walk(main):
            for field in ('body', 'orelse', 'finalbody'):
                statements = getattr(node, field, [])
                for index, statement in enumerate(statements):
                    if ast.unparse(statement) == 'vehicle.apply_control(control)':
                        self.assertEqual(ast.unparse(statements[index - 1]),
                            'safety_control_receiver.apply(control, time.monotonic(), gear=applied_gear, speed_kmh=current_speed_kmh, creep_throttle=creep_throttle, driver_throttle=driver_throttle)')
                        return
        self.fail('vehicle control call missing')
