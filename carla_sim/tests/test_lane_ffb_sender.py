"""Execute sender definitions from the real Windows file without importing CARLA."""
import ast
import json
from pathlib import Path
import socket
import unittest
from unittest.mock import Mock, patch

from evdev import ecodes
from carla_sim.g29_ffb import LFA_MAX_CORRECTION, LaneReceiver, WheelFFB

SOURCE = Path(__file__).resolve().parents[1] / 'carla_pedal_control_v17_safety_json_5006.py'
TREE = ast.parse(SOURCE.read_text())
CONSTANTS = {'RPI_FFB_IP', 'RPI_FFB_PORT', 'LANE_FFB_SEND_INTERVAL'}
NODES = [n for n in TREE.body if
         (isinstance(n, ast.ClassDef) and n.name == 'LaneFFBSender') or
         (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and
          (t.id in CONSTANTS or t.id.startswith('LFA_')) for t in n.targets)) or
         (isinstance(n, ast.FunctionDef) and n.name in
          {'clamp', 'lerp', 'move_toward', 'get_lfa_speed_parameters'})]
NS = {'socket': socket, 'json': json}
exec(compile(ast.Module(body=NODES, type_ignores=[]), str(SOURCE), 'exec'), NS)


class SenderTests(unittest.TestCase):
    def setUp(self):
        self.sock = Mock()
        with patch.object(socket, 'socket', return_value=self.sock):
            self.sender = NS['LaneFFBSender']()

    def packet(self, enabled, state, correction, now=1.):
        self.sender.send(now, enabled, state, correction)
        data, dest = self.sock.sendto.call_args.args
        self.assertEqual(dest, ('192.168.50.2', 5007))
        p = json.loads(data)
        self.assertEqual(set(p), {'type', 'sequence', 'lane_assist_enabled',
            'lane_assist_active', 'lane_assist_state', 'lane_assist_correction'})
        return p

    def test_states_and_json(self):
        cases = [(False, 'OFF', 0., False), (True, 'STANDBY', 0., False),
                 (True, 'ACTIVE', .034, True), (True, 'ACTIVE', -.034, True),
                 (True, 'DRIVER_OVERRIDE', .04, False),
                 (True, 'REACQUIRE', .02, False), (True, 'LOW_SPEED', .01, False),
                 (True, 'OVERRIDE_HOLD', .02, False),
                 (True, 'ACTIVE', .003, False)]
        for seq, (enabled, state, correction, active) in enumerate(cases):
            p = self.packet(enabled, state, correction, now=1.+seq)
            self.assertEqual(p['type'], 'lane_ffb')
            self.assertEqual(p['sequence'], seq)
            self.assertEqual(p['lane_assist_enabled'], enabled)
            self.assertEqual(p['lane_assist_active'], active)
            self.assertEqual(p['lane_assist_state'], state)
            self.assertEqual(p['lane_assist_correction'], correction)

    def test_send_only_and_rate(self):
        self.sock.bind.assert_not_called()
        self.sock.setblocking.assert_called_once_with(False)
        self.packet(True, 'ACTIVE', .1)
        self.sender.send(1.01, True, 'ACTIVE', .2)
        self.assertEqual(self.sock.sendto.call_count, 1)
        self.assertEqual(self.packet(True, 'ACTIVE', -.1, 1.04)['sequence'], 1)

    def test_send_error_and_close(self):
        self.sock.sendto.side_effect = OSError('network unavailable')
        self.sender.send(1., True, 'ACTIVE', .1)
        self.assertEqual(self.sender.sequence, 1)
        self.sock.sendto.side_effect = None
        self.assertEqual(self.packet(False, 'OFF', 0., 2.)['sequence'], 1)
        self.sender.close()
        self.sock.close.assert_called_once()

    def test_socket_creation_error(self):
        with patch.object(socket, 'socket', side_effect=OSError('no socket')):
            sender = NS['LaneFFBSender']()
        sender.send(1., False, 'OFF', 0.)
        sender.close()

    @patch("carla_sim.g29_ffb.ENABLE_LANE_FFB", True)
    def test_actual_limit_and_pi_normalization(self):
        limits = [NS['get_lfa_speed_parameters'](v / 10)['max_corr'] for v in range(1301)]
        self.assertEqual(max(limits), .16)
        self.assertEqual(LFA_MAX_CORRECTION, max(limits))
        device = Mock()
        device.capabilities.return_value = {ecodes.EV_FF: [ecodes.FF_CONSTANT]}
        device.upload_effect.return_value = 1
        wheel = WheelFFB(device)  # Real Pi default, no test-only limit.
        rx = LaneReceiver()
        for i, sign in enumerate((1, -1)):
            correction = sign * max(limits)
            self.assertEqual(correction / wheel.limit, float(sign))
            self.packet(True, 'ACTIVE', correction, now=1.+i)
            rx.accept(self.sock.sendto.call_args.args[0], 1.+i)
            wheel.update(rx, 1.+i)
            self.assertEqual(wheel.target, int(sign * 32767 * .15))

    def test_call_uses_final_frame_values_after_apply_control(self):
        main = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        calls = [n for n in ast.walk(main) if isinstance(n, ast.Call)]
        send = next(n for n in calls if ast.unparse(n.func) == 'lane_ffb_sender.send')
        apply = next(n for n in calls if ast.unparse(n) == 'vehicle.apply_control(control)')
        final = max(n.lineno for n in ast.walk(main) if isinstance(n, ast.Assign) and
                    any(ast.unparse(t) == 'lane_assist_correction' for t in n.targets))
        self.assertGreater(send.lineno, final)
        self.assertGreater(send.lineno, apply.lineno)
        self.assertEqual([ast.unparse(a) for a in send.args[1:]],
                         ['lane_assist_enabled', 'lane_assist_state', 'lane_assist_correction'])
        self.assertTrue(any(isinstance(n, ast.Call) and ast.unparse(n.func) == 'lane_ffb_sender.close'
                            for n in ast.walk(ast.Module(body=main.body[-1].finalbody, type_ignores=[]))))


if __name__ == '__main__':
    unittest.main()
