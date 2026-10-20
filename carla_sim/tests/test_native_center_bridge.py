"""Exercise the real bridge output policy without opening devices or sockets."""
import unittest
from unittest.mock import Mock, call, patch

from evdev import ecodes
from carla_sim import carla_bridge as bridge
from carla_sim.g29_ffb import NativeAutoCenter


class NativeCenterBridgeTests(unittest.TestCase):
    def test_lane_states_and_disconnect_cleanup(self):
        for state in ('OFF', 'STANDBY', 'ACTIVE', 'DRIVER_OVERRIDE'):
            for disconnect in (False, True):
                with self.subTest(state=state, disconnect=disconnect):
                    device = Mock()
                    device.capabilities.return_value = {
                        ecodes.EV_FF: [ecodes.FF_AUTOCENTER, ecodes.FF_CONSTANT]}
                    wheel = Mock(device=device, steer=.25, gear='D')
                    if disconnect:
                        wheel.refresh_steering.side_effect = OSError('disconnected')
                    lane = Mock(socket=Mock(), packet=dict(
                        lane_assist_state=state, lane_assist_enabled=state != 'OFF',
                        lane_assist_active=state == 'ACTIVE', lane_assist_correction=.16))
                    udp = Mock()
                    udp.sendto.side_effect = KeyboardInterrupt()
                    with patch.object(bridge.socket, 'socket', return_value=udp), \
                         patch.object(bridge, 'LaneReceiver', return_value=lane), \
                         patch.object(bridge, 'open_can', side_effect=OSError()), \
                         patch.object(bridge, 'find_g29', return_value=wheel), \
                         patch.object(bridge.select, 'select', return_value=([], [], [])):
                        bridge.main()
                    self.assertEqual(device.write.call_args_list, [
                        call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, int(65535*.35)),
                        call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)])
                    device.upload_effect.assert_not_called()
                    device.erase_effect.assert_not_called()
                    device.close.assert_called_once()
                    self.assertEqual(udp.sendto.call_args.args[1], bridge.UDP_DESTINATION)
                    expected = b'0,100,0.0000,P' if disconnect else b'0,100,0.2500,D'
                    self.assertEqual(udp.sendto.call_args.args[0], expected)

    def test_cleanup_survives_device_loss(self):
        device = Mock()
        device.capabilities.return_value = {ecodes.EV_FF: [ecodes.FF_AUTOCENTER]}
        controller = NativeAutoCenter(device)
        device.write.side_effect = OSError('lost device')
        controller.close()
        device.write.assert_called_with(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)


if __name__ == '__main__':
    unittest.main()
