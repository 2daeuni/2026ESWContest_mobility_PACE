import json
import unittest
from unittest.mock import Mock, patch

from evdev import ecodes
from carla_sim.g29_ffb import LaneReceiver, WheelFFB, diagnose_capabilities
from carla_sim import carla_bridge


class FFBTests(unittest.TestCase):
    def setUp(self):
        self.lane_flag = patch("carla_sim.g29_ffb.ENABLE_LANE_FFB", True)
        self.lane_flag.start()
        self.addCleanup(self.lane_flag.stop)
        self.device = Mock()
        self.device.capabilities.return_value = {ecodes.EV_FF: [
            ecodes.FF_CONSTANT, ecodes.FF_AUTOCENTER, ecodes.FF_GAIN]}
        self.device.upload_effect.return_value = 7
        self.rx = LaneReceiver()
        # Test-only limit; this is NOT a claimed CARLA limit.
        self.ffb = WheelFFB(self.device, correction_limit=.1)

    def packet(self, correction=.1, enabled=True, active=True, state='ACTIVE', seq=1, now=1.):
        self.rx.accept(json.dumps(dict(type='lane_ffb', sequence=seq,
            lane_assist_enabled=enabled, lane_assist_active=active,
            lane_assist_state=state, lane_assist_correction=correction)), now)
        self.ffb.update(self.rx, now)

    def test_diagnostic_is_read_only(self):
        self.device.reset_mock()
        caps = diagnose_capabilities(self.device)
        self.assertTrue(caps['FF_CONSTANT'])
        self.assertFalse(caps['FF_SPRING'])
        self.device.write.assert_not_called()
        self.device.upload_effect.assert_not_called()

    def test_a_off_center(self):
        self.packet(enabled=False, active=False, state='OFF')
        self.assertEqual(self.ffb.applied, 0)
        self.assertEqual(self.ffb.center, 0)
        self.device.write.assert_any_call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)

    def test_b_standby(self):
        self.packet(active=False, state='STANDBY')
        self.assertEqual(self.ffb.applied, 0)

    def test_c_d_direction(self):
        self.packet()
        positive = self.ffb.applied
        self.ffb.stop()
        self.packet(correction=-.1, seq=2)
        self.assertGreater(positive, 0)
        self.assertEqual(self.ffb.applied, -positive)

    def test_e_override_and_other_states(self):
        for i, state in enumerate(['DRIVER_OVERRIDE', 'REACQUIRE', 'LOW_SPEED', 'OFF']):
            self.packet(seq=2*i+1)
            self.packet(state=state, seq=2*i+2)
            self.assertEqual(self.ffb.applied, 0)
            self.device.write.assert_any_call(ecodes.EV_FF, 7, 0)

    def test_f_timeout(self):
        self.packet()
        self.ffb.update(self.rx, 1.201)
        self.assertEqual(self.ffb.applied, 0)
        self.assertEqual(self.ffb.center, 0)

    def test_g_slew_and_cap(self):
        self.packet(correction=100)
        for i in range(2, 15):
            previous = self.ffb.applied
            self.packet(correction=100, seq=i)
            self.assertLessEqual(abs(self.ffb.applied-previous), int(32767*.02))
            self.assertLessEqual(abs(self.ffb.applied), int(32767*.15))
        previous = self.ffb.applied
        self.packet(correction=-100, seq=15)
        self.assertLessEqual(abs(self.ffb.applied-previous), int(32767*.02))

    def test_invalid_json_and_duplicate(self):
        self.packet()
        self.rx.accept(b'bad json', 1.01)
        self.ffb.update(self.rx, 1.01)
        self.assertEqual(self.ffb.applied, 0)
        self.packet(seq=2, now=2.)
        self.rx.accept(json.dumps(self.rx.packet), 2.1)
        self.assertEqual(self.rx.last_rx, 2.)

    def test_unknown_limit_disables_lane(self):
        self.ffb.limit = None
        self.packet()
        self.device.upload_effect.assert_not_called()

    def test_cleanup(self):
        self.packet()
        self.ffb.close()
        self.device.erase_effect.assert_called_once_with(7)
        self.device.write.assert_any_call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)

    def test_h_bridge_sends_after_ffb_failure(self):
        # Execute real bridge loop with all I/O mocked, including select/CAN.
        self.device.reset_mock()
        self.device.write.side_effect = OSError('device failure')
        wheel = Mock(device=self.device, steer=.25, gear='D', raw=32767.5, minimum=0, maximum=65535)
        udp = Mock()
        udp.sendto.side_effect = [None, KeyboardInterrupt()]
        with patch.object(carla_bridge.socket, 'socket', return_value=udp), \
             patch.object(carla_bridge, 'LaneReceiver', return_value=self.rx), \
             patch.object(self.rx, 'open'), patch.object(self.rx, 'poll'), \
             patch.object(carla_bridge, 'open_can', side_effect=OSError()), \
             patch.object(carla_bridge, 'find_g29', return_value=wheel), \
             patch.object(carla_bridge.select, 'select', return_value=([], [], [])):
            self.rx.accept(json.dumps(dict(type='lane_ffb', sequence=1,
                lane_assist_enabled=True, lane_assist_active=True,
                lane_assist_state='ACTIVE', lane_assist_correction=.1)),
                carla_bridge.time.monotonic())
            carla_bridge.main()
        self.device.upload_effect.assert_not_called()
        self.device.write.assert_any_call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
        self.assertGreaterEqual(udp.sendto.call_count, 2)
        self.assertEqual(udp.sendto.call_args.args[1], ('192.168.50.1', 5005))
        self.assertEqual(udp.sendto.call_args.args[0], b'0,100,0.2500,D')


if __name__ == '__main__':
    unittest.main()
