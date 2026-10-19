import json
import unittest
from unittest.mock import Mock, patch

from evdev import ecodes
from carla_sim import g29_ffb as ffb
from carla_sim.carla_bridge import normalize_steering


class SoftwareCenterTests(unittest.TestCase):
    def setUp(self):
        self.device = Mock()
        self.device.capabilities.return_value = {ecodes.EV_FF: [ecodes.FF_CONSTANT, ecodes.FF_AUTOCENTER]}
        self.device.upload_effect.return_value = 2
        self.controller = ffb.WheelFFB(self.device)
        self.rx = ffb.LaneReceiver()

    def packet(self, state='ACTIVE', enabled=True, correction=.16, active=True):
        self.rx.accept(json.dumps(dict(type='lane_ffb', sequence=1,
            lane_assist_enabled=enabled, lane_assist_active=active,
            lane_assist_state=state, lane_assist_correction=correction)), 1.)

    def settle(self, steer, now=1.):
        for _ in range(40):
            self.controller.update(self.rx, now, steer)

    def test_default_lane_disabled_for_every_state(self):
        self.assertFalse(ffb.ENABLE_LANE_FFB)
        for state in ('OFF', 'STANDBY', 'ACTIVE', 'DRIVER_OVERRIDE'):
            for correction in (-.16, .16):
                self.rx = ffb.LaneReceiver()
                self.packet(state=state, correction=correction, enabled=state != 'OFF')
                self.settle(.02)
                self.assertEqual(self.controller.lane_applied, 0)
                self.assertEqual(self.controller.lane_target, 0)
                self.assertEqual(self.controller.applied, self.controller.center_applied)
                self.assertLessEqual(abs(self.controller.applied), int(32767*.35))

    def test_center_curve_and_small_deadzone(self):
        for steer in (0, .005, -.005):
            self.assertEqual(ffb.center_force_percent(steer), 0)
        self.assertLess(ffb.center_force_percent(.02), -6)
        self.assertGreater(ffb.center_force_percent(-.02), 6)
        self.assertAlmostEqual(ffb.center_force_percent(.02), -6.437185929648241)
        self.assertEqual(ffb.center_force_percent(1), -35)
        self.assertEqual(ffb.center_force_percent(-1), 35)

    def test_input_unchanged_and_ffb_deadzone_independent(self):
        self.assertEqual(ffb.normalize_center_steering(0, 0, 65535), -1)
        self.assertEqual(ffb.normalize_center_steering(32767.5, 0, 65535), 0)
        self.assertEqual(ffb.normalize_center_steering(65535, 0, 65535), 1)
        raw = 32767.5 * 1.007
        self.assertEqual(normalize_steering(raw, 0, 65535), 0)
        self.assertLess(ffb.center_force_percent(ffb.normalize_center_steering(raw, 0, 65535)), 0)

    def test_off_standby_override_timeout_and_no_carla_retain_center(self):
        cases = [('OFF', False, False, 1.), ('STANDBY', True, False, 1.),
                 ('DRIVER_OVERRIDE', True, True, 1.), ('ACTIVE', True, True, 1.201)]
        for state, enabled, active, now in cases:
            self.packet(state, enabled, active=active)
            self.settle(.02, now)
            self.assertEqual(self.controller.lane_applied, 0)
            self.assertLess(self.controller.center_applied, 0)
            self.assertEqual(self.controller.applied, self.controller.center_applied)
        self.rx.invalidate()
        self.settle(-.02)
        self.assertGreater(self.controller.applied, 0)

    @patch.object(ffb, "ENABLE_LANE_FFB", True)
    def test_combination_adds_and_subtracts(self):
        for correction in (-.16, .16):
            self.rx = ffb.LaneReceiver()
            self.packet(correction=correction)
            self.settle(1)
            self.assertEqual(self.controller.center_applied, int(-32767*.35))
            self.assertEqual(self.controller.lane_applied, int(32767*.15 * (1 if correction > 0 else -1)))
            self.assertEqual(self.controller.applied,
                self.controller.center_applied + self.controller.lane_applied)
        # Packet loss immediately removes the previously applied lane part.
        self.controller.update(self.rx, 1.201, 1)
        self.assertEqual(self.controller.lane_applied, 0)
        self.assertEqual(self.controller.applied, self.controller.center_applied)

    def test_clamp_and_slew(self):
        self.assertEqual(ffb.clamp_total_force(32767), int(32767*.5))
        self.assertEqual(ffb.clamp_total_force(-32767), -int(32767*.5))
        for steer in [1]*25 + [-1]*40 + [0]*25:
            previous = self.controller.center_applied
            self.controller.update(self.rx, 1., steer)
            self.assertLessEqual(abs(self.controller.center_applied-previous), int(32767*.02))
            self.assertLessEqual(abs(self.controller.applied), int(32767*.5))
        self.assertEqual(self.controller.applied, 0)

    @patch.object(ffb, "ENABLE_LANE_FFB", True)
    def test_independent_direction_signs(self):
        with patch.object(ffb, 'CENTER_FFB_DIRECTION_SIGN', -1):
            self.assertGreater(ffb.center_force_percent(.02), 0)
            self.packet()
            self.settle(.02)
            self.assertGreater(self.controller.lane_applied, 0)

    def test_native_off_and_disconnect_cleanup(self):
        self.device.write.assert_called_once_with(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
        self.settle(.02)
        self.device.upload_effect.side_effect = OSError('disconnect')
        self.controller.update(self.rx, 1., .02)
        self.assertTrue(self.controller.failed)
        self.assertEqual(self.controller.applied, 0)
        self.controller.close()
        self.device.erase_effect.assert_called_once_with(2)
        self.device.write.assert_any_call(ecodes.EV_FF, 2, 0)
        self.assertEqual(self.device.write.call_args.args, (ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0))

    def test_software_test_interrupt_cleans_up(self):
        wheel = Mock(device=self.device, raw=40000, minimum=0, maximum=65535)
        with patch('time.sleep', side_effect=KeyboardInterrupt()), patch.object(ffb.LaneReceiver, 'open') as udp_open:
            with self.assertRaises(KeyboardInterrupt):
                ffb.test_software_center(wheel)
        udp_open.assert_not_called()
        self.device.erase_effect.assert_called_once_with(2)
        self.assertEqual(self.device.write.call_args.args, (ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0))

    def test_software_test_normal_ten_second_exit(self):
        wheel = Mock(device=self.device, raw=40000, minimum=0, maximum=65535)
        # end=10, one update at 0, next loop reaches the deadline.
        with patch('time.monotonic', side_effect=[0, 0, 0, 0, 10]), patch('time.sleep'):
            ffb.test_software_center(wheel)
        self.device.erase_effect.assert_called_once_with(2)


if __name__ == '__main__':
    unittest.main()
