import unittest
from unittest.mock import Mock, patch, call

from evdev import ecodes
from carla_sim.g29_ffb import test_center, test_direction, cleanup_test, main


class HardwareCommandTests(unittest.TestCase):
    def test_center_only_and_cleanup(self):
        device = Mock()
        with patch('time.sleep') as sleep:
            test_center(device)
        sleep.assert_called_once_with(10)
        self.assertEqual(device.write.call_args_list, [
            call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, int(65535*.35)),
            call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)])
        device.upload_effect.assert_not_called()
        device.erase_effect.assert_not_called()

    def test_center_interrupt_and_write_error(self):
        for error in (KeyboardInterrupt(), OSError('lost device')):
            device = Mock()
            with patch('time.sleep', side_effect=error):
                with self.assertRaises(type(error)):
                    test_center(device)
            device.write.assert_any_call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
        device = Mock()
        device.write.side_effect = [OSError('partial write'), None]
        with self.assertRaises(OSError):
            test_center(device)
        self.assertEqual(device.write.call_args.args, (ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0))

    def test_direction_requires_yes(self):
        device = Mock()
        with patch('builtins.input', return_value='NO'):
            test_direction(device)
        self.assertEqual(device.mock_calls, [])

    def test_direction_pulses_and_neutral(self):
        device = Mock()
        device.upload_effect.return_value = 3
        with patch('builtins.input', return_value='YES'), patch('time.sleep') as sleep:
            test_direction(device)
        self.assertEqual(sleep.call_args_list, [call(1), call(1), call(1.5), call(1), call(1)])
        effects = [c.args[0] for c in device.upload_effect.call_args_list]
        self.assertEqual([e.u.ff_constant_effect.level for e in effects],
                         [4915, -4915])
        self.assertEqual([e.ff_replay.length for e in effects], [1000, 1000])
        device.erase_effect.assert_called_once_with(3)
        self.assertEqual(device.write.call_args_list, [
            call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0),
            call(ecodes.EV_FF, 3, 1), call(ecodes.EV_FF, 3, 0),
            call(ecodes.EV_FF, 3, 1), call(ecodes.EV_FF, 3, 0),
            call(ecodes.EV_FF, 3, 0),
            call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)])
        self.assertEqual(device.write.call_args.args, (ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0))

    def test_direction_interrupt_cleanup(self):
        device = Mock()
        device.upload_effect.return_value = 3
        with patch('builtins.input', return_value='YES'), \
             patch('time.sleep', side_effect=[None, KeyboardInterrupt()]):
            with self.assertRaises(KeyboardInterrupt):
                test_direction(device)
        device.write.assert_any_call(ecodes.EV_FF, 3, 0)
        device.erase_effect.assert_called_once_with(3)
        device.write.assert_any_call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)

    def test_direction_upload_and_play_errors(self):
        for fail_at in ('upload', 'play'):
            device = Mock()
            device.upload_effect.return_value = 3
            if fail_at == 'upload':
                device.upload_effect.side_effect = OSError('upload failed')
            else:
                device.write.side_effect = [None, OSError('play failed'), None, None]
            with patch('builtins.input', return_value='YES'), patch('time.sleep'), \
                 patch('builtins.print') as output:
                with self.assertRaises(OSError):
                    test_direction(device)
            logs = '\n'.join(str(c.args[0]) for c in output.call_args_list)
            self.assertIn('exception during effect ' + fail_at, logs)
            self.assertIn('OSError', logs)
            device.write.assert_any_call(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
            if fail_at == 'play':
                device.erase_effect.assert_called_once_with(3)
                self.assertIn('returned effect id=3', logs)

    def test_direction_cleanup_failure_logged(self):
        device = Mock()
        device.upload_effect.return_value = 3
        device.erase_effect.side_effect = OSError('erase failed')
        with patch('builtins.input', return_value='YES'), patch('time.sleep'), \
             patch('builtins.print') as output:
            test_direction(device)
        logs = '\n'.join(str(c.args[0]) for c in output.call_args_list)
        self.assertIn('cleanup incomplete', logs)
        self.assertNotIn('[FFB test] cleanup complete', logs)
        self.assertEqual(device.write.call_args.args, (ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0))

    def test_cleanup_attempts_all_after_error(self):
        device = Mock()
        device.write.side_effect = OSError('disconnect')
        cleanup_test(device, 3, True)
        device.erase_effect.assert_called_once_with(3)
        self.assertEqual(device.write.call_count, 2)

    def test_diagnose_never_writes(self):
        device = Mock()
        device.info.vendor, device.info.product = 0x046D, 0xC24F
        device.capabilities.return_value = {ecodes.EV_FF: [ecodes.FF_AUTOCENTER]}
        with patch('carla_sim.carla_bridge.find_g29', return_value=Mock(device=device)):
            self.assertEqual(main(['--diagnose']), 0)
        device.write.assert_not_called()
        device.upload_effect.assert_not_called()
        device.close.assert_called_once()

    def test_unsupported_refused(self):
        device = Mock()
        device.info.vendor, device.info.product = 0x046D, 0xC24F
        device.capabilities.return_value = {}
        with patch('carla_sim.carla_bridge.find_g29', return_value=Mock(device=device)):
            self.assertEqual(main(['--test-center']), 1)
        device.write.assert_not_called()


if __name__ == '__main__':
    unittest.main()
