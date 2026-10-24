import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from canbus.sender import CANSender
from control.command_mapper import CommandMapper


class PedalWarningOutputTest(unittest.TestCase):
    def setUp(self):
        self.mapper = CommandMapper()

    def command(self, level, accelerator, misuse=False, failsafe=False):
        return self.mapper.map(
            risk_level=level,
            risk_available=True,
            failsafe_active=failsafe,
            accelerator=accelerator,
            pedal_misuse_active=misuse,
        )

    def test_no_hazard_has_no_haptic(self):
        self.assertEqual(self.command(0, 0)["haptic"]["vibration_command"], 0)

    def test_unpressed_pedal_keeps_risk_and_motor_intervention(self):
        for level in (1, 2, 3):
            with self.subTest(level=level):
                command = self.command(level, 0, misuse=True)
                self.assertEqual(command["haptic"], dict(vibration_command=0, intensity=0, frequency=0))
                self.assertEqual(command["motor"]["control_mode"], 2 if level == 3 else 1 if level == 2 else 0)
        self.assertEqual(self.command(3, 0)["motor"]["command_flags"], 0x03)

    def test_normal_braking_and_accelerator_release_turn_haptic_off(self):
        self.assertEqual(self.command(2, 70, misuse=True)["haptic"]["vibration_command"], 1)
        self.assertEqual(self.command(2, 0, misuse=False)["haptic"], dict(vibration_command=0, intensity=0, frequency=0))
        self.assertEqual(self.command(0, 0)["haptic"]["vibration_command"], 0)

    def test_accelerator_alone_is_not_misuse(self):
        for level in (1, 2, 3):
            self.assertEqual(self.command(level, 70)["haptic"]["vibration_command"], 0)

    def test_deadband_rejects_pedal_noise(self):
        for accelerator in (0, 1, 2, self.mapper.ACCEL_HAPTIC_DEADBAND_PERCENT):
            self.assertEqual(self.command(2, accelerator, misuse=True)["haptic"]["vibration_command"], 0)
        self.assertEqual(self.command(2, self.mapper.ACCEL_HAPTIC_DEADBAND_PERCENT + 1, misuse=True)["haptic"]["vibration_command"], 1)

    def test_existing_vibration_patterns_and_failsafe(self):
        for level, intensity, frequency in ((1, 30, 20), (2, 45, 20), (3, 65, 30)):
            with self.subTest(level=level):
                self.assertEqual(self.command(level, 70, misuse=True)["haptic"],
                                 dict(vibration_command=1, intensity=intensity, frequency=frequency))
        self.assertEqual(self.command(3, 70, misuse=True, failsafe=True)["haptic"]["vibration_command"], 0)
        self.assertEqual(self.command(3, 70, misuse=True, failsafe=True)["motor"]["command_flags"], 0x03)

    def test_off_can_frame_preserves_risk_level(self):
        sender = CANSender()
        sent = []
        sender.send = lambda arbitration_id, data: sent.append((arbitration_id, data)) or True
        command = self.command(2, 0)
        self.assertTrue(sender.send_haptic_command(2, **command["haptic"], alive_counter=7))
        self.assertEqual(sent, [(0x110, bytes([2, 0, 0, 0, 0, 0, 0, 7]))])


if __name__ == "__main__":
    unittest.main()
