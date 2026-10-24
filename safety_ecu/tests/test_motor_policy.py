"""Distance boundaries and exact MOTOR_COMMAND wire policy, without hardware."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from risk.bns import BNSCalculator
from control.command_mapper import CommandMapper
from canbus.sender import CANSender


class MotorPolicyTests(unittest.TestCase):
    def test_distance_boundaries(self):
        bns = BNSCalculator()
        for level, boundary in ((1, 25), (2, 15), (3, 10)):
            for distance, expected in ((boundary - .01, True), (boundary, True),
                                       (boundary + .01, False)):
                with self.subTest(level=level, distance=distance):
                    bns.calculate(distance * 100, True, 50, None)
                    self.assertEqual(getattr(bns, f'distance_l{level}'), expected)

    def test_all_motor_packets_and_failsafe_priority(self):
        mapper = CommandMapper()
        sender = CANSender()
        sender.bus = Mock()
        for risk, expected in enumerate(((0, 100), (0, 100), (1, 30), (2, 0))):
            for available, failsafe in ((True, False), (True, True), (False, True)):
                with self.subTest(risk=risk, available=available, failsafe=failsafe):
                    motor = mapper.map(risk, available, failsafe)['motor']
                    mode, limit = (2, 0) if failsafe else expected
                    flags = 3 if failsafe or risk == 3 else 1
                    self.assertEqual(motor, dict(control_mode=mode, output_limit=limit,
                                                 command_flags=flags))
                    sender.send_motor_command(motor['control_mode'], motor['output_limit'],
                                              risk, motor['command_flags'], 0xA5)
                    frame = sender.bus.send.call_args.args[0]
                    self.assertEqual(frame.arbitration_id, 0x120)
                    self.assertEqual(frame.dlc, 8)
                    self.assertFalse(frame.is_extended_id)
                    self.assertEqual(bytes(frame.data), bytes([mode, limit, risk, flags,
                                                             0, 0, 0, 0xA5]))
