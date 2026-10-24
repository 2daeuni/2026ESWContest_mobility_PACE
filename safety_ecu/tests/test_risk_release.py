from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from control.command_mapper import CommandMapper
from control.failsafe import FailSafeManager
from hmi_sender import build_packet
from risk.directional_policy import DirectionalRiskPolicy


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.policy = DirectionalRiskPolicy()
        self.mapper = CommandMapper()

    def step(self, now, *, distance=0.0, speed=10.0, closing=5.0,
             ttc=0.0, brake=0, accelerator=0, present=True, sequence=None,
             gear="D", available=True):
        return self.policy.evaluate(
            gear=gear, distance_m=distance if present else None,
            speed_kmh=speed, closing_speed_mps=closing if present else 0.0,
            ttc_s=ttc if present else None, obstacle_present=present,
            bns=1.0 if present else 0.0, bns_available=available,
            pms=0.0, pms_available=True, now=now, brake=brake,
            accelerator=accelerator, sequence=sequence,
            obstacle_id=7 if present else None,
        )[1]

    def test_l3_controlled_stop_directly_releases_and_sends_off(self):
        self.assertEqual(self.step(0.0, sequence=1), 3)
        self.assertEqual(self.step(0.1, speed=0.1, closing=0.0, brake=30,
                                   sequence=2), 0)
        self.assertEqual(self.policy.reason, "BRAKE_RELEASE")
        self.assertEqual(self.policy.bns_score, 0.0)
        commands = self.mapper.map(0, True)
        self.assertEqual(commands["haptic"], dict(
            vibration_command=0, intensity=0, frequency=0))
        with patch.dict(sys.modules, {"can": Mock()}):
            from canbus.sender import CANSender
        sender = CANSender()
        sent = []
        sender.send = lambda arbitration_id, data: sent.append((arbitration_id, data)) or True
        self.assertTrue(sender.send_haptic_command(0, **commands["haptic"], alive_counter=4))
        self.assertEqual(sent, [(0x110, bytes([0, 0, 0, 0, 0, 0, 0, 4]))])
        state = SimpleNamespace(**{
            **{field: 0 for field in (
                "accelerator", "brake", "accelerator_rate", "brake_rate",
                "pedal_sensor_status", "haptic_fault_code", "bns", "pms",
                "risk_level", "total_risk", "carla_distance_cm",
                "carla_vehicle_speed", "carla_closing_speed_mps")},
            **{field: True for field in (
                "pedal_connected", "haptic_connected", "haptic_available",
                "carla_connected", "carla_distance_valid")},
            "tof_connected": False, "carla_ttc": None,
        })
        packet = build_packet(
            state, seq=1, input_mode="CARLA", bns_available=True,
            pms_available=True, risk_available=True, failsafe_active=False,
            failsafe_reason=None, commands=commands, motor_connected=True)
        self.assertEqual(packet["risk_level"], 0)
        self.assertEqual(packet["haptic_vibration_command"], 0)
        self.assertEqual(packet["haptic_intensity"], 0)
        self.assertEqual(packet["motor_control_mode"], 0)

    def test_l2_braking_and_measured_deceleration_directly_releases(self):
        self.assertEqual(self.step(0.0, distance=7, speed=30, closing=5,
                                   ttc=2.2, sequence=1), 0)
        self.assertEqual(self.step(0.2, distance=7, speed=30, closing=5,
                                   ttc=2.2, sequence=1), 2)
        self.assertEqual(self.step(0.3, distance=8, speed=28, closing=0.1,
                                   ttc=None, brake=10, sequence=2), 0)

    def test_brake_does_not_release_persistent_collision_risk(self):
        self.assertEqual(self.step(0.0, sequence=1), 3)
        self.assertEqual(self.step(0.1, distance=1, speed=20, closing=5,
                                   ttc=0.2, brake=30, sequence=2), 3)
        self.assertEqual(self.mapper.map(3, True)["motor"]["control_mode"], 2)

    def test_stopped_with_accelerator_conflict_stays_l3(self):
        self.assertEqual(self.step(0.0, sequence=1), 3)
        self.assertEqual(self.step(0.1, speed=0.0, closing=0.0, brake=30,
                                   accelerator=30, sequence=2), 3)

    def test_release_then_reappearance_uses_existing_rise_policy(self):
        self.assertEqual(self.step(0.0, sequence=1), 3)
        self.assertEqual(self.step(0.1, speed=0.0, closing=0.0, brake=30,
                                   sequence=2), 0)
        self.assertEqual(self.step(0.2, distance=1, speed=5, closing=3,
                                   ttc=0.3, sequence=3), 3)

    def test_gear_change_alone_does_not_use_brake_fast_release(self):
        self.assertEqual(self.step(0.0, sequence=1), 3)
        self.assertEqual(self.step(0.1, gear="R", present=False, speed=0.0,
                                   brake=30, sequence=2), 3)
        self.assertEqual(self.step(0.2, gear="R", present=False, speed=0.0,
                                   brake=30, sequence=3), 3)

    def test_zero_ttc_at_stationary_contact_is_not_imminent_if_braking(self):
        self.assertEqual(self.step(0.0, speed=0.0, closing=0.0, brake=30,
                                   sequence=1), 0)
        self.assertEqual(self.policy.reason, "CONTROLLED_STOP")
        self.assertEqual(self.step(0.1, speed=0.0, closing=0.3, brake=30,
                                   sequence=2), 3)

    def test_sensor_timeout_retains_failsafe_stop(self):
        self.assertEqual(self.step(0.0, sequence=1), 3)
        self.assertEqual(self.step(0.1, available=False, sequence=2), 0)
        self.assertFalse(self.policy.available)
        state = SimpleNamespace(carla_connected=False, pedal_connected=True,
                                pedal_sensor_status=0, carla_link_state="LOST",
                                pedal_ever_connected=True)
        failsafe = FailSafeManager()
        self.assertTrue(failsafe.evaluate(state, input_mode="CARLA"))
        commands = self.mapper.map(0, False, failsafe_active=failsafe.active)
        self.assertEqual(commands["motor"], dict(
            control_mode=2, output_limit=0, command_flags=3))

    def test_without_valid_braking_downward_hysteresis_remains(self):
        self.assertEqual(self.step(0.0, sequence=1), 3)
        self.assertEqual(self.step(0.1, present=False, sequence=2), 3)
        self.assertEqual(self.step(0.59, present=False, sequence=3), 3)
        self.assertEqual(self.step(0.6, present=False, sequence=4), 2)


if __name__ == "__main__":
    unittest.main()
