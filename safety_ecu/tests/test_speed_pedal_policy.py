import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from control.command_mapper import CommandMapper
from risk.bns import BNSCalculator, headway_assessment
from risk.directional_policy import DirectionalRiskPolicy
from risk.pms import PMSCalculator


class SpeedPedalPolicyTests(unittest.TestCase):
    def evaluate(self, policy, now, *, speed, distance, closing, accelerator=0,
                 accelerator_rate=0, brake=0, brake_rate=0, sequence=None,
                 obstacle=True, ttc=None):
        bns = BNSCalculator()
        pms = PMSCalculator()
        bns_score = bns.calculate(distance * 100 if obstacle else None,
                                  obstacle, speed, ttc,
                                  closing_speed_mps=closing if obstacle else 0,
                                  environment_connected=True)
        pms_score = pms.calculate(accelerator, accelerator_rate, True, 0,
                                  brake=brake, brake_rate=brake_rate)
        return policy.evaluate(
            gear="D", distance_m=distance if obstacle else None,
            speed_kmh=speed, closing_speed_mps=closing if obstacle else 0,
            ttc_s=ttc, obstacle_present=obstacle,
            bns=bns_score, bns_available=bns.available,
            pms=pms_score, pms_available=pms.available,
            now=now, brake=brake, brake_rate=brake_rate, accelerator=accelerator,
            sequence=sequence, obstacle_id=7 if obstacle else None,
            pedal_l1=pms.pedal_l1, pedal_l2=pms.pedal_l2,
            pedal_l3=pms.pedal_l3, pedal_conflict=pms.pedal_conflict,
        )[1]

    def confirmed(self, *, speed, distance, closing, **kwargs):
        policy = DirectionalRiskPolicy()
        self.evaluate(policy, 0, speed=speed, distance=distance, closing=closing,
                      sequence=1, **kwargs)
        level = self.evaluate(policy, 0.21, speed=speed, distance=distance,
                              closing=closing, sequence=1, **kwargs)
        return policy, level

    def test_same_distance_rises_with_speed(self):
        levels = []
        for speed in (10, 30, 60):
            policy, level = self.confirmed(speed=speed, distance=15, closing=0)
            levels.append(level)
            self.assertNotEqual(policy.risk_cause, "PEDAL_MISUSE")
        self.assertEqual(levels, [0, 1, 2])

    def test_stationary_obstacle_speed_rises_earlier(self):
        levels = []
        for speed in (10, 30, 60):
            closing = speed / 3.6
            policy, level = self.confirmed(speed=speed, distance=20,
                                           closing=closing, ttc=20 / closing)
            levels.append(level)
        self.assertEqual(levels, [0, 2, 3])

    def test_high_speed_short_stable_gap_is_warning_not_estop(self):
        policy, level = self.confirmed(speed=60, distance=10, closing=0)
        self.assertEqual(level, 2)
        self.assertEqual(policy.risk_cause, "HEADWAY")
        self.assertIsNone(policy.stopping_distance_m)
        self.assertEqual(CommandMapper().map(level, True)["motor"]["control_mode"], 1)
        self.assertFalse(CommandMapper().map(level, True)["motor"]["command_flags"] & 2)
        self.assertEqual(self.confirmed(speed=10, distance=10, closing=0)[1], 0)

    def test_stable_adequate_gap_stays_clear(self):
        self.assertEqual(self.confirmed(speed=60, distance=45, closing=0)[1], 0)

    def test_approaching_stationary_obstacle_rises_in_order(self):
        policy = DirectionalRiskPolicy()
        for now, distance, expected in ((0, 50, 0), (0.3, 30, 0),
                                        (0.51, 30, 1), (0.6, 20, 1),
                                        (0.81, 20, 2), (0.9, 10, 3)):
            closing = 30 / 3.6
            level = self.evaluate(policy, now, speed=30, distance=distance,
                                  closing=closing, ttc=distance / closing)
            self.assertEqual(level, expected)

    def test_fixed_gap_rises_as_speed_increases(self):
        policy = DirectionalRiskPolicy()
        for now, speed, expected in ((0, 10, 0), (0.3, 30, 0),
                                     (0.51, 30, 1), (0.6, 60, 1),
                                     (0.81, 60, 2)):
            self.assertEqual(self.evaluate(policy, now, speed=speed,
                                           distance=15, closing=0), expected)

    def test_leader_brakes_into_imminent_collision(self):
        policy, level = self.confirmed(speed=30, distance=15, closing=0)
        self.assertEqual(level, 1)
        self.assertEqual(self.evaluate(policy, 0.3, speed=30, distance=10,
                                       closing=8.3, ttc=1.2, sequence=2), 3)
        self.assertEqual(policy.risk_cause, "COLLISION")

    def test_normal_braking_never_becomes_pedal_misuse(self):
        for brake in (20, 50):
            policy, level = self.confirmed(speed=30, distance=15, closing=0,
                                           brake=brake, brake_rate=100)
            self.assertEqual(level, 1)
            self.assertEqual(policy.risk_cause, "HEADWAY")
            self.assertFalse(policy.pedal_misuse_active)
            self.assertTrue(policy.normal_braking)
        policy, level = self.confirmed(speed=30, distance=7, closing=5)
        self.assertEqual(level, 2)
        self.assertEqual(self.evaluate(policy, 0.31, speed=28, distance=8,
                                       closing=0.1, brake=20, sequence=2), 0)
        self.assertTrue(policy.normal_braking)
        self.assertEqual(policy.risk_cause, "NONE")

    def test_braking_does_not_cancel_imminent_collision(self):
        policy = DirectionalRiskPolicy()
        level = self.evaluate(policy, 0, speed=30, distance=2, closing=8,
                              ttc=0.25, brake=50, sequence=1)
        self.assertEqual(level, 3)
        self.assertFalse(policy.pedal_misuse_active)
        self.assertEqual(policy.risk_cause, "COLLISION")
        self.assertEqual(CommandMapper().map(level, True)["motor"]["command_flags"] & 2, 2)

    def test_acceleration_in_hazard_raises_misuse_not_automatic_estop(self):
        for accelerator, rate, expected in ((60, 260, 1), (90, 700, 2)):
            policy, level = self.confirmed(speed=30, distance=15, closing=0,
                                           accelerator=accelerator,
                                           accelerator_rate=rate)
            self.assertEqual(level, expected)
            self.assertTrue(policy.pedal_misuse_active)
            self.assertEqual(policy.risk_cause, "PEDAL_MISUSE")

    def test_collision_and_pedal_misuse_are_both_reported(self):
        policy, level = self.confirmed(speed=30, distance=7, closing=5,
                                       accelerator=90, accelerator_rate=700,
                                       ttc=2.2)
        self.assertEqual(level, 2)
        self.assertEqual(policy.risk_cause, "COLLISION_AND_PEDAL")
        self.assertTrue(policy.pedal_misuse_active)

    def test_conflicting_pedals_need_confirmation(self):
        policy = DirectionalRiskPolicy()
        self.assertEqual(self.evaluate(policy, 0, speed=30, distance=15,
                                       closing=0, accelerator=30, brake=30,
                                       sequence=1), 0)
        self.assertEqual(self.evaluate(policy, 0.21, speed=30, distance=15,
                                       closing=0, accelerator=30, brake=30,
                                       sequence=1), 2)
        self.assertEqual(policy.risk_cause, "PEDAL_MISUSE")

    def test_pedal_misuse_clears_when_driver_brakes_but_headway_remains(self):
        policy, level = self.confirmed(speed=30, distance=15, closing=0,
                                       accelerator=90, accelerator_rate=700)
        self.assertEqual(level, 2)
        self.assertEqual(policy.risk_cause, "PEDAL_MISUSE")
        self.assertEqual(self.evaluate(policy, 0.31, speed=28, distance=15,
                                       closing=0, brake=20, sequence=2), 2)
        self.assertFalse(policy.pedal_misuse_active)
        self.assertEqual(policy.risk_cause, "HEADWAY")
        self.assertEqual(self.evaluate(policy, 0.81, speed=28, distance=15,
                                       closing=0, brake=20, sequence=2), 1)

    def test_no_obstacle_acceleration_is_not_collision(self):
        policy, level = self.confirmed(speed=30, distance=0, closing=0,
                                       obstacle=False, accelerator=90,
                                       accelerator_rate=700)
        self.assertEqual(level, 0)
        self.assertGreater(policy.total_risk, 0)

    def test_stationary_headway_is_unavailable(self):
        self.assertEqual(headway_assessment(1, 0, 0)[0], 0)
        self.assertIsNone(headway_assessment(1, 0, 0)[1])
        self.assertEqual(headway_assessment(1, 60, -2)[0], 0)

    def test_unavailable_sensor_data_keeps_motor_stopped(self):
        command = CommandMapper().map(0, False)["motor"]
        self.assertEqual(command, dict(control_mode=2, output_limit=0, command_flags=1))


if __name__ == "__main__":
    unittest.main()
