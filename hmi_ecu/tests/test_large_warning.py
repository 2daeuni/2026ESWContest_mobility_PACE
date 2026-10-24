import unittest

from hmi.audio import choose_pattern
from hmi.state_merge import apply_carla, merge


def state(level, accelerator=0, brake=0, haptic=0, cause="COLLISION",
          normal_braking=False, gear="D"):
    udp = {"data": {
        "risk_level": level,
        "risk_available": True,
        "risk_cause": cause,
        "normal_braking": normal_braking,
        "pedal_connected": True,
        "accelerator": accelerator,
        "brake": brake,
        "haptic_vibration_command": haptic,
        "gear": gear,
    }, "t": 1.0}
    return merge({}, udp, now=1.0)


class LargeWarningTest(unittest.TestCase):
    def test_directional_warning_tracks_gear_without_changing_risk(self):
        for gear, title in (("D", "전방 차량 주의"), ("R", "후방 차량 주의"),
                            ("D", "전방 차량 주의"), ("P", "전방 차량 주의")):
            with self.subTest(gear=gear, title=title):
                merged = state(1, gear=gear)
                self.assertEqual(merged["gear"], gear)
                self.assertEqual(merged["banner"]["title"], title)
                self.assertEqual(merged["risk_level"], 1)
                self.assertEqual(choose_pattern(merged), "caution")

        self.assertEqual(state(0, gear="R")["banner"]["severity"], "ok")
        self.assertEqual(state(1, gear="R")["banner"]["title"], "후방 차량 주의")

    def test_scene_gear_fallback_refreshes_warning(self):
        merged = state(1, gear=None)
        self.assertEqual(merged["banner"]["title"], "전방 차량 주의")
        apply_carla(merged, {"ego": {"gear": "R"}})
        self.assertEqual(merged["banner"]["title"], "후방 차량 주의")
        self.assertEqual(merged["risk_level"], 1)

    def test_level_messages_and_sound(self):
        for level, title, sound in (
            (1, "전방 차량 주의", "caution"),
            (2, "브레이크를 밟으세요", "warning"),
            (3, "충돌 위험! 즉시 제동하세요", "danger"),
        ):
            with self.subTest(level=level):
                merged = state(level)
                self.assertEqual(merged["banner"]["title"], title)
                self.assertEqual(choose_pattern(merged), sound)
                self.assertFalse(merged["haptic_cmd_on"])

    def test_pedal_misuse_uses_same_risk_levels(self):
        for level, title in ((2, "가속페달에서 발을 떼세요"),
                             (3, "페달 오조작 위험! 즉시 제동하세요")):
            merged = state(level, accelerator=70, haptic=1,
                           cause="COLLISION_AND_PEDAL")
            self.assertEqual(merged["banner"]["title"], title)
            self.assertTrue(merged["haptic_cmd_on"])

    def test_braking_recovery_hides_warning_but_collision_does_not(self):
        recovered = state(2, brake=40, cause="RECOVERING", normal_braking=True)
        self.assertEqual(recovered["banner"]["severity"], "ok")
        self.assertIsNone(choose_pattern(recovered))
        danger = state(3, brake=40, cause="COLLISION", normal_braking=True)
        self.assertEqual(danger["banner"]["severity"], "danger")
        self.assertEqual(choose_pattern(danger), "danger")

    def test_scene_lead_does_not_replace_fresh_safety_no_obstacle(self):
        clear = state(0)
        apply_carla(clear, {"lead": {"distance": 4.0}})
        self.assertIsNone(clear["distance_m"])
        self.assertEqual(clear["risk_level"], 0)
        self.assertIsNone(choose_pattern(clear))

        offline = {"distance_m": None, "risk_source": "-", "link_carla": False}
        apply_carla(offline, {"lead": {"distance": 4.0}})
        self.assertEqual(offline["distance_m"], 4.0)


if __name__ == "__main__":
    unittest.main()
