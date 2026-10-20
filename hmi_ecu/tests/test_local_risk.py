"""Safety ECU 없이 CARLA 만 있을 때 HMI 자체 판정"""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "carla_tools"))

from hmi import state_merge  # noqa: E402
from hmi.local_risk import LocalRisk  # noqa: E402
import map_stream as ms  # noqa: E402


class LocalRiskTest(unittest.TestCase):
    def test_levels_rise_while_approaching_with_throttle(self):
        lr = LocalRisk()
        levels = []
        gap = 100.0
        for _ in range(300):
            gap -= 16.7 * 0.05
            out = lr.step(0.05, max(gap, 0.5), 16.7, 60.0, 0.0)
            levels.append(out["risk_level"])
        self.assertEqual(max(levels), 3)
        self.assertEqual(levels[0], 0)

    def test_no_risk_when_nothing_ahead(self):
        lr = LocalRisk()
        for _ in range(100):
            out = lr.step(0.05, None, None, 80.0, 0.0)
        self.assertLessEqual(out["risk_level"], 0)

    def test_collision_forces_l3(self):
        lr = LocalRisk()
        self.assertEqual(lr.step(0.05, None, None, 0, 0, collided=True)["risk_level"], 3)


class InputsTest(unittest.TestCase):
    def test_static_obstacle_closer_than_vehicle(self):
        s = {"pedal_source": None, "accelerator": None}
        pkt = {"ego": {"throttle": 0.5, "brake": 0.0, "speed_kmh": 36}, "ego_v_fwd": 10.0,
               "lead": {"distance": 40, "closing": 2.0}, "front_static": 12.0}
        inp = state_merge.carla_inputs(s, pkt)
        self.assertEqual(inp["kind"], "static")
        self.assertEqual(inp["dist"], 12.0)
        self.assertEqual(inp["closing"], 10.0)
        self.assertEqual(inp["acc"], 50.0)
        self.assertEqual(inp["pedal_src"], "CARLA")

    def test_reverse_uses_rear(self):
        s = {"pedal_source": None, "accelerator": None, "gear": "R"}
        pkt = {"ego": {"throttle": 0.6, "brake": 0.0}, "ego_v_fwd": -2.0,
               "lead": {"distance": 5, "closing": 3.0}, "front_static": 3.0,
               "rear": {"id": 9, "distance": 1.4}, "rear_static": 6.0}
        inp = state_merge.carla_inputs(s, pkt)
        self.assertEqual(inp["dist"], 1.4)
        self.assertEqual(inp["closing"], 2.0)
        self.assertEqual(inp["kind"], "vehicle")
        st = {}
        state_merge.apply_carla(st, pkt)
        self.assertEqual(st["rear_distance_m"], 1.4)

    def test_radar_packet_wins(self):
        # 브리지 레이더 결과가 있으면 차선 기반 lead / front_static 대신 그것만 사용 (옆에서 오는 차 포함)
        s = {"pedal_source": None, "accelerator": None, "gear": "D"}
        pkt = {"ego": {"throttle": 0.5, "brake": 0.0}, "lead": {"distance": 30, "closing": 0.0},
               "front_static": 12.0,
               "radar": {"id": 7, "distance": 4.2, "closing": 3.0, "kind": "vehicle", "dir": "우측"}}
        inp = state_merge.carla_inputs(s, pkt)
        self.assertEqual((inp["dist"], inp["closing"], inp["kind"]), (4.2, 3.0, "vehicle"))
        self.assertEqual(s["obstacle_dir"], "우측")
        pkt["radar"] = None
        inp = state_merge.carla_inputs(s, pkt)
        self.assertIsNone(inp["dist"])
        self.assertIsNone(s["obstacle_dir"])

    def test_real_pedal_wins(self):
        s = {"pedal_source": "CAN", "accelerator": 33.0, "brake": 0.0, "accelerator_rate": 5.0}
        inp = state_merge.carla_inputs(s, {"ego": {"throttle": 1.0}})
        self.assertEqual(inp["acc"], 33.0)
        self.assertEqual(inp["pedal_src"], "CAN")


class FrontStaticTest(unittest.TestCase):
    def test_wall_ahead_and_beside(self):
        boxes = [{"x": 20.0, "y": 0.0, "yaw": 0.0, "l": 1.0, "w": 6.0, "h": 3.0},     # 정면 벽
                 {"x": 8.0, "y": 6.0, "yaw": 0.0, "l": 10.0, "w": 2.0, "h": 3.0}]     # 옆 건물
        gap = ms.front_static_gap(boxes, (0.0, 0.0, 0.0), 4.8)
        self.assertAlmostEqual(gap, 20.0 - 0.5 - 2.4, places=1)

    def test_follows_lane_center(self):
        # 차선이 오른쪽으로 휘면 정면 직선 위 건물은 경로 밖
        boxes = [{"x": 30.0, "y": 0.0, "yaw": 0.0, "l": 2.0, "w": 2.0, "h": 3.0}]
        center = [[0, 0], [10, 0], [20, 3], [30, 8], [40, 14]]
        self.assertIsNone(ms.front_static_gap(boxes, (0.0, 0.0, 0.0), 4.8, center=center))
        self.assertIsNotNone(ms.front_static_gap(boxes, (0.0, 0.0, 0.0), 4.8))


if __name__ == "__main__":
    unittest.main()
