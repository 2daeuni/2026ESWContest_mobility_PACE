"""근거리 레이더 방식 장애물 탐지 검증 (CARLA 없이 가짜 객체)"""
import math
import os
import sys
import types
import unittest
from unittest.mock import patch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import obstacle_sensing as osn  # noqa: E402


class V:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class TF:
    def __init__(self, x, y, z, yaw):
        self.location = V(x, y, z)
        self.rotation = types.SimpleNamespace(yaw=yaw)

    def transform(self, loc):
        return V(self.location.x + loc.x, self.location.y + loc.y, self.location.z + loc.z)


class Actor:
    def __init__(self, aid, type_id, x, y, z=0.0, yaw=0.0, v=(0, 0), ext=(2.3, 0.95, 0.75)):
        self.id, self.type_id = aid, type_id
        self._tf = TF(x, y, z, yaw)
        self.bounding_box = types.SimpleNamespace(location=V(0, 0, ext[2]), extent=V(*ext))
        self._v = V(v[0], v[1], 0)

    def get_transform(self):
        return self._tf

    def get_velocity(self):
        return self._v


class Actors(list):
    def filter(self, pat):
        pre = pat.rstrip("*")
        return [a for a in self if a.type_id.startswith(pre)]


class World:
    id = 1

    def __init__(self, actors):
        self._a = Actors(actors)

    def get_actors(self):
        return self._a


class Statics:
    def __init__(self, items):
        self.items = items

    def near(self, x, y, r):
        return self.items


def wall(x, y, hl, hw, z0=0.0, z1=3.0, yaw=0.0, label="Walls"):
    return {"id": osn.STATIC_OBSTACLE_ID_BASE + 11, "label": label, "x": x, "y": y, "yaw": yaw,
            "hl": hl, "hw": hw, "z0": z0, "z1": z1, "r": math.hypot(hl, hw)}


def run(actors, statics=(), reverse=False):
    w = World(actors)
    return osn.find_obstacle_for_safety(None, w, actors[0], reverse=reverse, statics=Statics(list(statics)))


def ego(v=(10, 0), z=0.0):
    return Actor(1, "vehicle.tesla.model3", 0, 0, z, 0, v=v)


class Tests(unittest.TestCase):
    def test_extra_target_filters_out_vehicles_before_selection(self):
        candidates = [
            {"id": 2, "kind": "vehicle", "gap": 3.0, "closing": 10.0, "in_path": True},
            {"id": 3, "kind": "walker", "gap": 5.0, "closing": 5.0, "in_path": True},
            {"id": 4, "kind": "walker", "gap": 8.0, "closing": 20.0, "in_path": True},
        ]
        with patch.object(osn, "scan", return_value=candidates):
            result = osn.find_path_obstacle_for_safety(
                None, None, None, kinds=("walker", "static")
            )
        self.assertEqual(result["obstacle_id"], 3)
        self.assertEqual(result["kind"], "walker")

    def test_following_car_reported_without_closing(self):
        r = run([ego((10, 0)), Actor(2, "vehicle.audi.a2", 20, 0, v=(10, 0))])
        self.assertEqual(r["obstacle_id"], 2)
        self.assertAlmostEqual(r["distance_m"], 20 - 4.6, places=2)
        self.assertIsNone(r["ttc_s"])
        self.assertEqual(r["direction"], "전방")

    def test_parallel_car_beside_not_reported(self):
        # 옆 차선 같은 속도 → 경로 밖 + 다가오지 않음 → 보내지 않음
        self.assertIsNone(run([ego((10, 0)), Actor(2, "vehicle.audi.a2", 0, 3.5, v=(10, 0))]))

    def test_side_car_closing_reported_with_direction(self):
        r = run([ego((0, 0)), Actor(2, "vehicle.audi.a2", 0, 6, yaw=90, v=(0, -5))])
        self.assertEqual(r["obstacle_id"], 2)
        self.assertEqual(r["direction"], "우측")
        self.assertGreater(r["closing_speed_mps"], 4.9)
        self.assertAlmostEqual(r["ttc_s"], r["distance_m"] / r["closing_speed_mps"])

    def test_under_bridge_vehicle_ignored(self):
        self.assertIsNone(run([ego((10, 0), z=8.0), Actor(2, "vehicle.audi.a2", 15, 0, z=0.0)]))

    def test_wall_ahead_static(self):
        r = run([ego((5, 0))], [wall(15, 0, 0.3, 4.0)])
        self.assertEqual(r["kind"], "static")
        self.assertAlmostEqual(r["distance_m"], 15 - 0.3 - 2.3, places=2)
        self.assertAlmostEqual(r["closing_speed_mps"], 5.0, places=2)

    def test_wall_beside_while_parked_contact(self):
        # 정차 중 옆 0.5 m 벽 → 접촉 직전으로 보고
        r = run([ego((0, 0))], [wall(0, 1.45, 3.0, 0.1)])
        self.assertEqual(r["direction"], "우측")
        self.assertLess(r["distance_m"], 1.0)

    def test_box_covering_road_ignored(self):
        # 곡선 가드레일처럼 도로까지 덮는 큰 박스 안에 자차가 있으면 장애물 아님
        self.assertIsNone(run([ego((5, 0))], [wall(3, 2, 15, 10, label="GuardRail")]))

    def test_overhead_structure_ignored(self):
        self.assertIsNone(run([ego((5, 0))], [wall(10, 0, 3, 3, z0=4.5, z1=6.0)]))

    def test_reverse_rear_in_path(self):
        r = run([ego((-2, 0)), Actor(2, "vehicle.audi.a2", 20, 0, v=(0, 0)), Actor(3, "vehicle.audi.a2", -9, 0)],
                reverse=True)
        self.assertEqual(r["obstacle_id"], 3)
        self.assertEqual(r["direction"], "후방")
        self.assertAlmostEqual(r["closing_speed_mps"], 2.0, places=2)

    def test_pedestrian(self):
        r = run([ego((8, 0)), Actor(3, "walker.pedestrian.0001", 12, 0.5, ext=(0.3, 0.3, 0.9))])
        self.assertEqual(r["kind"], "walker")

    def test_poly_gap_and_direction(self):
        a = osn.box_corners(0, 0, 0, 1, 1)
        b = osn.box_corners(5, 0, 0, 1, 1)
        self.assertAlmostEqual(osn.poly_gap(a, b)[0], 3.0)
        self.assertEqual(osn.direction_name(1, 0, 0, 1, -1, -1), "후방 좌측")


if __name__ == "__main__":
    unittest.main()
