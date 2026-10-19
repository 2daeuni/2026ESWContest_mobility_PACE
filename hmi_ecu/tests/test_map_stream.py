"""CARLA 실제 도로 맵 스트림 검증 (carla 없이)"""

import math
import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "carla_tools"))

import map_stream as ms  # noqa: E402
from hmi import scene  # noqa: E402


class XfTest(unittest.TestCase):
    def test_anchor_to_ego_matches_direct(self):
        rnd = random.Random(3)
        for _ in range(200):
            anchor = (rnd.uniform(-300, 300), rnd.uniform(-300, 300), rnd.uniform(-180, 180))
            ego = (anchor[0] + rnd.uniform(-50, 50), anchor[1] + rnd.uniform(-50, 50), rnd.uniform(-180, 180))
            px, py = rnd.uniform(-400, 400), rnd.uniform(-400, 400)
            ax, ay = ms.to_frame(*anchor, px, py)
            xf = ms.map_xf(anchor, ego)
            got = ms.apply_xf(xf, ax, ay)
            want = ms.to_frame(*ego, px, py)
            self.assertAlmostEqual(got[0], want[0], places=1)
            self.assertAlmostEqual(got[1], want[1], places=1)


class CropTest(unittest.TestCase):
    def setUp(self):
        self.mc = ms.MapCache()
        xs = [float(x) for x in range(0, 400, 2)]
        self.mc.lanes.append({"L": [(x, 0.0) for x in xs], "R": [(x, 3.5) for x in xs],
                              "lm": ("double", "y"), "rm": ("dash", "w"), "j": False})
        self.mc.boxes.append({"x": 30, "y": 15, "yaw": 0, "l": 20, "w": 10, "h": 12})
        self.mc.boxes.append({"x": 390, "y": 15, "yaw": 0, "l": 20, "w": 10, "h": 12})

    def test_crop_radius_and_frame(self):
        lanes, boxes = self.mc.crop((20.0, 1.75, 0.0), radius=50)
        self.assertEqual(len(lanes), 1)
        xs = [p[0] for p in lanes[0]["L"]]
        self.assertTrue(min(xs) >= -51 and max(xs) <= 51)
        self.assertEqual(len(boxes), 1)
        self.assertAlmostEqual(boxes[0]["x"], 10.0)
        self.assertAlmostEqual(boxes[0]["y"], 13.25)

    def test_packets_split_and_rejoin(self):
        lanes, boxes = self.mc.crop((200.0, 0.0, 0.0), radius=300)
        pkts = ms.map_packets(5, lanes, boxes, max_bytes=800)
        self.assertGreater(len(pkts), 1)
        self.assertTrue(all(p["parts"] == len(pkts) and p["ver"] == 5 for p in pkts))
        back_l = sum((p["lanes"] for p in pkts), [])
        self.assertEqual(sum(len(l["L"]) for l in back_l), sum(len(l["L"]) for l in lanes))

    def test_map_scene_markings(self):
        lanes, boxes = self.mc.crop((20.0, 1.75, 0.0), radius=50)
        m = scene.build_map_scene(lanes, boxes)
        self.assertEqual(len(m["bands"]), 1)
        self.assertEqual(len(m["yellow"]), 2)          # 이중선
        self.assertEqual(len(m["white_dash"]), 1)
        self.assertLessEqual(m["boxes"][0]["h"], 3.0)

    def test_scene_with_map_offroad(self):
        carla = {"lane": {"center": [[-10, 0], [80, 0]], "width": 3.5, "on_road": False},
                 "vehicles": [], "lead": {}, "map_ver": 1, "map_xf": {"x": 0, "y": 0, "rot": 0},
                 "collision": {"n": 1, "age": 0.3, "with": "static"}}
        sc = scene.build_scene(scene.SpriteLibrary(), carla=carla)
        self.assertTrue(sc["has_map"])
        self.assertEqual(sc["lines"], [])
        self.assertFalse(sc["on_road"])
        self.assertTrue(sc["collided"])


class FakeTownTest(unittest.TestCase):
    def test_build(self):
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        import fake_carla_town
        mc = fake_carla_town.build_town()
        lanes, boxes = mc.crop((0.0, 5.25, 0.0))
        self.assertTrue(lanes and boxes)
        for p in ms.map_packets(1, lanes, boxes):
            import json
            self.assertLess(len(json.dumps(p, separators=(",", ":"))), 60000)


if __name__ == "__main__":
    unittest.main()
