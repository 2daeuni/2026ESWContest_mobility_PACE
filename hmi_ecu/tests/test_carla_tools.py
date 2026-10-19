"""
CARLA 서버 없이 carla_tools 검증 (가짜 carla 객체 사용).
python3 -m unittest discover -s tests
"""

import math
import os
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "carla_tools"))

import carla_hmi_bridge as bridge  # noqa: E402
from hmi import scene  # noqa: E402


# ------------------------------------------------------------------
# 가짜 CARLA: 월드 +y 방향으로 뻗은 직선 도로, 차선 폭 3.5
# ------------------------------------------------------------------
class V3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    def distance(self, o):
        return math.dist((self.x, self.y, self.z), (o.x, o.y, o.z))


class Rot:
    def __init__(self, yaw=0.0):
        self.yaw, self.pitch, self.roll = yaw, 0.0, 0.0


class TF:
    def __init__(self, x, y, yaw):
        self.location = V3(x, y)
        self.rotation = Rot(yaw)

    def get_matrix(self):
        # Independent mock: compose elementary rotations in CARLA convention.
        yaw, pitch, roll = map(math.radians, (self.rotation.yaw, -self.rotation.pitch, -self.rotation.roll))
        cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
        rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
        ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
        rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]

        def mul(a, b):
            return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        r = mul(mul(rz, ry), rx)
        return [r[i] + [getattr(self.location, 'xyz'[i])] for i in range(3)] + [[0, 0, 0, 1]]

    def get_inverse_matrix(self):
        matrix = self.get_matrix()
        return [[matrix[j][i] for j in range(3)] +
                [-sum(matrix[k][i] * matrix[k][3] for k in range(3))]
                for i in range(3)] + [[0, 0, 0, 1]]


class BB:
    def __init__(self, ex, ey, ez=0.75):
        self.extent = V3(ex, ey, ez)


class Ctrl:
    reverse = False
    hand_brake = False
    throttle = 0.4
    brake = 0.0
    steer = 0.0


class Actor:
    def __init__(self, aid, type_id, x, y, yaw, speed=(0, 0), role=""):
        self.id = aid
        self.type_id = type_id
        self._tf = TF(x, y, yaw)
        self.bounding_box = BB(2.3, 0.92)
        self._v = V3(*speed)
        self.attributes = {"role_name": role}
        self.is_alive = True

    def get_transform(self):
        return self._tf

    def get_location(self):
        return self._tf.location

    def get_velocity(self):
        return self._v

    def get_control(self):
        return Ctrl()


class ActorList(list):
    def filter(self, pattern):
        prefix = pattern.rstrip("*")
        return ActorList(a for a in self if a.type_id.startswith(prefix))


class WP:
    lane_width = 3.5
    lane_type = "Driving"
    lane_id = -1
    is_junction = False

    def __init__(self, s, lane_x=0.0):
        self.s = s
        self.lane_x = lane_x
        self.transform = TF(lane_x, s, 90.0)

    def next(self, d):
        return [WP(self.s + d, self.lane_x)]

    def previous(self, d):
        return [WP(self.s - d, self.lane_x)]

    def get_left_lane(self):      # 진행방향(+y) 기준 왼쪽 = 월드 +x
        return WP(self.s, self.lane_x + 3.5) if self.lane_x < 3 else None

    def get_right_lane(self):
        return None


class FakeMap:
    name = "Town_Fake"

    def get_waypoint(self, loc, project_to_road=True, lane_type=None):
        return WP(loc.y, 0.0)


class FakeWorld:
    id = 1

    def __init__(self, actors):
        self._actors = ActorList(actors)

    def get_actors(self):
        return self._actors

    def get_spectator(self):
        return types.SimpleNamespace(get_location=lambda: V3(0, 95, 5))


fake_carla = types.SimpleNamespace(LaneType=types.SimpleNamespace(Driving="Driving"))


class BridgeMathTest(unittest.TestCase):
    def test_to_ego_heading_plus_y(self):
        # 자차가 월드 +y(yaw 90) 를 볼 때, 월드 (0, 110) 은 10 m 앞
        x, y = bridge.to_ego(0, 100, 90, 0, 110)
        self.assertAlmostEqual(x, 10, places=6)
        self.assertAlmostEqual(y, 0, places=6)
        # yaw 90 기준 오른쪽은 월드 -x (CARLA: x 동, y 남 → 북쪽(+y) 을 보면 오른쪽은 -x)
        x, y = bridge.to_ego(0, 100, 90, -3.5, 100)
        self.assertAlmostEqual(x, 0, places=6)
        self.assertAlmostEqual(y, 3.5, places=6)

    def test_choose_lead_same_lane_only(self):
        center = [[-10, 0], [100, 0]]
        objs = [
            {"id": 1, "x": 30, "y": 3.5, "length": 4.6, "width": 1.8},   # 옆 차선
            {"id": 2, "x": 50, "y": 0.2, "length": 4.6, "width": 1.8},   # 같은 차선
            {"id": 3, "x": -8, "y": 0, "length": 4.6, "width": 1.8},     # 뒤
        ]
        lead_id, gap = bridge.choose_lead(objs, center, 3.5, 4.6)
        self.assertEqual(lead_id, 2)
        self.assertAlmostEqual(gap, 50 - 4.6)

    def test_build_packet_with_fake_world(self):
        ego = Actor(10, "vehicle.tesla.model3", 0, 100, 90, speed=(0, 20), role="hero")
        lead = Actor(11, "vehicle.lincoln.mkz_2020", 0, 140, 90, speed=(0, 10))
        side = Actor(12, "vehicle.audi.tt", 3.5, 120, 90)          # 월드 +x = 자차 왼쪽
        far = Actor(13, "vehicle.audi.tt", 0, 400, 90)             # 범위 밖
        ped = Actor(14, "walker.pedestrian.0001", -6, 130, 0)
        world = FakeWorld([ego, lead, side, far, ped])
        args = types.SimpleNamespace(ahead=100.0, behind=20.0, side=20.0, role_name="hero", actor_id=None)

        self.assertIs(bridge.find_ego(world, args), ego)
        pkt = bridge.build_packet(fake_carla, world, FakeMap(), ego, args, seq=7)

        self.assertAlmostEqual(pkt["ego"]["speed_kmh"], 72.0, places=1)
        ids = {v["id"]: v for v in pkt["vehicles"]}
        self.assertEqual(set(ids), {11, 12, 14})
        self.assertAlmostEqual(ids[11]["x"], 40.0, places=1)
        self.assertAlmostEqual(ids[12]["y"], -3.5, places=1)       # 왼쪽 = 음수
        self.assertEqual(ids[14]["kind"], "walker")
        self.assertEqual(pkt["lead"]["id"], 11)
        self.assertAlmostEqual(pkt["lead"]["distance"], 40 - 4.6, places=1)
        self.assertEqual(pkt["lane"]["lanes_left"], 1)
        self.assertEqual(pkt["lane"]["lanes_right"], 0)
        # 차선 중심선은 자차 기준 직선 (y≈0) 이고 앞으로 100 m 까지
        self.assertTrue(all(abs(p[1]) < 1e-6 for p in pkt["lane"]["center"]))
        self.assertGreaterEqual(pkt["lane"]["center"][-1][0], 99)

        # HMI 장면으로 변환되는지
        sc = scene.build_scene(scene.SpriteLibrary("/nonexistent"), carla=pkt)
        self.assertEqual(sc["source"], "CARLA")
        self.assertEqual([v["lead"] for v in sc["vehicles"] if v["id"] == 11], [True])
        # 왼쪽 1차선, 오른쪽 0차선 → ego 2 + 왼쪽 1 = 3개 차선 라인
        self.assertEqual(len(sc["lines"]), 3)


class SpriteTest(unittest.TestCase):
    def test_make_sprite_mask_and_crop(self):
        import numpy as np
        from PIL import Image
        import capture_vehicle_sprites as cap

        h, w = 300, 400
        rgb = np.zeros((h, w, 4), np.uint8)
        rgb[:, :, 0] = 255          # B (하늘)
        rgb[100:200, 50:350, 0] = 0     # 차량 영역: 파랑 제거
        rgb[100:200, 50:350, 2] = 200   # 차량 영역 R
        sem = np.zeros((h, w, 4), np.uint8)
        sem[:, :, 2] = 11           # Sky
        sem[100:200, 50:350, 2] = 14    # Car
        img = cap.make_sprite(np, Image, rgb, sem, 150)
        self.assertEqual(img.mode, "RGBA")
        self.assertEqual(img.size, (150, 50))
        a = np.array(img)
        self.assertGreater(a[25, 75, 3], 250)       # 차량 중심 불투명
        self.assertLess(a[25, 75, 2], 60)           # 하늘색(파랑) 섞이지 않음

    def test_no_vehicle_returns_none(self):
        import numpy as np
        from PIL import Image
        import capture_vehicle_sprites as cap
        z = np.zeros((50, 50, 4), np.uint8)
        self.assertIsNone(cap.make_sprite(np, Image, z, z, 100))


class SceneTest(unittest.TestCase):
    def test_offset_polyline_right_is_plus_y(self):
        pts = scene.offset_polyline([[0, 0], [10, 0]], 1.75)
        self.assertAlmostEqual(pts[0][1], 1.75)

    def test_sprite_fallback(self):
        import tempfile
        d = tempfile.mkdtemp()
        open(os.path.join(d, "default_rear.png"), "wb").close()
        open(os.path.join(d, "vehicle.audi.tt_rear.png"), "wb").close()
        lib = scene.SpriteLibrary(d)
        self.assertTrue(lib.get("vehicle.audi.tt", "rear").endswith("vehicle.audi.tt_rear.png"))
        self.assertTrue(lib.get("vehicle.bmw.x", "rear").endswith("default_rear.png"))
        self.assertEqual(lib.get("vehicle.bmw.x", "hero"), "")

    def test_demo_scene_far_lead_hidden(self):
        lib = scene.SpriteLibrary("/nonexistent")
        far = scene.build_scene(lib, demo={"distance_m": 1000})
        near = scene.build_scene(lib, demo={"distance_m": 30})
        self.assertEqual(far["vehicles"], [])
        self.assertEqual(len(near["vehicles"]), 1)
        self.assertEqual(near["lead_gap"], 30)


if __name__ == "__main__":
    unittest.main()
