"""3D 주행 화면 지원 코드 검증 (차선 형상, 차종 분류, 모델 파일)"""

import os
import struct
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from hmi import scene  # noqa: E402


class CategoryTest(unittest.TestCase):
    def test_categories(self):
        self.assertEqual(scene.model_category("vehicle.tesla.model3"), "car")
        self.assertEqual(scene.model_category("vehicle.carlamotors.carlacola"), "truck")
        self.assertEqual(scene.model_category("vehicle.mitsubishi.fusorosa"), "bus")
        self.assertEqual(scene.model_category("vehicle.harley-davidson.low_rider"), "bike")
        self.assertEqual(scene.model_category("walker.pedestrian.0001", kind="walker"), "person")
        self.assertEqual(scene.model_category("vehicle.unknown", length=12), "bus")

    def test_scene_vehicle_has_model(self):
        sc = scene.build_scene(scene.SpriteLibrary(), distance_m=20)
        self.assertEqual(sc["vehicles"][0]["model"], "car")


class ModelFilesTest(unittest.TestCase):
    def test_mesh_files_exist(self):
        d = os.path.join(ROOT, "assets", "models")
        needed = ["ego_paint_mesh.mesh", "ego_glass_mesh.mesh", "ego_tire_mesh.mesh", "ego_rim_mesh.mesh",
                  "ego_brakelight_mesh.mesh", "car_body_mesh.mesh", "car_dark_mesh.mesh", "car_wheel_mesh.mesh",
                  "truck_body_mesh.mesh", "bus_body_mesh.mesh", "bike_body_mesh.mesh", "person_body_mesh.mesh"]
        for n in needed:
            self.assertTrue(os.path.getsize(os.path.join(d, n)) > 100, n)
        for n in ("studio.hdr", "studio_dark.hdr", "contact_shadow.png", "lane_dash.png"):
            self.assertTrue(os.path.exists(os.path.join(ROOT, "assets", "env", n)), n)


class RibbonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtGui import QGuiApplication
        cls.app = QGuiApplication.instance() or QGuiApplication(["test", "-platform", "offscreen"])

    def _verts(self, g):
        data = bytes(g.vertexData())
        return [struct.unpack_from("<8f", data, i) for i in range(0, len(data), 32)]

    def test_strip_axes_and_uv(self):
        from hmi.geometry3d import RibbonGeometry
        g = RibbonGeometry()
        g.width = 0.2
        g.dash = 3.0
        g.gap = 6.0
        g.points = [[0, 1.75], [18, 1.75]]
        v = self._verts(g)
        self.assertEqual(len(v) % 3, 0)
        xs = {round(p[0], 3) for p in v}
        self.assertEqual(xs, {1.65, 1.85})               # X = y_ego ± 폭/2
        zs = [p[2] for p in v]
        self.assertAlmostEqual(min(zs), -18.0)            # Z = -x_ego
        self.assertAlmostEqual(max(p[7] for p in v), 2.0)  # v = 거리 / (dash + gap)
        for p in v:
            self.assertEqual(p[4], 1.0)                   # 법선 위

    def test_band_and_empty(self):
        from hmi.geometry3d import RibbonGeometry
        g = RibbonGeometry()
        g.left = [[0, -1], [10, -1]]
        g.right = [[0, 1], [10, 1]]
        self.assertEqual(len(self._verts(g)), 6)
        e = RibbonGeometry()
        self.assertEqual(len(self._verts(e)), 3)          # 빈 형상도 유효


if __name__ == "__main__":
    unittest.main()


class LaneFallbackTest(unittest.TestCase):
    def test_far_lane_falls_back_to_straight(self):
        carla = {"lane": {"center": [[-10, 30], [40, 30]], "width": 3.5}, "vehicles": [], "lead": {}}
        sc = scene.build_scene(scene.SpriteLibrary(), carla=carla)
        self.assertAlmostEqual(scene.interp_y(sc["center"], 0.0), 0.0)

    def test_short_lane_falls_back(self):
        carla = {"lane": {"center": [[-10, 0], [2, 0]], "width": 3.5}, "vehicles": [], "lead": {}}
        sc = scene.build_scene(scene.SpriteLibrary(), carla=carla)
        self.assertGreater(sc["center"][-1][0], 50)
        self.assertTrue(sc["lines"])
