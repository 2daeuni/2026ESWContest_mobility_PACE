"""
카메라 영상 경로 검증: 투영 계산, clean 스타일, TCP 프레임 송수신 (CARLA 없이)
python3 -m unittest discover -s tests
"""

import os
import queue
import socket
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "carla_tools"))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import camera_stream as cs  # noqa: E402
from fake_camera_sender import box_vertices, camera_inverse  # noqa: E402

W, H = 960, 540


class ProjectionTest(unittest.TestCase):
    def setUp(self):
        self.K = cs.intrinsics(W, H, 90.0)
        self.inv = camera_inverse(back=0.0, up=0.0, pitch_deg=0.0)   # 원점, 정면

    def test_center_point_maps_to_image_center(self):
        u, v, d = cs.world_to_pixel(self.inv, self.K, (10, 0, 0))
        self.assertAlmostEqual(u, W / 2)
        self.assertAlmostEqual(v, H / 2)
        self.assertAlmostEqual(d, 10)

    def test_axes(self):
        u, _v, _d = cs.world_to_pixel(self.inv, self.K, (10, 2, 0))       # 오른쪽 → u 증가
        self.assertGreater(u, W / 2)
        _u, v, _d = cs.world_to_pixel(self.inv, self.K, (10, 0, 2))       # 위 → v 감소
        self.assertLess(v, H / 2)
        # fov 90: x=10 일 때 y=10 이면 화면 오른쪽 끝
        u, _v, _d = cs.world_to_pixel(self.inv, self.K, (10, 10, 0))
        self.assertAlmostEqual(u, W, places=3)

    def test_behind_camera_is_none(self):
        self.assertIsNone(cs.world_to_pixel(self.inv, self.K, (-5, 0, 0)))

    def test_box_clamped_and_offscreen(self):
        box = cs.project_box(self.inv, self.K, box_vertices(20, 0, 4.6, 1.9, 1.5), W, H)
        self.assertIsNotNone(box)
        x0, y0, x1, y1 = box
        self.assertTrue(0 <= x0 < W / 2 < x1 <= W)
        self.assertLess(y0, y1)
        self.assertIsNone(cs.project_box(self.inv, self.K, box_vertices(-20, 0, 4.6, 1.9, 1.5), W, H))

    def test_lane_overlay_cut_behind_camera(self):
        inv = camera_inverse(back=6.4, up=4.1, pitch_deg=-10.0)
        K = cs.intrinsics(W, H, 75.0)
        left = [(x, -1.75, 0) for x in range(-20, 60, 2)]
        out = cs.build_overlay(inv, K, W, H, {"lane_left": left, "lane_right": [], "lead_vertices": None})
        self.assertTrue(out["lane_left"])
        self.assertLess(len(out["lane_left"]), len(left))
        self.assertIsNone(out["lead_box"])


class CleanStyleTest(unittest.TestCase):
    def test_tags(self):
        import numpy as np
        rgb = np.full((4, 4, 3), 90, dtype=np.uint8)
        rgb[0, 0] = (200, 30, 30)
        tags = np.zeros((4, 4), dtype=np.uint8)
        tags[0, 0] = 14            # Car → 원본 렌더링 유지(탈색)
        tags[1, :] = cs.TAG_ROAD
        tags[2, :] = cs.TAG_ROADLINE
        out = cs.stylize_clean(np, rgb, tags)
        self.assertEqual(tuple(out[2, 0]), (252, 252, 252))
        self.assertGreater(int(out[0, 0, 0]), int(out[0, 0, 2]))     # 빨간 차는 여전히 붉은 기
        self.assertGreater(int(out[3, 3, 0]), 220)                    # 배경은 밝게


class TcpRoundTripTest(unittest.TestCase):
    def test_sender_to_receiver(self):
        from hmi.sources import CameraReceiver
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()

        q = queue.Queue()
        rx = CameraReceiver(lambda k, p, t: q.put((k, p)), host="127.0.0.1", port=port)
        rx.start()
        time.sleep(0.2)
        tx = cs.FrameSender("127.0.0.1", port)
        tx.start()
        try:
            jpeg = b"\xff\xd8" + b"x" * 5000 + b"\xff\xd9"
            got = None
            deadline = time.time() + 5
            while time.time() < deadline and got is None:
                tx.offer({"type": "camera_frame", "frame": 7, "overlay": {"lead_gap": 12.5}}, jpeg)
                try:
                    while True:
                        k, p = q.get(timeout=0.2)
                        if k == "camera":
                            got = p
                            break
                except queue.Empty:
                    pass
            self.assertIsNotNone(got)
            header, data = got
            self.assertEqual(header["frame"], 7)
            self.assertEqual(header["overlay"]["lead_gap"], 12.5)
            self.assertEqual(data, jpeg)
        finally:
            tx.stop()
            rx.stop()
            tx.join(2)
            rx.join(2)


if __name__ == "__main__":
    unittest.main()


class RearGuideTest(unittest.TestCase):
    """후방 카메라 가이드선 투영 = 실제 카메라 행렬 투영(좌우 반전)과 같은지"""

    def test_matches_camera_matrix(self):
        import math
        w, h, cam_h, a_deg, L = 640, 360, 0.85, cs.REAR_PITCH, 2.3
        K = cs.intrinsics(w, h, cs.REAR_FOV)
        a = math.radians(a_deg)
        fwd = (-math.cos(a), 0.0, -math.sin(a))
        right = (0.0, -1.0, 0.0)
        up = (-math.sin(a), 0.0, math.cos(a))
        pos = (-L, 0.0, cam_h)

        def row(v):
            return [v[0], v[1], v[2], -(v[0] * pos[0] + v[1] * pos[1] + v[2] * pos[2])]
        inv = [row(fwd), row(right), row(up), [0, 0, 0, 1]]
        for d, lat in ((0.5, 1.0), (1.0, -1.0), (3.0, 0.4)):
            u, v, _ = cs.world_to_pixel(inv, K, (-L - d, lat, 0.0))
            p = cs.rear_cam_pixel(K, w, cam_h, a_deg, d, lat)
            self.assertAlmostEqual(p[0], round(w - u, 1), places=0)
            self.assertAlmostEqual(p[1], round(v, 1), places=0)

    def test_guides_colors_and_mirror(self):
        K = cs.intrinsics(640, 360, cs.REAR_FOV)
        g = cs.rear_guides(K, 640, 360, 0.85, cs.REAR_PITCH, 1.1)
        self.assertEqual({x["c"] for x in g}, {"red", "yellow", "green"})
        # 운전자 오른쪽 선이 화면 오른쪽 (룸미러처럼)
        right = cs.rear_cam_pixel(K, 640, 0.85, cs.REAR_PITCH, 2.0, 1.1)
        left = cs.rear_cam_pixel(K, 640, 0.85, cs.REAR_PITCH, 2.0, -1.1)
        self.assertGreater(right[0], left[0])


class HitDirectionTest(unittest.TestCase):
    def test_directions(self):
        import carla_hmi_bridge as b
        self.assertEqual(b.hit_direction(0, 1, 0), "전방")
        self.assertEqual(b.hit_direction(0, 0, 1), "우측")
        self.assertEqual(b.hit_direction(0, -1, 0), "후방")
        self.assertEqual(b.hit_direction(90, 1, 0), "좌측")
        self.assertEqual(b.hit_direction(0, 1, -1), "전방 좌측")
