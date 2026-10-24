"""
카메라 영상 경로 검증: 투영 계산, clean 스타일, TCP 프레임 송수신 (CARLA 없이)
python3 -m unittest discover -s tests
"""

import os
import io
import queue
import socket
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

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


class CameraDirectionTest(unittest.TestCase):
    def test_hmi_keeps_only_latest_queued_camera_frame(self):
        from hmi.backend import HmiBackend

        receiver = SimpleNamespace(_camera_lock=threading.Lock(), _camera_latest=None,
                                   _inbox=queue.Queue(), _dropped=0)
        HmiBackend.sink(receiver, "camera", ({"frame": 1}, b"old"), 1.0)
        HmiBackend.sink(receiver, "camera", ({"frame": 2}, b"latest"), 2.0)
        self.assertEqual(receiver._camera_latest[0]["frame"], 2)
        self.assertTrue(receiver._inbox.empty())

    def test_single_sensor_switches_relative_transform_and_clears_old_frame(self):
        camera = cs.ChaseCamera.__new__(cs.ChaseCamera)
        camera.world = Mock()
        camera.world.get_snapshot.return_value.frame = 42
        camera.cam = Mock()
        camera.seg = Mock()
        camera.sender = Mock()
        camera.front_transform = object()
        camera.rear_transform = object()
        camera.mode = "clean"
        camera.view = "D"
        camera.min_frame = 0
        camera.last_tags = (41, object())
        camera.lock = threading.Lock()

        camera.set_view("R")
        self.assertEqual(camera.view, "R")
        self.assertEqual(camera.min_frame, 43)
        self.assertIsNone(camera.last_tags)
        camera.cam.set_transform.assert_called_once_with(camera.rear_transform)
        camera.seg.set_transform.assert_called_once_with(camera.rear_transform)
        camera.sender.clear.assert_called_once()

        camera.set_view("P")
        camera.set_view("R")
        self.assertEqual(camera.cam.set_transform.call_count, 1)
        camera.set_view("D")
        self.assertEqual(camera.cam.set_transform.call_count, 2)
        camera.cam.set_transform.assert_called_with(camera.front_transform)

    def test_hmi_hides_old_view_until_matching_frame_arrives(self):
        from PIL import Image
        from hmi.backend import HmiBackend

        receiver = SimpleNamespace(
            _cam={"fresh": True, "view": "D", "frameId": 1},
            _cam_pending=({"view": "D"}, b"old", 1.0),
            _cam_t=1.0,
            _cam_count=0,
            camera_provider=SimpleNamespace(image=None),
            cameraChanged=SimpleNamespace(emit=Mock()),
            _add_event=Mock(),
        )
        HmiBackend._update_camera(receiver, 1.1, "R")
        self.assertFalse(receiver._cam["fresh"])
        self.assertEqual(receiver._cam["frameId"], 1)

        receiver._cam_pending = ({"frame": 3}, b"legacy", 1.15)
        HmiBackend._update_camera(receiver, 1.15, "R")
        self.assertFalse(receiver._cam["fresh"])
        self.assertEqual(receiver._cam["frameId"], 1)

        image = Image.new("RGB", (8, 8))
        encoded = io.BytesIO()
        image.save(encoded, format="JPEG")
        receiver._cam_pending = ({"view": "R", "mode": "real"}, encoded.getvalue(), 1.2)
        HmiBackend._update_camera(receiver, 1.2, "R")
        self.assertTrue(receiver._cam["fresh"])
        self.assertEqual(receiver._cam["frameId"], 2)
        self.assertEqual(receiver._cam["view"], "R")


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
