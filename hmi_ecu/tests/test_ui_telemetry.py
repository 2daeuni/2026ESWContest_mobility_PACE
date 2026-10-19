"""UI-only telemetry tests; no sockets, CAN, or CARLA server."""

import os
import copy
import json
import sys
import unittest
from unittest.mock import patch, Mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hmi import config, events, state_merge
from hmi.demo_scenario import DemoScenario

try:
    from PySide6.QtGui import QGuiApplication
    from hmi.backend import HmiBackend
except ImportError:
    HmiBackend = None


class UiTelemetryTest(unittest.TestCase):
    def test_day_night_and_lane_transitions(self):
        for night, enabled, active in (
            (False, False, False),
            (True, True, False),
            (True, True, True),
            (False, True, False),
            (False, False, False),
        ):
            state = state_merge.merge({}, None, now=100)
            packet = {
                "night_mode": night,
                "lane_assist_enabled": enabled,
                "lane_assist_active": active,
                "ego": {"speed_kmh": 32, "gear": "D"},
                "lead": {"distance": 18},
            }
            state_merge.apply_carla(state, packet)
            state_merge.apply_carla_ui(state, {"data": packet, "t": 100}, now=100)
            self.assertIs(state["night_mode"], night)
            self.assertIs(state["lane_assist_enabled"], enabled)
            self.assertIs(state["lane_assist_active"], active)
            self.assertEqual(state["speed_kmh"], 32)
            self.assertEqual(state["distance_m"], 18)

    def test_missing_or_expired_ui_hides_assist_and_marks_night_unknown(self):
        state = {}
        state_merge.apply_carla_ui(state, None, now=100)
        self.assertEqual(state, {"night_mode": None, "lane_assist_enabled": False,
                                 "lane_assist_active": False})
        store = {"data": {"night_mode": True, "lane_assist_enabled": True,
                          "lane_assist_active": True}, "t": 100}
        state_merge.apply_carla_ui(state, store, now=100 + config.CARLA_UI_STATE_TIMEOUT)
        self.assertIsNone(state["night_mode"])
        self.assertFalse(state["lane_assist_enabled"])
        self.assertFalse(state["lane_assist_active"])

    def test_tof_link_changes_emit_no_event(self):
        self.assertEqual(events.diff({"link_tof": True}, {"link_tof": False}), [])

    def test_demo_has_no_tof_failsafe(self):
        demo = DemoScenario()
        for _ in range(400):
            _, packet = demo.step(0.1)
            self.assertFalse(packet["failsafe_active"])
            self.assertIsNone(packet["failsafe_reason"])


@unittest.skipIf(HmiBackend is None, "PySide6 is required for backend mock tests")
class BackendUiIsolationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QGuiApplication.instance() or QGuiApplication(["test", "-platform", "offscreen"])

    def setUp(self):
        with patch("hmi.backend.AlertSound") as audio:
            audio.return_value.muted = True
            audio.return_value.backend = "mock"
            self.backend = HmiBackend(sound=False)
        self.backend._timer.stop()
        self.backend._diag_timer.stop()

    def tearDown(self):
        self.backend.deleteLater()

    def tick(self, now):
        with patch("hmi.backend.time.monotonic", return_value=now):
            self.backend._tick()

    def test_scene_interleaved_with_ui_transitions_and_independent_expiry(self):
        backend = self.backend
        scene = {
            "type": "carla_scene", "seq": 1,
            "ego": {"speed_kmh": 32, "gear": "D"},
            "lane": {"center": [[-10, 0], [60, 0]], "width": 3.5},
            "vehicles": [{"id": 7, "kind": "car", "x": 20, "y": 0}],
            "lead": {"id": 7, "distance": 18},
        }
        safety = {"data": {"carla_connected": True}, "t": 100, "count": 1}
        backend._udp_store = safety
        backend._carla_map = {"ver": 4, "lanes": ["sentinel"]}
        backend._cam_pending = ({"w": 1}, b"camera sentinel", 100)
        backend.sink("carla", copy.deepcopy(scene), 100)
        self.tick(100)
        def geometry_snapshot():
            # Packet sequence is diagnostic metadata, not rendered geometry or N/J state.
            return {key: copy.deepcopy(value) for key, value in backend._state["scene"].items()
                    if key != "scene_seq"}

        geometry = geometry_snapshot()
        self.assertEqual(backend._carla_store["data"], scene)

        for i, (night, enabled, active) in enumerate((
            (False, False, False), (True, True, False), (True, True, True),
            (False, True, False), (False, False, False),
        )):
            now = 100.1 + i * 0.1
            scene_store = backend._carla_store
            map_before = copy.deepcopy(backend._carla_map)
            camera_before = copy.deepcopy(backend._cam)
            pending = backend._cam_pending
            collision = backend._collision_n
            backend._process("carla", {
                "type": "carla_ui_state", "night_mode": night,
                "lane_assist_enabled": enabled, "lane_assist_active": active,
                # Extra fields in a UI packet must never reach scene/camera/Safety.
                "vehicles": [], "lane": {}, "collision": {"n": 999},
            }, now)
            self.assertIs(backend._carla_store, scene_store)
            self.assertEqual(backend._carla_map, map_before)
            self.assertEqual(backend._cam, camera_before)
            self.assertIs(backend._cam_pending, pending)
            self.assertEqual(backend._collision_n, collision)
            self.assertIs(backend._udp_store, safety)
            self.tick(now)
            self.assertIs(backend._state["night_mode"], night)
            self.assertIs(backend._state["lane_assist_enabled"], enabled)
            self.assertIs(backend._state["lane_assist_active"], active)
            self.assertEqual(geometry_snapshot(), geometry)

            # Scene traffic between UI updates must preserve the UI store/flags.
            ui_store = backend._carla_ui_store
            scene["seq"] += 1
            backend.sink("carla", copy.deepcopy(scene), now + 0.01)
            self.tick(now + 0.01)
            self.assertIs(backend._carla_ui_store, ui_store)
            self.assertIs(backend._state["lane_assist_enabled"], enabled)
            self.assertIs(backend._state["lane_assist_active"], active)
            self.assertEqual(geometry_snapshot(), geometry)

        # Malformed/unknown packets neither replace scene nor refresh UI freshness.
        ui_store = backend._carla_ui_store
        scene_store = backend._carla_store
        backend._process("carla", {"type": "carla_ui_state", "night_mode": "true"}, 101)
        backend._process("carla", {"type": "unknown"}, 101)
        self.assertIs(backend._carla_ui_store, ui_store)
        self.assertIs(backend._carla_store, scene_store)

        backend._process("carla", {
            "type": "carla_ui_state", "night_mode": True,
            "lane_assist_enabled": True, "lane_assist_active": True,
        }, 100.7)
        self.tick(100.7)
        self.assertTrue(backend._state["lane_assist_active"])
        expiry = 100.7 + config.CARLA_UI_STATE_TIMEOUT
        backend.sink("carla", scene, expiry)
        self.tick(expiry)
        self.assertFalse(backend._state["lane_assist_enabled"])
        self.assertFalse(backend._state["lane_assist_active"])
        self.assertIsNone(backend._state["night_mode"])
        self.assertEqual(geometry_snapshot(), geometry)

    def test_ui_does_not_keep_expired_scene_alive(self):
        backend = self.backend
        backend._process("carla", {"type": "carla_scene", "vehicles": [], "lane": {}}, 100)
        backend._process("carla", {
            "type": "carla_ui_state", "night_mode": True,
            "lane_assist_enabled": False, "lane_assist_active": True,
        }, 102)
        self.tick(102)
        self.assertEqual(backend._carla_store["t"], 100)
        self.assertFalse(backend._state["link_carla"])
        self.assertTrue(backend._state["night_mode"])
        self.assertFalse(backend._state["lane_assist_active"])


class ReceiverRoutingTest(unittest.TestCase):
    def test_same_socket_forwards_scene_ui_and_map_without_merging(self):
        from hmi.sources import UdpReceiver
        received = []
        receiver = UdpReceiver(lambda kind, data, t: received.append((kind, data)), kind="carla", port=5601)
        packets = [{"type": "carla_scene", "vehicles": [7]},
                   {"type": "carla_ui_state", "night_mode": True,
                    "lane_assist_enabled": True, "lane_assist_active": False},
                   {"type": "carla_map", "ver": 4}]
        encoded = iter(json.dumps(packet).encode() for packet in packets)
        sock = Mock()

        def recv(_size):
            packet = next(encoded)
            if json.loads(packet)["type"] == "carla_map":
                receiver.stop()
            return packet, ("127.0.0.1", 12345)

        sock.recvfrom.side_effect = recv
        with patch("hmi.sources.socket.socket", return_value=sock):
            receiver.run()
        self.assertEqual([data for kind, data in received if kind == "carla"], packets)
        sock.bind.assert_called_once_with((config.UDP_BIND_HOST, 5601))


if __name__ == "__main__":
    unittest.main()
