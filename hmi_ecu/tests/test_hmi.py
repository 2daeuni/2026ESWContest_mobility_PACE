"""
python3 -m unittest discover -s tests   (hmi_ecu 폴더에서 실행)
Qt 없이 디코더 / 병합 / 이벤트 / Safety ECU 퍼블리셔 호환성 검증.
"""

import json
import os
import socket
import struct
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), "safety_ecu"))

from hmi import can_decoder, config, events, state_merge  # noqa: E402


def pedal_frame(acc, brk, acc_rate=0.0, status=0, alive=1):
    return bytes([acc, brk, *struct.pack("<h", int(acc_rate * 10)), *struct.pack("<h", 0), status, alive])


class DecoderTest(unittest.TestCase):
    def test_pedal_matches_safety_decoder_example(self):
        # safety_ecu/canbus/decoder.py __main__ 의 테스트 벡터
        name, d = can_decoder.decode(0x100, bytes.fromhex("48 00 AC 0D 00 00 00 37"))
        self.assertEqual(name, "PEDAL_STATUS")
        self.assertEqual(d["accelerator"], 0x48)
        self.assertAlmostEqual(d["accelerator_rate"], 0x0DAC * 0.1, places=3)
        self.assertEqual(d["alive"], 0x37)

    def test_haptic_command_layout_matches_sender(self):
        name, d = can_decoder.decode(0x110, bytes([3, 1, 65, 30, 0, 0, 0, 9]))
        self.assertEqual((d["risk_level"], d["vibration_command"], d["intensity"], d["frequency"]), (3, 1, 65, 30))

    def test_bad_dlc_ignored(self):
        self.assertIsNone(can_decoder.decode(0x100, bytes(4)))
        self.assertIsNone(can_decoder.decode(0x7FF, bytes(8)))


def can_store(now, **frames):
    store = {}
    for name, (can_id, data) in frames.items():
        _n, decoded = can_decoder.decode(can_id, data)
        store[name] = {"data": decoded, "t": now}
    return store


class MergeTest(unittest.TestCase):
    def test_no_data_is_not_normal(self):
        s = state_merge.merge({}, None, now=100.0)
        self.assertFalse(s["link_safety"])
        self.assertIsNone(s["risk_level"])
        self.assertEqual(s["banner"]["severity"], "offline")

    def test_can_only_level0_not_reported_as_normal(self):
        now = 100.0
        store = can_store(now, HAPTIC_COMMAND=(0x110, bytes([0, 0, 0, 0, 0, 0, 0, 1])))
        s = state_merge.merge(store, None, now=now)
        self.assertTrue(s["link_safety"])
        self.assertIsNone(s["risk_level"])
        self.assertNotEqual(s["banner"]["severity"], "ok")

    def test_can_only_level3_shown(self):
        now = 100.0
        store = can_store(
            now,
            HAPTIC_COMMAND=(0x110, bytes([3, 1, 65, 30, 0, 0, 0, 1])),
            PEDAL_STATUS=(0x100, pedal_frame(80, 0)),
        )
        s = state_merge.merge(store, None, now=now)
        self.assertEqual(s["risk_level"], 3)
        self.assertTrue(s["brake_now"])

    def test_stale_can_dropped(self):
        store = can_store(100.0, PEDAL_STATUS=(0x100, pedal_frame(50, 0)))
        s = state_merge.merge(store, None, now=100.0 + config.CAN_FRESH_TIMEOUT + 0.01)
        self.assertIsNone(s["accelerator"])
        self.assertFalse(s["link_pedal"])

    def test_udp_unavailable_risk(self):
        udp = {"data": {"risk_level": 0, "risk_available": False, "pedal_connected": True,
                        "distance_valid": True, "distance_cm": 120}, "t": 100.0}
        s = state_merge.merge({}, udp, now=100.1)
        self.assertIsNone(s["risk_level"])
        self.assertEqual(s["banner"]["severity"], "info")
        self.assertIn("차속", s["banner"]["title"])
        self.assertAlmostEqual(s["distance_m"], 1.2)

    def test_failsafe_from_udp(self):
        udp = {"data": {"failsafe_active": True, "failsafe_reason": "PEDAL_SENSOR_ERROR"}, "t": 1.0}
        s = state_merge.merge({}, udp, now=1.05)
        self.assertEqual(s["banner"]["severity"], "fault")
        self.assertIn("페달 센서", s["banner"]["title"])

    def test_score_normalized(self):
        udp = {"data": {"risk_level": 2, "risk_available": True, "total_risk": 64,
                        "bns": 80, "bns_available": True, "score_max": 100}, "t": 1.0}
        s = state_merge.merge({}, udp, now=1.0)
        self.assertAlmostEqual(s["total_risk"], 0.64)
        self.assertAlmostEqual(s["bns"], 0.8)

    def test_garbage_values_do_not_crash(self):
        udp = {"data": {"risk_level": "x", "vehicle_speed": "fast", "ttc": float("nan"),
                        "gear": 7, "distance_valid": True, "distance_cm": None}, "t": 1.0}
        s = state_merge.merge({}, udp, now=1.0)
        self.assertIsNone(s["speed_kmh"])
        self.assertIsNone(s["ttc_s"])
        self.assertIsNone(s["gear"])


class EventTest(unittest.TestCase):
    def test_level_and_link_events(self):
        prev = state_merge.merge({}, None, now=0)
        udp = {"data": {"risk_level": 3, "risk_available": True}, "t": 1.0}
        cur = state_merge.merge({}, udp, now=1.0)
        texts = [e["text"] for e in events.diff(prev, cur)]
        self.assertTrue(any("Safety ECU 연결됨" in t for t in texts))
        self.assertTrue(any("L3" in t for t in texts))


class PublisherCompatTest(unittest.TestCase):
    """Verify that HMI understands packets from the current Safety sender."""

    @staticmethod
    def sample():
        state = types.SimpleNamespace(
            accelerator=70, brake=0, accelerator_rate=120.5, brake_rate=0.0,
            pedal_connected=True, pedal_sensor_status=0,
            tof_connected=True, carla_connected=False,
            distance_valid=True, distance_cm=850.0,
            vehicle_speed=None, ttc=None,
            bns=0.0, pms=0.0, total_risk=0.0, risk_level=0,
            haptic_connected=True, haptic_available=True, haptic_fault_code=0,
        )
        commands = {"haptic": {"vibration_command": 0, "intensity": 0, "frequency": 0},
                    "motor": {"control_mode": 0, "output_limit": 100, "command_flags": 1}}
        values = dict(input_mode="HARDWARE", bns_available=False, pms_available=True,
                      risk_available=False, failsafe_active=False, failsafe_reason=None,
                      commands=commands, motor_connected=True)
        return state, values

    def test_roundtrip(self):
        from hmi_sender import HmiSender

        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(1.0)
        port = rx.getsockname()[1]

        state, values = self.sample()
        sender = HmiSender()
        sender.target = ("127.0.0.1", port)
        sender.start()
        try:
            sender.publish(state, **values)
            packet = json.loads(rx.recvfrom(65535)[0])
        finally:
            sender.close()
            rx.close()

        s = state_merge.merge({}, {"data": packet, "t": 5.0}, now=5.0)
        self.assertEqual(s["accelerator"], 70)
        self.assertAlmostEqual(s["distance_m"], 8.5)
        self.assertIsNone(s["risk_level"])          # risk_available=False
        self.assertEqual(s["motor_mode"], "NORMAL")
        self.assertTrue(s["link_safety"])

    def test_publish_never_raises_when_no_listener(self):
        from hmi_sender import HmiSender

        state, values = self.sample()
        sender = HmiSender()
        sender.target = ("127.0.0.1", 9)
        sender.start()
        try:
            for _ in range(3):
                sender.publish(state, **values)
        finally:
            sender.close()


if __name__ == "__main__":
    unittest.main()


class DemoDriveTest(unittest.TestCase):
    def setUp(self):
        from hmi.demo_drive import DemoDrive
        self.Demo = DemoDrive

    def run_until(self, d, seconds, acc, brk=0.0, dt=0.02):
        out = None
        for _ in range(int(seconds / dt)):
            out = d.step(dt, pedal_accel=acc, pedal_brake=brk, pedal_accel_rate=0.0)
        return out

    def test_accelerates_and_brakes_like_a_car(self):
        d = self.Demo(start_distance=5000)
        out = self.run_until(d, 8, acc=100)
        self.assertGreater(out["speed_kmh"], 70)          # 풀가속 8초 → 70 km/h 이상
        self.assertLess(out["speed_kmh"], 130)
        out = self.run_until(d, 6, acc=0, brk=100)
        self.assertEqual(out["speed_kmh"], 0.0)           # 풀브레이크로 정지, 음수 속도 없음

    def test_approach_raises_levels_and_limits_motor(self):
        d = self.Demo(start_distance=300)
        seen = set()
        limited = False
        for _ in range(int(40 / 0.02)):
            out = d.step(0.02, pedal_accel=60, pedal_brake=0, pedal_accel_rate=0.0)
            seen.add(out["risk_level"])
            limited |= out["motor_limit"] == 30
            if out["collided"]:
                break
        self.assertTrue({1, 2, 3} <= seen)
        self.assertTrue(limited)
        self.assertTrue(d.collided)                        # 브레이크 안 밟으면 결국 충돌

    def test_braking_driver_avoids_collision(self):
        d = self.Demo(start_distance=300)
        braking = False
        for _ in range(int(60 / 0.02)):
            acc, brk = (0, 90) if braking else (60, 0)
            out = d.step(0.02, pedal_accel=acc, pedal_brake=brk, pedal_accel_rate=0.0)
            braking |= out["risk_level"] >= 2
            if braking and out["speed_kmh"] == 0:
                break
        self.assertFalse(d.collided)
        self.assertGreater(out["distance_m"], 0)

    def test_keyboard_input(self):
        d = self.Demo(start_distance=1000)
        d.key_accel = True
        for _ in range(100):
            out = d.step(0.02)
        self.assertEqual(out["input_source"], "KEYBOARD")
        self.assertGreater(out["accelerator"], 90)
        self.assertGreater(out["speed_kmh"], 5)

    def test_apply_demo_overrides_display(self):
        d = self.Demo(start_distance=20)
        out = self.run_until(d, 3, acc=80)
        s = state_merge.merge({}, None, now=1.0)
        state_merge.apply_demo(s, out)
        self.assertTrue(s["demo_drive"])
        self.assertEqual(s["risk_level"], out["risk_level"])
        self.assertNotEqual(s["banner"]["severity"], "offline")


class ReactionTest(unittest.TestCase):
    def make(self):
        import tempfile
        from hmi.reaction import ReactionRecorder
        return ReactionRecorder(log_dir=tempfile.mkdtemp(), write_csv=True)

    def st(self, level, acc, brk, dist=30.0, collided=False):
        return {"risk_level": level, "risk_available": True, "accelerator": acc, "brake": brk,
                "distance_m": dist, "ttc_s": 2.0, "speed_kmh": 50, "demo_drive": True,
                "demo_input": "PEDAL", "demo_collided": collided}

    def test_measures_release_and_brake(self):
        r = self.make()
        t = 0.0
        self.assertIsNone(r.update(self.st(2, 60, 0), t))          # 경고 시작
        r.update(self.st(2, 60, 0), t + 0.3)
        r.update(self.st(2, 2, 0), t + 0.6)                        # 가속 해제 0.6 s
        r.update(self.st(2, 0, 40), t + 1.1)                       # 브레이크 1.1 s
        r.update(self.st(1, 0, 40), t + 2.0)
        res = r.update(self.st(1, 0, 40), t + 3.6)                 # 1.5 s 유지 → 종료
        self.assertIsNotNone(res)
        self.assertAlmostEqual(res["release_s"], 0.6, places=2)
        self.assertAlmostEqual(res["brake_s"], 1.1, places=2)
        self.assertEqual(res["outcome"], "회피")
        import os
        self.assertTrue(os.path.exists(os.path.join(r.log_dir, "reaction_log.csv")))

    def test_collision_records_once(self):
        r = self.make()
        r.update(self.st(3, 80, 0), 0.0)
        res = r.update(self.st(3, 80, 0, dist=0, collided=True), 0.5)
        self.assertEqual(res["outcome"], "충돌")
        for i in range(30):                                         # 충돌 상태가 계속돼도 다시 기록 안 함
            self.assertIsNone(r.update(self.st(3, 80, 0, dist=0, collided=True), 0.6 + i * 0.03))
        self.assertEqual(len(r.history), 1)

    def test_summary_by_sound(self):
        r = self.make()
        for sound, rel in ((True, 0.5), (False, 1.0)):
            r.update(self.st(2, 60, 0), 0.0, sound_on=sound)
            r.update(self.st(2, 0, 30), rel, sound_on=sound)
            r.update(self.st(0, 0, 0), rel + 0.1, sound_on=sound)
            r.update(self.st(0, 0, 0), rel + 2.0, sound_on=sound)
        summ = {g["sound"]: g for g in r.summary()}
        self.assertAlmostEqual(summ["ON"]["release_avg"], 0.5)
        self.assertAlmostEqual(summ["OFF"]["release_avg"], 1.0)


class AudioTest(unittest.TestCase):
    def test_patterns(self):
        from hmi.audio import AlertSound, choose_pattern
        self.assertEqual(choose_pattern({"risk_level": 3}), "danger")
        self.assertEqual(choose_pattern({"risk_level": 2}), "warning")
        self.assertEqual(choose_pattern({"risk_level": 1}), "caution")
        self.assertEqual(choose_pattern({"risk_level": 0, "failsafe_active": True}), "fault")
        self.assertEqual(choose_pattern({"risk_level": 0}), None)
        self.assertEqual(choose_pattern({"risk_level": 2, "demo_collided": True}), "collision")

        a = AlertSound(enabled=False)        # 재생기 없이 타이밍만
        a.muted = False
        plays = []
        a._play = lambda key, now: (plays.append((key, now)), setattr(a, "last_play", now))
        for i in range(31):                  # L2 3초 → 1.2 s 마다: 0, 1.2, 2.4
            a.update({"risk_level": 2}, now=i * 0.1)
        self.assertEqual([p[0] for p in plays], ["warning"] * 3)
        plays.clear()
        for i in range(10):                  # L1 은 진입 시 1회
            a.update({"risk_level": 1}, now=10 + i * 0.5)
        self.assertEqual(len(plays), 1)

    def test_condition_normalize(self):
        c = state_merge._conditions([[1, 1, 0], [0, 1, 0], [0, 0, 0]])
        self.assertEqual(c["1"], [True, True, False])
        self.assertIsNone(state_merge._conditions({"1": [1, 2]}))


class MotorStatusTest(unittest.TestCase):
    def test_0x210_layout_matches_safety_decoder(self):
        sys.path.insert(0, os.path.join(os.path.dirname(ROOT), "safety_ecu"))
        from canbus.decoder import decode_motor_status as safety_decode
        data = bytes([22, 100, *struct.pack("<H", 1111), 1, 0x00, 7, 9])
        _n, d = can_decoder.decode(0x210, data)
        ref = safety_decode(data)
        self.assertEqual(d["actual_pwm"], ref["actual_pwm"])
        self.assertEqual(d["applied_limit"], ref["applied_output_limit"])
        self.assertAlmostEqual(d["speed_mps"], ref["vehicle_speed"])
        self.assertAlmostEqual(d["speed_kmh"], 11.11 * 3.6)
        self.assertEqual(d["motor_state_name"], ref["motor_state_name"])

    def test_invalid_speed_and_estop(self):
        data = bytes([0, 0, 0xFF, 0xFF, 0, 0x08, 0, 1])
        _n, d = can_decoder.decode(0x210, data)
        self.assertIsNone(d["speed_kmh"])
        self.assertTrue(d["estop_active"])
        s = state_merge.merge(can_store(1.0, MOTOR_STATUS=(0x210, data)), None, now=1.0)
        self.assertEqual(s["failsafe_reason"], "MOTOR_ESTOP")

    def test_speed_source_priority(self):
        motor = bytes([22, 100, *struct.pack("<H", 1000), 1, 0, 0, 1])     # 10 m/s = 36 km/h
        store = can_store(1.0, MOTOR_STATUS=(0x210, motor))
        s = state_merge.merge(store, None, now=1.0)
        self.assertEqual(s["speed_source"], "Motor ECU")
        self.assertAlmostEqual(s["speed_kmh"], 36.0)
        state_merge.apply_carla(s, {"ego": {"speed_kmh": 50.0}})
        self.assertEqual(s["speed_source"], "CARLA")                       # 노션: CARLA 모드면 CARLA 속도
        udp = {"data": {"vehicle_speed": 5.0}, "t": 1.0}                    # Safety 값 m/s
        s2 = state_merge.merge(store, udp, now=1.0)
        self.assertAlmostEqual(s2["speed_kmh"], 18.0)
        self.assertEqual(s2["speed_source"], "Safety ECU")

    def test_no_motor_status_stop_command(self):
        cmd = bytes([2, 0, 0, 0x03, 0, 0, 0, 1])
        s = state_merge.merge(can_store(1.0, MOTOR_COMMAND=(0x120, cmd)), None, now=1.0)
        self.assertEqual(s["failsafe_reason"], "MOTOR_NO_RESPONSE")
