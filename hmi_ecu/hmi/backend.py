"""
Qt 쪽 브리지. 소스 스레드 → queue → (Qt 메인 스레드 타이머) → QML 프로퍼티.

QML 에서는 아래만 사용:
    hmi.state   : 현재 표시 상태 dict (state_merge.merge 결과)
    hmi.events  : 최신순 이벤트 리스트
    hmi.diag    : 진단 탭 행 리스트
    hmi.mode    : "LIVE" / "SCRIPTED"
    hmi.state.scene : 도로 장면 (CARLA / DEMO / SENSOR)
    slots       : toggleDemo, demoReset, setDemoStartDistance, setDemoLeadSpeed, setKey
"""

import queue
import threading
import time

from PySide6.QtCore import QObject, Property, QTimer, Signal, Slot
from PySide6.QtGui import QImage
from PySide6.QtQuick import QQuickImageProvider

from hmi import can_decoder, config, events, scene, state_merge
from hmi.audio import AlertSound
from hmi.reaction import ReactionRecorder
from hmi.demo_drive import DemoDrive
from hmi.local_risk import LocalRisk


class CameraImageProvider(QQuickImageProvider):
    """QML: Image { source: "image://carla/" + hmi.camera.frameId } → 최신 CARLA 영상 프레임"""

    def __init__(self):
        super().__init__(QQuickImageProvider.ImageType.Image)
        self.image = QImage()

    def requestImage(self, image_id, size, requested_size):
        img = self.image
        if img.isNull():
            img = QImage(16, 9, QImage.Format.Format_RGB32)
            img.fill(0)
        return img


class HmiBackend(QObject):
    cameraChanged = Signal()
    carlaMapChanged = Signal()
    stateChanged = Signal()
    eventsChanged = Signal()
    diagChanged = Signal()
    trialsChanged = Signal()
    trialFinished = Signal("QVariantMap")

    def __init__(self, mode="LIVE", sound=True, parent=None):
        super().__init__(parent)
        self._mode = mode
        self._audio = AlertSound(enabled=sound)
        self._reaction = ReactionRecorder()
        self._perf = {"fps": None, "tick_ms": 0.0, "cpu": None, "input_ms": None}
        self._cpu_prev = (time.process_time(), time.monotonic())
        self._input_lat = []
        self._inbox = queue.Queue(maxsize=20000)

        self._can_store = {}      # name -> {"data", "t", "id", "raw", "count"}
        self._udp_store = None    # {"data", "t", "count", "seq"}
        self._carla_store = None  # {"data", "t", "count"}
        self._carla_ui_store = None  # UI-only booleans, independent freshness.
        self._carla_status = {"ok": False, "text": "대기"}
        self._map_parts = {}          # ver -> {part: packet}
        self._carla_map = {"ver": None}
        self._collision_n = None
        self._local = LocalRisk()
        self._carla_crash = None      # DEMO·CARLA 충돌: RESET 누를 때까지 유지 (부딪힌 대상)
        self._crash_seen_n = None
        self.camera_provider = CameraImageProvider()
        self._cam_pending = None          # 아직 디코딩 안 한 최신 (header, jpeg, t)
        self._camera_lock = threading.Lock()
        self._camera_latest = None
        self._cam = {"fresh": False, "frameId": 0, "w": 0, "h": 0, "mode": "", "overlay": {}, "fps": 0.0, "kb": 0}
        self._cam_t = None
        self._cam_count = 0
        self._cam_status = {"ok": False, "text": "대기"}

        self._sprites = scene.SpriteLibrary()
        self._demo = None                       # DemoDrive (DEMO 버튼 켜졌을 때)
        self._demo_cfg = {"start": config.DEMO_START_DISTANCE, "lead": config.DEMO_LEAD_SPEED}
        self._keys = {"accel": False, "brake": False}
        self._last_tick = time.monotonic()
        self._can_status = {"ok": False, "text": "대기"}
        self._udp_status = {"ok": False, "text": "대기"}

        self._state = state_merge.merge({}, None)
        self._prev_state = state_merge.merge({}, None)   # 시작 시 연결 이벤트도 기록
        self._events = []
        self._diag = []
        self._dropped = 0

        self._rate_prev = {}
        self._rates = {}
        self._rate_time = time.monotonic()

        self._timer = QTimer(self)
        self._timer.setInterval(int(1000 / config.UI_REFRESH_HZ))
        self._timer.timeout.connect(self._tick)
        self._timer.start()

        self._diag_timer = QTimer(self)
        self._diag_timer.setInterval(500)
        self._diag_timer.timeout.connect(self._update_diag)
        self._diag_timer.start()

        self._add_event("info", "HMI 시작 ({})".format(mode))

    # ------------------------------------------------------------
    # 소스 스레드에서 호출 (thread-safe)
    # ------------------------------------------------------------
    def sink(self, kind, payload, timestamp):
        if kind == "camera":
            with self._camera_lock:
                self._camera_latest = (payload[0], payload[1], timestamp)
            return
        try:
            self._inbox.put_nowait((kind, payload, timestamp))
        except queue.Full:
            self._dropped += 1

    # ------------------------------------------------------------
    # Qt 메인 스레드
    # ------------------------------------------------------------
    def _process(self, kind, payload, t):
        if kind == "can":
            can_id, data = payload
            decoded = can_decoder.decode(can_id, data)
            if decoded is None:
                return
            name, fields = decoded
            entry = self._can_store.get(name)
            count = entry["count"] + 1 if entry else 1
            self._can_store[name] = {
                "data": fields, "t": t, "id": can_id,
                "raw": " ".join("{:02X}".format(b) for b in data), "count": count,
            }
        elif kind == "udp":
            count = self._udp_store["count"] + 1 if self._udp_store else 1
            self._udp_store = {"data": payload, "t": t, "count": count, "seq": payload.get("seq")}
        elif kind == "carla" and payload.get("type") == "carla_map":
            self._on_map_part(payload)
        elif kind == "carla" and payload.get("type") == "carla_ui_state":
            fields = ("night_mode", "lane_assist_enabled", "lane_assist_active")
            if all(isinstance(payload.get(field), bool) for field in fields):
                self._carla_ui_store = {
                    "data": {field: payload[field] for field in fields}, "t": t,
                }
        elif kind == "carla" and payload.get("type") in (None, "carla_scene"):
            count = self._carla_store["count"] + 1 if self._carla_store else 1
            self._carla_store = {"data": payload, "t": t, "count": count}
            col = payload.get("collision")
            if col and col.get("n") != self._collision_n:
                if self._collision_n is not None or (col.get("age") or 99) < 1.0:
                    self._add_event("danger", "CARLA 충돌 감지: " + str(col.get("with", "")))
                self._collision_n = col.get("n")
        elif kind == "camera":
            self._cam_pending = (payload[0], payload[1], t)    # 최신 것만 남김 (밀린 프레임은 디코딩 안 함)
        elif kind == "camera_status":
            if payload != self._cam_status:
                self._cam_status = payload
                self._add_event("ok" if payload["ok"] else "fault", "CARLA 영상 " + payload["text"])
        elif kind == "carla_status":
            if payload != self._carla_status:
                self._carla_status = payload
                self._add_event("ok" if payload["ok"] else "fault", "CARLA 장면 " + payload["text"])
        elif kind == "can_status":
            if payload != self._can_status:
                self._can_status = payload
                self._add_event("ok" if payload["ok"] else "fault", "CAN: " + payload["text"])
        elif kind == "udp_status":
            if payload != self._udp_status:
                self._udp_status = payload
                self._add_event("ok" if payload["ok"] else "fault", payload["text"])

    @Slot()
    def _tick(self):
        tick_start = time.perf_counter()
        # 한 틱에 너무 오래 걸리지 않도록 최대 개수 제한 (100 Hz x 5 프레임 = 500/s)
        for _ in range(2000):
            try:
                kind, payload, t = self._inbox.get_nowait()
            except queue.Empty:
                break
            self._process(kind, payload, t)

        now = time.monotonic()
        with self._camera_lock:
            if self._camera_latest is not None:
                self._cam_pending = self._camera_latest
                self._camera_latest = None
        pedal = self._can_store.get("PEDAL_STATUS")
        if pedal is not None and pedal.get("lat_mark") != pedal["count"]:
            # 페달 프레임 수신 → 화면 상태 반영까지 걸린 시간
            pedal["lat_mark"] = pedal["count"]
            self._input_lat.append((now - pedal["t"]) * 1000)
            del self._input_lat[:-60]
        dt = now - self._last_tick
        self._last_tick = now

        new_state = state_merge.merge(self._can_store, self._udp_store, now)

        carla = None
        if self._carla_store and now - self._carla_store["t"] < config.CARLA_SCENE_TIMEOUT:
            carla = self._carla_store["data"]
            state_merge.apply_carla(new_state, carla)

        self._update_camera(now, new_state.get("gear"))

        state_merge.apply_carla_ui(new_state, self._carla_ui_store, now)

        demo_out = None
        if self._demo is not None and carla is not None:
            # DEMO + CARLA 연결: 가상 차량 대신 CARLA 실제 주행 값으로 HMI 가 판정 (화면 안에서만, CAN 송신 없음)
            col = carla.get("collision") or {}
            fresh_hit = (col.get("age") if col.get("age") is not None else 99) < 1.5
            if fresh_hit and col.get("n") != self._crash_seen_n and self._carla_crash is None:
                self._carla_crash = col.get("with", "") or "obstacle"      # RESET 전까지 충돌 화면 유지
                self._add_event("danger", "DEMO 충돌 (CARLA: {}) · RESET 으로 다시 시작".format(self._carla_crash))
            if fresh_hit:
                self._crash_seen_n = col.get("n")
            collided = self._carla_crash is not None
            inp = state_merge.carla_inputs(new_state, carla)
            out = self._local.step(dt, inp["dist"], inp["closing"], inp["acc"], inp["brk"],
                                   acc_rate=inp["rate"], collided=collided)
            state_merge.apply_carla_local(new_state, inp, out, collided, self._carla_crash or "")
        elif self._demo is not None:
            real_pedal = new_state.get("pedal_source") in ("CAN", "UDP") and new_state.get("accelerator") is not None
            self._demo.key_accel = self._keys["accel"]
            self._demo.key_brake = self._keys["brake"]
            was_collided = self._demo.collided
            demo_out = self._demo.step(
                dt,
                pedal_accel=new_state["accelerator"] if real_pedal else None,
                pedal_brake=new_state["brake"] if real_pedal else None,
                pedal_accel_rate=new_state["accelerator_rate"] if real_pedal else None,
            )
            state_merge.apply_demo(new_state, demo_out)
            if demo_out["collided"] and not was_collided:
                self._add_event("danger", "DEMO 충돌 ({:.0f} km/h 에서)".format(self._prev_state.get("speed_kmh") or 0))

        new_state["scene"] = scene.build_scene(
            self._sprites,
            carla=None if demo_out is not None else carla,
            demo=demo_out,
            distance_m=new_state.get("distance_m"),
        )

        # 청각 경고
        self._audio.update(new_state, now)
        new_state["sound_muted"] = self._audio.muted
        new_state["sound_backend"] = self._audio.backend

        # 반응 시간 측정
        trial = self._reaction.update(new_state, now, sound_on=not self._audio.muted)
        if trial is not None:
            self._add_event("info", "반응 시간: 가속 해제 {} · 브레이크 {} ({})".format(
                "-" if trial["release_s"] is None else "{:.2f} s".format(trial["release_s"]),
                "-" if trial["brake_s"] is None else "{:.2f} s".format(trial["brake_s"]),
                trial["outcome"]))
            self.trialsChanged.emit()
            self.trialFinished.emit(trial)
        new_state["measuring"] = self._reaction.trial is not None

        for ev in events.diff(self._prev_state, new_state):
            self._add_event(ev["severity"], ev["text"], ev["time"])
        self._prev_state = new_state

        self._perf["tick_ms"] = self._perf["tick_ms"] * 0.9 + (time.perf_counter() - tick_start) * 1000 * 0.1

        if new_state != self._state:
            self._state = new_state
            self.stateChanged.emit()

    def _on_map_part(self, pkt):
        ver = pkt.get("ver")
        if ver is None or ver == self._carla_map.get("ver"):
            return                                  # 이미 받은 버전 (2 초마다 재전송분)
        parts = self._map_parts.setdefault(ver, {})
        parts[pkt.get("part", 0)] = pkt
        if len(parts) < int(pkt.get("parts", 1)):
            return
        lanes, boxes = [], []
        for i in sorted(parts):
            lanes += parts[i].get("lanes") or []
            boxes += parts[i].get("boxes") or []
        m = scene.build_map_scene(lanes, boxes)
        m["ver"] = ver
        self._carla_map = m
        self._map_parts = {k: v for k, v in self._map_parts.items() if k > ver}
        self.carlaMapChanged.emit()

    def _update_camera(self, now, gear):
        cam = self._cam
        changed = False
        if self._cam_pending is not None:
            header, jpeg, t = self._cam_pending
            self._cam_pending = None
            frame_view = header.get("view")
            if (gear not in ("D", "R") or frame_view == gear
                    or (gear == "D" and frame_view is None)):
                img = QImage()
            else:
                img = None
            if img is not None and img.loadFromData(jpeg, "JPG"):
                self.camera_provider.image = img
                self._cam_t = t
                self._cam_count += 1
                cam = dict(cam)
                cam.update({
                    "frameId": cam["frameId"] + 1,
                    "w": img.width(), "h": img.height(),
                    "mode": header.get("mode", ""),
                    "view": header.get("view"),
                    "overlay": header.get("overlay") or {},
                    "kb": len(jpeg) // 1024,
                })
                changed = True
        fresh = (self._cam_t is not None and now - self._cam_t < config.CAMERA_TIMEOUT
                 and (gear not in ("D", "R") or cam.get("view") == gear
                      or (gear == "D" and cam.get("view") is None)))
        if fresh != cam["fresh"]:
            cam = dict(cam)
            cam["fresh"] = fresh
            changed = True
            self._add_event("ok" if fresh else "caution", "CARLA 카메라 영상 " + ("수신" if fresh else "끊김 → 그래픽 화면"))
        if changed:
            self._cam = cam
            self.cameraChanged.emit()

    def _add_event(self, severity, text, stamp=None):
        self._events.insert(0, {
            "time": stamp or time.strftime("%H:%M:%S"),
            "severity": severity,
            "text": text,
        })
        del self._events[config.EVENT_LOG_MAX:]
        self.eventsChanged.emit()

    @Slot()
    def _update_diag(self):
        now = time.monotonic()
        span = max(1e-3, now - self._rate_time)
        rows = []

        for can_id, (name, _fn) in sorted(can_decoder.DECODERS.items()):
            entry = self._can_store.get(name)
            count = entry["count"] if entry else 0
            rate = (count - self._rate_prev.get(name, 0)) / span
            self._rate_prev[name] = count
            age = (now - entry["t"]) * 1000 if entry else None
            rows.append({
                "kind": "CAN",
                "id": "0x{:03X}".format(can_id),
                "name": name,
                "rate": round(rate, 1),
                "age": None if age is None else int(age),
                "ok": entry is not None and age < config.CAN_FRESH_TIMEOUT * 1000,
                "raw": entry["raw"] if entry else "-",
                "alive": entry["data"].get("alive") if entry else None,
                "count": count,
            })

        ucount = self._udp_store["count"] if self._udp_store else 0
        urate = (ucount - self._rate_prev.get("UDP", 0)) / span
        self._rate_prev["UDP"] = ucount
        uage = (now - self._udp_store["t"]) * 1000 if self._udp_store else None
        rows.append({
            "kind": "UDP",
            "id": str(config.UDP_PORT),
            "name": "SAFETY_HMI_STATE",
            "rate": round(urate, 1),
            "age": None if uage is None else int(uage),
            "ok": uage is not None and uage < config.SAFETY_UDP_TIMEOUT * 1000,
            "raw": "seq={}".format(self._udp_store.get("seq")) if self._udp_store else "-",
            "alive": None,
            "count": ucount,
        })

        ccount = self._carla_store["count"] if self._carla_store else 0
        crate = (ccount - self._rate_prev.get("CARLA", 0)) / span
        self._rate_prev["CARLA"] = ccount
        cage = (now - self._carla_store["t"]) * 1000 if self._carla_store else None
        nveh = len(self._carla_store["data"].get("vehicles", [])) if self._carla_store else 0
        rows.append({
            "kind": "UDP",
            "id": str(config.CARLA_SCENE_PORT),
            "name": "CARLA_SCENE",
            "rate": round(crate, 1),
            "age": None if cage is None else int(cage),
            "ok": cage is not None and cage < config.CARLA_SCENE_TIMEOUT * 1000,
            "raw": "vehicles={} sprites={}".format(nveh, self._sprites.count) if self._carla_store else "sprites={}".format(self._sprites.count),
            "alive": None,
            "count": ccount,
        })

        vrate = (self._cam_count - self._rate_prev.get("CAM", 0)) / span
        self._rate_prev["CAM"] = self._cam_count
        vage = (now - self._cam_t) * 1000 if self._cam_t is not None else None
        self._cam["fps"] = round(vrate, 1)
        rows.append({
            "kind": "TCP",
            "id": str(config.CAMERA_PORT),
            "name": "CARLA_CAMERA",
            "rate": round(vrate, 1),
            "age": None if vage is None else int(vage),
            "ok": vage is not None and vage < config.CAMERA_TIMEOUT * 1000,
            "raw": "{}x{} {} {} KB".format(self._cam["w"], self._cam["h"], self._cam["mode"], self._cam["kb"]) if self._cam_t else "-",
            "alive": None,
            "count": self._cam_count,
        })

        cpu_now = (time.process_time(), now)
        dcpu = cpu_now[0] - self._cpu_prev[0]
        dwall = max(1e-3, cpu_now[1] - self._cpu_prev[1])
        self._cpu_prev = cpu_now
        self._perf["cpu"] = round(100.0 * dcpu / dwall, 1)
        self._perf["input_ms"] = round(sum(self._input_lat) / len(self._input_lat), 1) if self._input_lat else None

        self._rate_time = now
        self._diag = rows
        self.diagChanged.emit()

    # ------------------------------------------------------------
    # DEMO 주행 (QML 버튼에서 호출)
    # ------------------------------------------------------------
    @Slot()
    def toggleDemo(self):
        if self._demo is None:
            self._demo = DemoDrive(self._demo_cfg["start"], self._demo_cfg["lead"])
            carla_on = self._carla_store is not None and time.monotonic() - self._carla_store["t"] < config.CARLA_SCENE_TIMEOUT
            self._add_event("info", "DEMO 주행 시작 (CARLA 연결: 실제 주행 값으로 판정)" if carla_on
                            else "DEMO 주행 시작 (전방 {:.0f} m)".format(self._demo_cfg["start"]))
        else:
            self._demo = None
            self._add_event("info", "DEMO 주행 종료")
        self._local.reset()
        self._carla_crash = None
        self._reaction.reset()
        self._tick()

    @Slot()
    def demoReset(self):
        if self._carla_crash is not None:
            self._carla_crash = None
            self._local.reset()
            self._reaction.reset()
            self._add_event("info", "DEMO 리셋 (CARLA)")
            return
        if self._demo is not None:
            self._demo.reset()
            self._reaction.reset()
            self._add_event("info", "DEMO 리셋 (전방 {:.0f} m)".format(self._demo.start_distance))

    @Slot(float)
    def setDemoStartDistance(self, meters):
        self._demo_cfg["start"] = float(meters)
        if self._demo is not None:
            self._demo.set_start_distance(meters)
            self._add_event("info", "DEMO 전방 거리 {:.0f} m".format(meters))

    @Slot(float)
    def setDemoLeadSpeed(self, kmh):
        self._demo_cfg["lead"] = float(kmh)
        if self._demo is not None:
            self._demo.set_lead_speed(kmh)

    @Slot()
    def toggleMute(self):
        self._audio.set_muted(not self._audio.muted)
        self._add_event("info", "경고음 " + ("끔" if self._audio.muted else "켬"))
        self._tick()

    @Slot(float)
    def reportFps(self, fps):
        self._perf["fps"] = round(fps, 1)

    @Slot()
    def clearTrials(self):
        self._reaction.history.clear()
        self._reaction.reset()
        self.trialsChanged.emit()

    @Slot(str, bool)
    def setKey(self, name, pressed):
        if name in self._keys:
            self._keys[name] = bool(pressed)

    @Slot()
    def rescanSprites(self):
        self._sprites.rescan()

    # ------------------------------------------------------------
    # QML 프로퍼티
    # ------------------------------------------------------------
    def _get_state(self):
        return self._state

    def _get_events(self):
        return self._events

    def _get_diag(self):
        return self._diag

    def _get_mode(self):
        return self._mode

    def _get_links(self):
        return {"can": self._can_status, "udp": self._udp_status, "carla": self._carla_status,
                "camera": self._cam_status, "dropped": self._dropped}

    def _get_camera(self):
        return self._cam

    def _get_carla_map(self):
        return self._carla_map

    def _get_trials(self):
        return self._reaction.history

    def _get_trial_summary(self):
        return self._reaction.summary()

    def _get_perf(self):
        return dict(self._perf, tick_ms=round(self._perf["tick_ms"], 2))

    trials = Property("QVariantList", _get_trials, notify=trialsChanged)
    trialSummary = Property("QVariantList", _get_trial_summary, notify=trialsChanged)
    perf = Property("QVariantMap", _get_perf, notify=diagChanged)
    state = Property("QVariantMap", _get_state, notify=stateChanged)
    events = Property("QVariantList", _get_events, notify=eventsChanged)
    diag = Property("QVariantList", _get_diag, notify=diagChanged)
    links = Property("QVariantMap", _get_links, notify=diagChanged)
    mode = Property(str, _get_mode, constant=True)
    camera = Property("QVariantMap", _get_camera, notify=cameraChanged)
    carlaMap = Property("QVariantMap", _get_carla_map, notify=carlaMapChanged)
