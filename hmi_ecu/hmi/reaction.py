"""
운전자 반응 시간 측정 (행동교정 효과 입증용)

1회 측정(trial) = 위험 단계가 L2 이상으로 올라간 순간부터
  - 가속 해제 반응 시간 : 경고 시작 → 가속 페달 ≤ 5 %
  - 브레이크 반응 시간  : 경고 시작 → 브레이크 ≥ 10 %
  - 최고 단계, 최소 거리, 최소 TTC, 결과(회피 / 충돌 / 미반응)
종료 조건: L1 이하로 1.5 s 유지, 충돌, 20 s 경과, Safety ECU/DEMO 데이터 끊김

같은 시나리오를 "경고음 ON / OFF", "실차 / DEMO" 로 바꿔 가며 반복하면
조건별 평균 반응 시간을 바로 비교할 수 있음 → 기록 탭 + logs/reaction_log.csv
"""

import csv
import os
import time

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")

TRIGGER_LEVEL = 2
ACCEL_RELEASED = 5.0      # %
BRAKE_PRESSED = 10.0      # %
END_HOLD = 1.5            # s
TIMEOUT = 20.0            # s

FIELDS = [
    "date", "time", "mode", "input", "sound", "trigger_level", "max_level",
    "speed_at_warning_kmh", "distance_at_warning_m", "accel_at_warning",
    "release_s", "brake_s", "min_distance_m", "min_ttc_s", "outcome",
]


def _r(v, nd=2):
    return None if v is None else round(v, nd)


class ReactionRecorder:
    def __init__(self, log_dir=LOG_DIR, write_csv=True):
        self.log_dir = log_dir
        self.write_csv = write_csv
        self.trial = None
        self.history = []          # 최신순
        self._calm_since = None
        self._armed = True         # 한 번 측정이 끝나면 단계가 내려갈 때까지 다시 시작 안 함

    # -------------------------------------------------------------
    def update(self, s, now=None, sound_on=True):
        """매 틱 호출. 측정이 끝나면 결과 dict 반환."""
        now = time.monotonic() if now is None else now
        level = s.get("risk_level")
        acc = s.get("accelerator")
        brk = s.get("brake")
        live = s.get("risk_available") and level is not None

        if self.trial is None:
            if not live or level < TRIGGER_LEVEL:
                if not s.get("demo_collided"):
                    self._armed = True
                return None
            if self._armed and not s.get("demo_collided"):
                self._start(s, now, sound_on)
            return None

        t = self.trial
        if not live:
            return self._finish(now, "데이터 끊김")

        t["max_level"] = max(t["max_level"], level)
        dist = s.get("distance_m")
        ttc = s.get("ttc_s")
        if dist is not None:
            t["min_distance_m"] = dist if t["min_distance_m"] is None else min(t["min_distance_m"], dist)
        if ttc is not None:
            t["min_ttc_s"] = ttc if t["min_ttc_s"] is None else min(t["min_ttc_s"], ttc)

        if t["release_s"] is None and acc is not None and acc <= ACCEL_RELEASED:
            t["release_s"] = now - t["t0"]
        if t["brake_s"] is None and brk is not None and brk >= BRAKE_PRESSED:
            t["brake_s"] = now - t["t0"]
            if t["release_s"] is None:
                t["release_s"] = t["brake_s"]

        if s.get("demo_collided"):
            return self._finish(now, "충돌")
        if now - t["t0"] > TIMEOUT:
            return self._finish(now, "시간 초과")

        if level < TRIGGER_LEVEL:
            if self._calm_since is None:
                self._calm_since = now
            elif now - self._calm_since >= END_HOLD:
                return self._finish(now, None)
        else:
            self._calm_since = None
        return None

    def reset(self):
        self.trial = None
        self._calm_since = None
        self._armed = True

    # -------------------------------------------------------------
    def _start(self, s, now, sound_on):
        acc = s.get("accelerator")
        self.trial = {
            "t0": now,
            "stamp": time.time(),
            "mode": "DEMO" if s.get("demo_drive") else "LIVE",
            "input": s.get("demo_input") or s.get("pedal_source") or "-",
            "sound": "ON" if sound_on else "OFF",
            "trigger_level": s.get("risk_level"),
            "max_level": s.get("risk_level"),
            "speed_at_warning_kmh": s.get("speed_kmh"),
            "distance_at_warning_m": s.get("distance_m"),
            "accel_at_warning": acc,
            # 경고 순간 이미 가속 페달을 안 밟고 있었으면 해제 반응 0 s
            "release_s": 0.0 if (acc is not None and acc <= ACCEL_RELEASED) else None,
            "brake_s": None,
            "min_distance_m": s.get("distance_m"),
            "min_ttc_s": s.get("ttc_s"),
        }
        self._calm_since = None

    def _finish(self, now, forced):
        t = self.trial
        self.trial = None
        self._calm_since = None
        # 정상 종료(단계가 이미 내려옴)면 바로 다음 측정 가능, 충돌/시간초과는 단계가 내려갈 때까지 대기
        self._armed = forced is None
        if forced == "충돌":
            outcome = "충돌"
        elif forced == "데이터 끊김":
            outcome = "중단"
        elif t["release_s"] is None and t["brake_s"] is None:
            outcome = "미반응"
        elif forced == "시간 초과":
            outcome = "시간 초과"
        else:
            outcome = "회피"

        lt = time.localtime(t["stamp"])
        result = {
            "date": time.strftime("%Y-%m-%d", lt),
            "time": time.strftime("%H:%M:%S", lt),
            "mode": t["mode"],
            "input": t["input"],
            "sound": t["sound"],
            "trigger_level": t["trigger_level"],
            "max_level": t["max_level"],
            "speed_at_warning_kmh": _r(t["speed_at_warning_kmh"], 1),
            "distance_at_warning_m": _r(t["distance_at_warning_m"], 1),
            "accel_at_warning": _r(t["accel_at_warning"], 0),
            "release_s": _r(t["release_s"]),
            "brake_s": _r(t["brake_s"]),
            "min_distance_m": _r(t["min_distance_m"], 1),
            "min_ttc_s": _r(t["min_ttc_s"]),
            "outcome": outcome,
        }
        self.history.insert(0, result)
        del self.history[200:]
        if self.write_csv:
            self._append_csv(result)
        return result

    def _append_csv(self, row):
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            path = os.path.join(self.log_dir, "reaction_log.csv")
            new = not os.path.exists(path)
            with open(path, "a", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=FIELDS)
                if new:
                    w.writeheader()
                w.writerow({k: ("" if row.get(k) is None else row.get(k)) for k in FIELDS})
        except OSError:
            pass

    # -------------------------------------------------------------
    def summary(self):
        """조건(소리 ON/OFF)별 평균."""
        groups = {}
        for r in self.history:
            g = groups.setdefault(r["sound"], {"n": 0, "release": [], "brake": [], "collisions": 0})
            g["n"] += 1
            if r["release_s"] is not None:
                g["release"].append(r["release_s"])
            if r["brake_s"] is not None:
                g["brake"].append(r["brake_s"])
            if r["outcome"] == "충돌":
                g["collisions"] += 1
        out = []
        for key in ("ON", "OFF"):
            g = groups.get(key)
            if not g:
                continue
            out.append({
                "sound": key,
                "n": g["n"],
                "release_avg": _r(sum(g["release"]) / len(g["release"])) if g["release"] else None,
                "brake_avg": _r(sum(g["brake"]) / len(g["brake"])) if g["brake"] else None,
                "collisions": g["collisions"],
            })
        return out
