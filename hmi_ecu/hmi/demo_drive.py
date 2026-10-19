"""
DEMO 주행 모드 (HMI 하단 DEMO 버튼)

- 전방 차량(장애물)을 일정 거리(기본 1 km) 앞에 두고
- 실제 페달(Pedal ECU, CAN 0x100) 입력으로 종방향 차량 모델을 구동
  (CAN 페달이 없으면 키보드 ↑ 가속 / ↓ 또는 Space 브레이크)
- 거리 / TTC / 페달 입력으로 위험 단계를 판정해서 경고 UI 를 실제처럼 체험

위험 판정 구조는 safety_ecu/risk/risk_manager.py 와 동일하게 맞춤
  - 단계별 (거리, TTC, 페달) 3개 조건 중 2개 이상 → 해당 단계 후보
  - 상승: 같은 후보 3회 연속, 하강: 5회 연속 + 한 단계씩
  - 단계별 햅틱/모터 명령 = safety_ecu/control/command_mapper.py 와 동일
임계값은 아직 Safety ECU 에 확정값이 없어서 아래 THRESHOLDS 에 임시값을 둠
(팀에서 확정되면 여기 숫자만 맞추면 됨)

주의: DEMO 판정은 HMI 화면 안에서만 쓰임. 실제 CAN 으로 햅틱/모터 명령을 보내지 않음.
"""

import math


# ---------------------------------------------------------------
# 위험 판정 임계값 (임시) : (거리 m 미만, TTC s 미만, 가속페달 % 이상, 가속 변화율 %/s 이상)
# ---------------------------------------------------------------
THRESHOLDS = {
    1: {"distance": 80.0, "ttc": 6.0, "accel": 10.0, "accel_rate": 80.0},
    2: {"distance": 45.0, "ttc": 4.0, "accel": 30.0, "accel_rate": 150.0},
    3: {"distance": 25.0, "ttc": 2.5, "accel": 50.0, "accel_rate": 250.0},
}

def threshold_labels():
    """화면 표시용 문구: {"1": ["< 80 m", "< 6 s", "≥ 10 %"], ...}"""
    return {
        str(lv): ["< {:.0f} m".format(t["distance"]), "< {:g} s".format(t["ttc"]), "≥ {:.0f} %".format(t["accel"])]
        for lv, t in THRESHOLDS.items()
    }


UP_CONFIRM = 3
DOWN_CONFIRM = 5
RISK_EVAL_PERIOD = 0.05          # 20 Hz

# command_mapper.py 와 동일
HAPTIC_TABLE = {0: (False, 0, 0), 1: (True, 30, 20), 2: (True, 45, 20), 3: (True, 65, 30)}
MOTOR_TABLE = {0: ("NORMAL", 100), 1: ("NORMAL", 100), 2: ("NORMAL", 100), 3: ("LIMIT", 30)}

# ---------------------------------------------------------------
# 차량 모델 (중형 세단/EV 수준)
# ---------------------------------------------------------------
MASS = 1600.0               # kg
POWER_MAX = 120e3           # W
TRACTION_MAX = 5200.0       # N  (저속 최대 구동력, 약 0.33 g)
BRAKE_MAX = 14500.0         # N  (풀 브레이크 약 0.92 g)
REGEN = 450.0               # N  (가속 페달을 뗐을 때 감속)
CDA = 0.68                  # m^2
RHO = 1.2
CRR = 0.012
G = 9.81
V_MAX = 190 / 3.6

KEY_ACCEL_UP = 160.0        # %/s  키보드 가속 페달 밟는 속도
KEY_RELEASE = 320.0         # %/s


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DemoDrive:
    def __init__(self, start_distance=1000.0, lead_speed_kmh=0.0):
        self.start_distance = start_distance
        self.lead_speed_kmh = lead_speed_kmh
        self.key_accel = False
        self.key_brake = False
        self.reset()

    # -----------------------------------------------------------
    # 제어
    # -----------------------------------------------------------
    def reset(self):
        self.ego_pos = 0.0
        self.ego_v = 0.0
        self.lead_pos = self.start_distance
        self.collided = False
        self.level = 0
        self._prev_candidate = 0
        self._count = 0
        self._eval_acc = 0.0
        self._kb_acc = 0.0
        self._kb_brk = 0.0
        self._prev_acc = None
        self.elapsed = 0.0
        self.last = None

    def set_start_distance(self, meters):
        self.start_distance = float(meters)
        self.reset()

    def set_lead_speed(self, kmh):
        self.lead_speed_kmh = float(kmh)

    # -----------------------------------------------------------
    @property
    def distance(self):
        return self.lead_pos - self.ego_pos

    def _keyboard_pedals(self, dt):
        if self.key_accel:
            self._kb_acc = min(100.0, self._kb_acc + KEY_ACCEL_UP * dt)
        else:
            self._kb_acc = max(0.0, self._kb_acc - KEY_RELEASE * dt)
        if self.key_brake:
            self._kb_brk = min(100.0, self._kb_brk + KEY_ACCEL_UP * 1.5 * dt)
        else:
            self._kb_brk = max(0.0, self._kb_brk - KEY_RELEASE * dt)
        return self._kb_acc, self._kb_brk

    def _forces(self, throttle, brake, v):
        drive = 0.0
        if throttle > 0:
            drive = throttle * min(TRACTION_MAX, POWER_MAX / max(v, 1.0))
        resist = 0.5 * RHO * CDA * v * v + (CRR * MASS * G if v > 0.05 else 0.0)
        regen = REGEN if (v > 0.3 and throttle < 0.02) else 0.0
        braking = brake * BRAKE_MAX
        return drive - resist - regen - braking

    def _conditions(self, level, dist, ttc, acc, acc_rate):
        """(거리, TTC, 페달) 조건 충족 여부 - risk_manager.py 의 level_conditions 와 같은 순서"""
        th = THRESHOLDS[level]
        c_dist = dist is not None and dist < th["distance"]
        c_ttc = ttc is not None and ttc < th["ttc"]
        c_pedal = acc >= th["accel"] or acc_rate >= th["accel_rate"]
        return [bool(c_dist), bool(c_ttc), bool(c_pedal)]

    def _condition_count(self, level, dist, ttc, acc, acc_rate):
        return sum(self._conditions(level, dist, ttc, acc, acc_rate))

    def _evaluate(self, dist, ttc, acc, acc_rate):
        candidate = 0
        for lv in (3, 2, 1):
            if self._condition_count(lv, dist, ttc, acc, acc_rate) >= 2:
                candidate = lv
                break

        if candidate == self._prev_candidate:
            self._count += 1
        else:
            self._prev_candidate = candidate
            self._count = 1

        if candidate == self.level:
            self._count = 0
        elif candidate > self.level and self._count >= UP_CONFIRM:
            self.level = candidate
            self._count = 0
        elif candidate < self.level and self._count >= DOWN_CONFIRM:
            self.level -= 1
            self._count = 0
        return candidate

    # -----------------------------------------------------------
    def step(self, dt, pedal_accel=None, pedal_brake=None, pedal_accel_rate=None):
        """
        pedal_* : 실제 페달 값(%) . None 이면 키보드 입력 사용
        반환: 표시용 dict
        """
        dt = _clamp(dt, 0.0, 0.1)
        self.elapsed += dt

        if pedal_accel is None:
            acc, brk = self._keyboard_pedals(dt)
            source = "KEYBOARD"
        else:
            acc, brk = float(pedal_accel), float(pedal_brake or 0.0)
            source = "PEDAL"

        if pedal_accel_rate is not None:
            acc_rate = float(pedal_accel_rate)
        elif self._prev_acc is not None and dt > 0:
            acc_rate = (acc - self._prev_acc) / dt
        else:
            acc_rate = 0.0
        self._prev_acc = acc

        motor_mode, out_limit = MOTOR_TABLE[self.level]

        # ---- 차량 운동 ----
        lead_v = self.lead_speed_kmh / 3.6
        if not self.collided:
            throttle = _clamp(acc / 100.0, 0, 1) * out_limit / 100.0
            brake = _clamp(brk / 100.0, 0, 1)
            a = self._forces(throttle, brake, self.ego_v) / MASS
            self.ego_v = _clamp(self.ego_v + a * dt, 0.0, V_MAX)
            self.ego_pos += self.ego_v * dt
            self.lead_pos += lead_v * dt
            if self.distance <= 0.0:
                self.collided = True
                self.ego_pos = self.lead_pos
                self.ego_v = 0.0
        dist = max(0.0, self.distance)

        closing = self.ego_v - lead_v
        ttc = dist / closing if closing > 0.1 else None
        if ttc is not None and ttc > 99:
            ttc = None

        # ---- 위험 판정 (20 Hz) ----
        self._eval_acc += dt
        while self._eval_acc >= RISK_EVAL_PERIOD:
            self._eval_acc -= RISK_EVAL_PERIOD
            self._evaluate(dist, ttc, acc, acc_rate)
        if self.collided:
            self.level = 3

        haptic_on, h_int, h_freq = HAPTIC_TABLE[self.level]
        motor_mode, out_limit = MOTOR_TABLE[self.level]

        # 표시용 점수 (0..1)
        ttc_risk = 0.0 if ttc is None else _clamp((6.0 - ttc) / 5.0, 0, 1)
        dist_risk = _clamp((80.0 - dist) / 75.0, 0, 1)
        bns = _clamp(0.55 * ttc_risk + 0.45 * dist_risk, 0, 1)
        pms = _clamp(0.7 * acc / 100.0 + 0.3 * max(acc_rate, 0.0) / 300.0, 0, 1)
        total = _clamp(0.6 * bns + 0.4 * pms, 0, 1)

        conditions = {str(lv): self._conditions(lv, dist, ttc, acc, acc_rate) for lv in (1, 2, 3)}

        self.last = {
            "conditions": conditions,
            "accelerator": acc,
            "brake": brk,
            "accelerator_rate": acc_rate,
            "input_source": source,
            "speed_kmh": self.ego_v * 3.6,
            "distance_m": dist,
            "ttc_s": ttc,
            "relative_speed_kmh": (lead_v - self.ego_v) * 3.6,
            "lead_speed_kmh": self.lead_speed_kmh,
            "start_distance": self.start_distance,
            "bns": bns,
            "pms": pms,
            "total_risk": total,
            "risk_level": self.level,
            "haptic_on": haptic_on,
            "haptic_intensity": h_int,
            "haptic_frequency": h_freq,
            "motor_mode": motor_mode,
            "motor_limit": out_limit,
            "collided": self.collided,
            "travelled_m": self.ego_pos,
        }
        return self.last


def _selftest():
    d = DemoDrive(start_distance=300)
    t = 0.0
    while t < 60 and not d.collided:
        out = d.step(0.02, pedal_accel=60, pedal_brake=0, pedal_accel_rate=0)
        t += 0.02
        if int(t * 50) % 50 == 0:
            print("t={:4.1f} v={:5.1f} d={:6.1f} ttc={} L{}".format(
                t, out["speed_kmh"], out["distance_m"],
                None if out["ttc_s"] is None else round(out["ttc_s"], 1), out["risk_level"]))
    print("collided:", d.collided, "t:", round(t, 1))


if __name__ == "__main__":
    _selftest()
