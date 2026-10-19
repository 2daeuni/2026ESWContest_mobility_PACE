"""
CARLA 단독 판정 (Safety ECU 없이 CARLA 만 연결됐을 때)

Safety ECU 가 꺼져 있으면 아무도 위험도를 계산하지 않아서 경고가 안 뜸.
→ CARLA 브리지가 보낸 실제 값으로 HMI 가 직접 판정 (DEMO 와 같은 로직, safety_ecu/risk/risk_manager.py 구조)
  - 거리: 앞차 범퍼 거리 / 벽·건물 등 정적 장애물 거리 중 가까운 것
  - TTC : 거리 / 접근 속도
  - 페달: CAN 페달(0x100) 이 있으면 그 값, 없으면 CARLA 차량의 throttle/brake
  - 3 조건 중 2 개 → 단계 후보, 상승 3 회 / 하강 5 회 확인
Safety ECU 가 연결되면 이 판정은 쓰지 않음 (Safety ECU 값이 우선)
"""

from hmi.demo_drive import DOWN_CONFIRM, RISK_EVAL_PERIOD, THRESHOLDS, UP_CONFIRM, threshold_labels


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class LocalRisk:
    def __init__(self):
        self.reset()

    def reset(self):
        self.level = 0
        self._prev_candidate = 0
        self._count = 0
        self._acc_t = 0.0
        self._prev_acc = None
        self._prev_dist = None

    @staticmethod
    def conditions(level, dist, ttc, acc, acc_rate):
        th = THRESHOLDS[level]
        return [dist is not None and dist < th["distance"],
                ttc is not None and ttc < th["ttc"],
                acc >= th["accel"] or acc_rate >= th["accel_rate"]]

    def _evaluate(self, dist, ttc, acc, acc_rate):
        candidate = 0
        for lv in (3, 2, 1):
            if sum(self.conditions(lv, dist, ttc, acc, acc_rate)) >= 2:
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

    def step(self, dt, dist, closing, acc, brk, acc_rate=None, collided=False):
        """
        dist    : 전방 범퍼 거리 m (None = 앞에 없음)
        closing : 접근 속도 m/s (+ 가까워짐)
        acc/brk : 페달 %
        """
        dt = _clamp(dt, 0.0, 0.2)
        if acc_rate is None:
            acc_rate = 0.0 if self._prev_acc is None or dt <= 0 else (acc - self._prev_acc) / dt
        self._prev_acc = acc
        if closing is None and dist is not None and self._prev_dist is not None and dt > 0:
            closing = (self._prev_dist - dist) / dt
        self._prev_dist = dist
        ttc = dist / closing if (dist is not None and closing is not None and closing > 0.3) else None
        if ttc is not None and ttc > 99:
            ttc = None

        self._acc_t += dt
        while self._acc_t >= RISK_EVAL_PERIOD:
            self._acc_t -= RISK_EVAL_PERIOD
            self._evaluate(dist, ttc, acc, acc_rate)
        if collided:
            self.level = 3

        d = 200.0 if dist is None else dist
        ttc_risk = 0.0 if ttc is None else _clamp((6.0 - ttc) / 5.0, 0, 1)
        dist_risk = _clamp((80.0 - d) / 75.0, 0, 1)
        bns = _clamp(0.55 * ttc_risk + 0.45 * dist_risk, 0, 1)
        pms = _clamp(0.7 * acc / 100.0 + 0.3 * max(acc_rate, 0.0) / 300.0, 0, 1)
        return {
            "risk_level": self.level,
            "distance_m": dist,
            "ttc_s": ttc,
            "accelerator_rate": acc_rate,
            "bns": bns,
            "pms": pms,
            "total_risk": _clamp(0.6 * bns + 0.4 * pms, 0, 1),
            "conditions": {str(lv): self.conditions(lv, dist, ttc, acc, acc_rate) for lv in (1, 2, 3)},
        }


THRESHOLD_LABELS = threshold_labels()
