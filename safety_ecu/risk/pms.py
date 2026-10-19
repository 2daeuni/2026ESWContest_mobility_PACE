"""Initial pedal misuse policy; sensor faults remain the FailSafeManager's job."""
import math

PMS_ACCEL_L1_PERCENT = 60
PMS_ACCEL_L2_PERCENT = 75
PMS_ACCEL_L3_PERCENT = 90
PMS_ACCEL_RATE_L1 = 250  # %/s, already decoded (signed int16 LE * 0.1)
PMS_ACCEL_RATE_L2 = 450
PMS_ACCEL_RATE_L3 = 650
PMS_BRAKE_SUPPRESSION_PERCENT = 5
PMS_CONFLICT_ACCEL_PERCENT = 20
PMS_CONFLICT_BRAKE_PERCENT = 20
PMS_CONFLICT_SCORE = 0.65  # Conflict contributes L1/L2, not a standalone L3.


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _severity(value, limits):
    """Increasing anchors .35/.65/.90; saturate one tier interval above L3."""
    if value < limits[0]:
        return 0.0
    anchors = list(zip((*limits, limits[2] + limits[2] - limits[1]), (0.35, 0.65, 0.90, 1.0)))
    for (lo, low_score), (hi, high_score) in zip(anchors, anchors[1:]):
        if value <= hi:
            return low_score + (high_score - low_score) * (value - lo) / (hi - lo)
    return 1.0


class PMSCalculator:
    def __init__(self):
        self.score = 0.0
        self.available = False
        self._clear_conditions()

    def _clear_conditions(self):
        self.pedal_conflict = False
        self.pedal_l1 = self.pedal_l2 = self.pedal_l3 = False

    def calculate(self, accelerator, accelerator_rate, pedal_connected,
                  sensor_status, *, brake=0, brake_rate=0.0):
        """Percent positions; both rates are signed %/s from 0x100 decoder.

        brake>=5 suppresses acceleration-misuse conditions. Accel>=20 AND
        brake>=20 produces a separate conflict condition with L1/L2 severity.
        Brake rate is accepted/validated, but is not itself acceleration misuse.
        No command/E-STOP logic belongs here.
        """
        self.score, self.available = 0.0, False
        self._clear_conditions()
        if not pedal_connected or sensor_status != 0x00:
            return self.score
        if not all(_number(v) for v in (accelerator, accelerator_rate, brake, brake_rate)):
            return self.score
        if not (0 <= accelerator <= 100 and 0 <= brake <= 100):
            return self.score
        self.available = True
        self.pedal_conflict = accelerator >= PMS_CONFLICT_ACCEL_PERCENT and brake >= PMS_CONFLICT_BRAKE_PERCENT
        acceleration_limits = (PMS_ACCEL_L1_PERCENT, PMS_ACCEL_L2_PERCENT, PMS_ACCEL_L3_PERCENT)
        rate_limits = (PMS_ACCEL_RATE_L1, PMS_ACCEL_RATE_L2, PMS_ACCEL_RATE_L3)
        if brake < PMS_BRAKE_SUPPRESSION_PERCENT:
            for level, (acc_limit, rate_limit) in enumerate(zip(acceleration_limits, rate_limits), 1):
                setattr(self, f"pedal_l{level}", accelerator >= acc_limit or accelerator_rate >= rate_limit)
            self.score = max(_severity(accelerator, acceleration_limits), _severity(accelerator_rate, rate_limits))
        if self.pedal_conflict:
            self.pedal_l1 = self.pedal_l2 = True
            self.score = max(self.score, PMS_CONFLICT_SCORE)
        return self.score
