"""Initial normalized environment risk, separate from the 2-of-3 level policy."""
import math

BNS_DISTANCE_L1_M = 25.0
BNS_DISTANCE_L2_M = 15.0
BNS_DISTANCE_L3_M = 10.0
BNS_TTC_L1_S = 4.0
BNS_TTC_L2_S = 2.5
BNS_TTC_L3_S = 1.5


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _severity(value, limits):
    """Descending anchors: L1=.35, L2=.65, L3=.90, zero distance/TTC=1."""
    if value is None or value > limits[0]:
        return 0.0
    anchors = list(zip((*limits, 0.0), (0.35, 0.65, 0.90, 1.0)))
    for (hi, low_score), (lo, high_score) in zip(anchors, anchors[1:]):
        if value >= lo:
            return low_score + (high_score - low_score) * (hi - value) / (hi - lo)
    return 1.0


class BNSCalculator:
    def __init__(self):
        self.score = 0.0
        self.available = False
        self._clear_conditions()

    def _clear_conditions(self):
        for kind in ("distance", "ttc"):
            for level in (1, 2, 3):
                setattr(self, f"{kind}_l{level}", False)

    def calculate(self, distance_cm, distance_valid, vehicle_speed, ttc,
                  *, closing_speed_mps=None, environment_connected=None):
        """Legacy input units: cm, km/h, seconds; closing speed is signed m/s.

        main supplies environment_connected from CARLA freshness, NOT distance
        validity. A fresh packet without an obstacle is available and safe.
        Speed is validated but adds no unspecified speed-dependent threshold.
        Closing <=0 cannot activate TTC risk; None TTC is never fabricated.
        """
        self.score, self.available = 0.0, False
        self._clear_conditions()
        # Keep old callers' distance-valid semantics; live CARLA passes an
        # explicit connection flag so an absent obstacle is not a link fault.
        connected = distance_valid if environment_connected is None else environment_connected
        if not connected or not _number(vehicle_speed) or vehicle_speed < 0:
            return self.score
        if closing_speed_mps is not None and not _number(closing_speed_mps):
            return self.score
        if distance_cm is not None and (not distance_valid or not _number(distance_cm) or distance_cm < 0):
            return self.score
        if ttc is not None and (not _number(ttc) or ttc < 0 or distance_cm is None):
            return self.score

        self.available = True
        distance_m = distance_cm / 100.0 if distance_cm is not None else None
        usable_ttc = ttc if closing_speed_mps is None or closing_speed_mps > 0 else None
        distance_limits = (BNS_DISTANCE_L1_M, BNS_DISTANCE_L2_M, BNS_DISTANCE_L3_M)
        ttc_limits = (BNS_TTC_L1_S, BNS_TTC_L2_S, BNS_TTC_L3_S)
        for level, (distance_limit, ttc_limit) in enumerate(zip(distance_limits, ttc_limits), 1):
            setattr(self, f"distance_l{level}", distance_m is not None and distance_m <= distance_limit)
            setattr(self, f"ttc_l{level}", usable_ttc is not None and usable_ttc <= ttc_limit)
        self.score = max(_severity(distance_m, distance_limits), _severity(usable_ttc, ttc_limits))
        return self.score
