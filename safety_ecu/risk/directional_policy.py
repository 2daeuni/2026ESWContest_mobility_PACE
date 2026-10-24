import math

from risk.bns import headway_assessment
from risk.pms import PMS_BRAKE_SUPPRESSION_PERCENT


class DirectionalRiskPolicy:
    """CARLA collision policy with direction, braking evidence and timed transitions."""

    BRAKE_DECEL_MPS2 = 6.0
    CONTROL_LATENCY_S = 0.3
    CLEARANCE_M = 1.5
    L1_TTC_S = 4.0
    L2_TTC_S = 2.5
    L3_TTC_S = 1.5
    L1_DISTANCE_FACTOR = 2.5
    L2_DISTANCE_FACTOR = 1.5
    L2_REQUIRED_DECEL_MPS2 = 3.0
    MIN_CLOSING_SPEED_MPS = 0.4
    UP_CONFIRM_S = 0.2
    DOWN_CONFIRM_S = 0.5
    IMPROVING_DOWN_CONFIRM_S = 0.25
    RELEASE_BRAKE_MIN_PERCENT = PMS_BRAKE_SUPPRESSION_PERCENT
    RELEASE_ACCEL_MAX_PERCENT = 3
    RELEASE_STOP_SPEED_KMH = 0.5
    RELEASE_MAX_STATIONARY_CLOSING_MPS = 0.1
    RELEASE_MIN_DECEL_MPS2 = 0.2
    BRAKE_RATE_RESPONSE_MIN_PERCENT_PER_S = 10.0

    def __init__(self):
        self.total_risk = 0.0
        self.bns_score = 0.0
        self.risk_level = 0
        self.candidate_level = 0
        self.available = False
        self.reason = "NO_DATA"
        self.stopping_distance_m = None
        self.required_deceleration_mps2 = None
        self.time_headway_s = None
        self.safe_distance_m = None
        self.risk_cause = "NONE"
        self.normal_braking = False
        self.pedal_misuse_active = False
        self.pedal_conflict = False
        self.observed_deceleration_mps2 = 0.0
        self._gear = None
        self._candidate_since = None
        self._sample = None
        self._escape_target_id = None
        self._escape_distance_m = None
        self._escape_confirmed = False
        self._improving = False
        self._last_hazard_id = None
        self._last_hazard_distance_m = None

    def evaluate(self, *, gear, distance_m, speed_kmh, closing_speed_mps,
                 ttc_s, obstacle_present, bns, bns_available, pms,
                 pms_available, now, brake=0, brake_rate=0.0, accelerator=0, sequence=None,
                 obstacle_id=None, signed_speed_mps=None,
                 escape_obstacle_id=None, escape_distance_m=None,
                 pedal_l1=False, pedal_l2=False, pedal_l3=False,
                 pedal_conflict=False):
        def number(value):
            return type(value) in (int, float) and math.isfinite(value)

        self.available = bool(
            gear in ("D", "R", "P") and bns_available and pms_available
            and number(speed_kmh) and speed_kmh >= 0 and number(now)
            and number(brake) and 0 <= brake <= 100
            and number(brake_rate)
            and number(accelerator) and 0 <= accelerator <= 100
            and (signed_speed_mps is None or number(signed_speed_mps))
            and (not obstacle_present or (
                number(distance_m) and distance_m >= 0
                and number(closing_speed_mps)
                and (ttc_s is None or number(ttc_s) and ttc_s >= 0)
            ))
        )
        if not self.available:
            self.total_risk = self.bns_score = 0.0
            self.risk_level = self.candidate_level = 0
            self.reason = "INVALID_INPUT"
            self.stopping_distance_m = None
            self.required_deceleration_mps2 = None
            self.time_headway_s = None
            self.safe_distance_m = None
            self.risk_cause = "NONE"
            self.normal_braking = False
            self.pedal_misuse_active = False
            self.pedal_conflict = False
            self._candidate_since = None
            self._sample = None
            self._improving = False
            return self.total_risk, self.risk_level

        gear_changed = gear != self._gear
        if gear_changed:
            self._escape_target_id = self._last_hazard_id if self.risk_level > 0 else None
            self._escape_distance_m = self._last_hazard_distance_m if self.risk_level > 0 else None
            self._escape_confirmed = False
            self.candidate_level = 0
            self._candidate_since = now
            self._sample = None
            self._gear = gear
            self._improving = False

        new_sample = self._sample is None or sequence is None or sequence != self._sample["sequence"]
        improving = self._improving
        if new_sample:
            improving = False
            previous = self._sample
            self.observed_deceleration_mps2 = 0.0
            if previous is not None and now > previous["now"]:
                interval = now - previous["now"]
                self.observed_deceleration_mps2 = max(0.0, min(10.0,
                    (previous["speed_kmh"] - speed_kmh) / 3.6 / interval))
                same_target = (obstacle_present and previous["distance_m"] is not None
                               and obstacle_id == previous["obstacle_id"])
                if same_target and brake >= 2 and accelerator <= 3:
                    improving = self.observed_deceleration_mps2 >= 0.2 and (
                        closing_speed_mps <= previous["closing_speed_mps"] - 0.1
                        or distance_m >= previous["distance_m"] + 0.05
                        or ttc_s is not None and previous["ttc_s"] is not None
                        and ttc_s >= previous["ttc_s"] + 0.1
                    )
            if (self._escape_target_id is not None
                and escape_obstacle_id == self._escape_target_id
                and number(escape_distance_m)
                and self._escape_distance_m is not None
                and number(signed_speed_mps)
                and ((gear == "D" and signed_speed_mps > 0.3)
                     or (gear == "R" and signed_speed_mps < -0.3))):
                self._escape_confirmed = escape_distance_m > self._escape_distance_m + 0.05
                self._escape_distance_m = escape_distance_m
            else:
                self._escape_confirmed = False
            self._sample = dict(sequence=sequence, now=now, speed_kmh=speed_kmh,
                                obstacle_id=obstacle_id if obstacle_present else None,
                                distance_m=distance_m, closing_speed_mps=closing_speed_mps,
                                ttc_s=ttc_s)
            self._improving = improving
            if obstacle_present and obstacle_id is not None:
                self._last_hazard_id = obstacle_id
                self._last_hazard_distance_m = distance_m

        controlled_stop = (
            brake >= self.RELEASE_BRAKE_MIN_PERCENT
            and accelerator <= self.RELEASE_ACCEL_MAX_PERCENT
            and speed_kmh <= self.RELEASE_STOP_SPEED_KMH
            and (not obstacle_present or
                 closing_speed_mps <= self.RELEASE_MAX_STATIONARY_CLOSING_MPS)
        )
        self.normal_braking = (
            brake >= self.RELEASE_BRAKE_MIN_PERCENT
            and accelerator <= self.RELEASE_ACCEL_MAX_PERCENT
            and (controlled_stop or self.observed_deceleration_mps2 >= self.RELEASE_MIN_DECEL_MPS2
                 or brake_rate >= self.BRAKE_RATE_RESPONSE_MIN_PERCENT_PER_S)
        )
        self.pedal_conflict = bool(pedal_conflict)
        collision_candidate = 0
        following_candidate = 0
        pedal_candidate = 0
        reason = "CLEAR"
        self.stopping_distance_m = None
        self.required_deceleration_mps2 = None
        self.time_headway_s = None
        self.safe_distance_m = None
        if controlled_stop:
            reason = "CONTROLLED_STOP"
        elif gear in ("D", "R") and obstacle_present:
            following_candidate, self.time_headway_s, self.safe_distance_m, _ = (
                headway_assessment(distance_m, speed_kmh, closing_speed_mps))
            if self.normal_braking and improving and closing_speed_mps <= self.MIN_CLOSING_SPEED_MPS:
                following_candidate = 0
            if distance_m <= 0.25 and closing_speed_mps >= -0.1:
                collision_candidate, reason = 3, "CONTACT"
            elif closing_speed_mps > self.MIN_CLOSING_SPEED_MPS:
                relative_speed = closing_speed_mps
                if distance_m > self.CLEARANCE_M:
                    self.required_deceleration_mps2 = (
                        relative_speed ** 2 / (2 * (distance_m - self.CLEARANCE_M)))
                self.stopping_distance_m = (
                    relative_speed * self.CONTROL_LATENCY_S
                    + relative_speed ** 2 / (2 * self.BRAKE_DECEL_MPS2)
                    + self.CLEARANCE_M
                )
                if distance_m <= self.stopping_distance_m or (
                    ttc_s is not None and ttc_s <= self.L3_TTC_S
                ):
                    collision_candidate, reason = 3, "STOPPING_DISTANCE_OR_TTC"
                elif distance_m <= self.stopping_distance_m * self.L2_DISTANCE_FACTOR or (
                    ttc_s is not None and ttc_s <= self.L2_TTC_S
                ) or (self.required_deceleration_mps2 is not None
                      and self.required_deceleration_mps2 >= self.L2_REQUIRED_DECEL_MPS2):
                    collision_candidate, reason = 2, "DECELERATION_REQUIRED"
                elif distance_m <= self.stopping_distance_m * self.L1_DISTANCE_FACTOR or (
                    ttc_s is not None and ttc_s <= self.L1_TTC_S
                ):
                    collision_candidate, reason = 1, "APPROACH_WARNING"

        hazard_candidate = max(collision_candidate, following_candidate)
        if pedal_conflict:
            pedal_candidate = 2 if hazard_candidate else 1
        elif hazard_candidate and brake < self.RELEASE_BRAKE_MIN_PERCENT:
            pedal_candidate = 2 if pedal_l2 or pedal_l3 else 1 if pedal_l1 else 0
        elif (gear in ("D", "R") and obstacle_present
              and speed_kmh <= self.RELEASE_STOP_SPEED_KMH
              and distance_m <= self.CLEARANCE_M
              and accelerator > self.RELEASE_ACCEL_MAX_PERCENT
              and brake < self.RELEASE_BRAKE_MIN_PERCENT):
            pedal_candidate = 2 if pedal_l2 or pedal_l3 else 1 if pedal_l1 else 0
        self.pedal_misuse_active = pedal_candidate > 0
        candidate = max(collision_candidate, following_candidate, pedal_candidate)
        if candidate > collision_candidate and candidate == pedal_candidate:
            reason = "PEDAL_CONFLICT" if pedal_conflict else "PEDAL_MISUSE"
        elif candidate > collision_candidate and candidate == following_candidate:
            reason = "SHORT_HEADWAY"

        if candidate != self.candidate_level:
            self.candidate_level = candidate
            self._candidate_since = now
        resolved_by_braking = (
            candidate == 0
            and self._escape_target_id is None
            and brake >= self.RELEASE_BRAKE_MIN_PERCENT
            and accelerator <= self.RELEASE_ACCEL_MAX_PERCENT
            and (controlled_stop or (
                self.observed_deceleration_mps2 >= self.RELEASE_MIN_DECEL_MPS2
                and (not obstacle_present or
                     closing_speed_mps <= self.MIN_CLOSING_SPEED_MPS)
            ))
        )
        released_by_braking = resolved_by_braking and self.risk_level > 0
        if candidate == 3:
            self.risk_level = 3
        elif released_by_braking:
            self.risk_level = 0
            self._candidate_since = now
            self._escape_target_id = None
            self._last_hazard_id = None
            self._last_hazard_distance_m = None
        elif candidate > self.risk_level:
            if now - self._candidate_since >= self.UP_CONFIRM_S:
                self.risk_level = candidate
        elif candidate < self.risk_level:
            down_delay = (self.IMPROVING_DOWN_CONFIRM_S
                          if improving or self._escape_confirmed else self.DOWN_CONFIRM_S)
            if now - self._candidate_since >= down_delay:
                self.risk_level -= 1
                self._candidate_since = now
                if self.risk_level == 0:
                    self._escape_target_id = None
                    self._last_hazard_id = None
                    self._last_hazard_distance_m = None
        self.reason = "BRAKE_RELEASE" if released_by_braking else (
            "ESCAPING" if self._escape_confirmed and candidate == 0 else reason)
        if self.risk_level == 0:
            self.risk_cause = "NONE"
        elif candidate == 0:
            self.risk_cause = "RECOVERING"
        elif collision_candidate >= candidate:
            self.risk_cause = (
                "COLLISION_AND_PEDAL" if pedal_candidate > 0 and collision_candidate < 3
                else "COLLISION"
            )
        elif pedal_candidate >= candidate:
            self.risk_cause = "PEDAL_MISUSE"
        else:
            self.risk_cause = "HEADWAY"
        self.bns_score = 0.0 if controlled_stop else max(
            0.0, min(1.0, bns * (0.85 if improving and candidate < 3 else 1.0)))
        self.total_risk = max(self.bns_score, max(0.0, min(1.0, pms)))
        return self.total_risk, self.risk_level
