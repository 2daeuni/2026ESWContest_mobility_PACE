import math


class RiskManager:
    # -------------------------------------------------
    # Risk Level definitions
    # -------------------------------------------------
    RISK_LEVEL_NORMAL = 0
    RISK_LEVEL_CAUTION = 1
    RISK_LEVEL_WARNING = 2
    RISK_LEVEL_DANGER = 3

    # -------------------------------------------------
    # State transition confirmation counts
    # -------------------------------------------------

    # Higher Risk Level must be detected
    # consecutively before upward transition.
    UP_CONFIRM_COUNT_REQUIRED = 3

    # Lower Risk Level must remain consecutively
    # before downward transition.
    #
    # This acts as temporal hysteresis and prevents
    # rapid Risk Level oscillation.
    DOWN_CONFIRM_COUNT_REQUIRED = 5

    def __init__(self):
        # -------------------------------------------------
        # Final integrated risk state
        # -------------------------------------------------
        self.total_risk = 0.0
        self.risk_level = self.RISK_LEVEL_NORMAL

        # -------------------------------------------------
        # Availability
        # -------------------------------------------------
        self.bns_available = False
        self.pms_available = False
        self.available = False

        # -------------------------------------------------
        # Risk Level state machine
        # -------------------------------------------------

        # Instantaneous Risk Level calculated from
        # the current conditions.
        self.candidate_level = self.RISK_LEVEL_NORMAL

        # Candidate observed during the previous
        # evaluation cycle.
        self.previous_candidate_level = (
            self.RISK_LEVEL_NORMAL
        )

        # Number of consecutive observations of the
        # same candidate level.
        self.candidate_count = 0

    def evaluate(
        self,
        bns,
        bns_available,
        pms,
        pms_available,
        level1_conditions=None,
        level2_conditions=None,
        level3_conditions=None,
    ):
        """
        Evaluate integrated risk state.

        Total risk is max(BNS, PMS), bounded to [0,1]. Level selection uses
        the supplied distance/TTC/pedal boolean conditions, not score bands.
        Existing 3-up/5-down confirmation policy is unchanged.
        """

        self.bns_available = bns_available
        self.pms_available = pms_available

        # Integrated risk calculation requires
        # both BNS and PMS to be available.
        if not (
            self.bns_available
            and self.pms_available
        ):
            self.total_risk = 0.0
            self.risk_level = self.RISK_LEVEL_NORMAL
            self.available = False

            # Invalid risk input must not preserve
            # an unfinished confirmation sequence.
            self._reset_candidate_state()

            return (
                self.total_risk,
                self.risk_level,
            )

        def normalized(value):
            if type(value) not in (int, float) or not math.isfinite(value):
                return 0.0
            return max(0.0, min(1.0, value))

        self.total_risk = max(normalized(bns), normalized(pms))

        if (
            level1_conditions is not None
            and level2_conditions is not None
            and level3_conditions is not None
        ):
            candidate_level = (
                self.determine_candidate_level(
                    level1_conditions,
                    level2_conditions,
                    level3_conditions,
                )
            )

            self.update_candidate_confirmation(
                candidate_level
            )

        self.available = True

        return (
            self.total_risk,
            self.risk_level,
        )

    def determine_candidate_level(
        self,
        level1_conditions,
        level2_conditions,
        level3_conditions,
    ):
        """
        Determine instantaneous candidate Risk Level
        using the 2-of-3 rule.

        Each level_conditions argument must contain
        exactly three boolean values:

            (
                distance_condition,
                ttc_condition,
                pedal_condition,
            )

        A Risk Level condition is satisfied when
        at least two of the three conditions are True.

        Higher Risk Levels are checked first.
        """

        level3_met = (
            self._count_true_conditions(
                level3_conditions
            )
            >= 2
        )

        level2_met = (
            self._count_true_conditions(
                level2_conditions
            )
            >= 2
        )

        level1_met = (
            self._count_true_conditions(
                level1_conditions
            )
            >= 2
        )

        if level3_met:
            return self.RISK_LEVEL_DANGER

        if level2_met:
            return self.RISK_LEVEL_WARNING

        if level1_met:
            return self.RISK_LEVEL_CAUTION

        return self.RISK_LEVEL_NORMAL

    def update_candidate_confirmation(
        self,
        candidate_level,
    ):
        """
        Update Risk Level state using consecutive
        candidate confirmations.

        Upward transition:
        - candidate > current risk_level
        - same candidate must be detected
          UP_CONFIRM_COUNT_REQUIRED times
        - confirmed candidate is applied directly

        Downward transition:
        - candidate < current risk_level
        - same lower candidate must be detected
          DOWN_CONFIRM_COUNT_REQUIRED times
        - risk_level decreases only one level at a time

        Equal level:
        - candidate == current risk_level
        - current Risk Level is maintained

        Returns:
            True:
                A state transition occurred.

            False:
                No Risk Level transition occurred.
        """

        if not self._is_valid_risk_level(
            candidate_level
        ):
            raise ValueError(
                f"Invalid candidate risk level: "
                f"{candidate_level}"
            )

        self.candidate_level = candidate_level

        # -------------------------------------------------
        # Candidate sequence tracking
        # -------------------------------------------------

        if (
            self.candidate_level
            == self.previous_candidate_level
        ):
            self.candidate_count += 1

        else:
            self.previous_candidate_level = (
                self.candidate_level
            )
            self.candidate_count = 1

        # -------------------------------------------------
        # Same level
        # -------------------------------------------------

        if self.candidate_level == self.risk_level:
            self.candidate_count = 0
            self.previous_candidate_level = (
                self.candidate_level
            )

            return False

        # -------------------------------------------------
        # Upward transition
        # -------------------------------------------------

        if self.candidate_level > self.risk_level:
            if (
                self.candidate_count
                >= self.UP_CONFIRM_COUNT_REQUIRED
            ):
                self.risk_level = self.candidate_level

                # Transition completed.
                # Reset confirmation counter.
                self.candidate_count = 0
                self.previous_candidate_level = (
                    self.risk_level
                )

                return True

            return False

        # -------------------------------------------------
        # Downward transition
        # -------------------------------------------------

        if self.candidate_level < self.risk_level:
            if (
                self.candidate_count
                >= self.DOWN_CONFIRM_COUNT_REQUIRED
            ):
                # Decrease only one Risk Level at a time.
                self.risk_level -= 1

                # Restart confirmation after every
                # downward transition.
                self.candidate_count = 0
                self.previous_candidate_level = (
                    self.candidate_level
                )

                return True

            return False

        return False

    def _reset_candidate_state(self):
        """
        Reset temporary Risk Level confirmation state.
        """

        self.candidate_level = (
            self.RISK_LEVEL_NORMAL
        )

        self.previous_candidate_level = (
            self.RISK_LEVEL_NORMAL
        )

        self.candidate_count = 0

    @staticmethod
    def _count_true_conditions(
        conditions,
    ):
        """
        Count how many conditions are True.

        Exactly three conditions are required.
        """

        if len(conditions) != 3:
            raise ValueError(
                "Exactly three conditions are required: "
                "distance, TTC, pedal."
            )

        return sum(
            1
            for condition in conditions
            if bool(condition)
        )

    def _is_valid_risk_level(
        self,
        risk_level,
    ):
        """
        Check whether a Risk Level value is valid.
        """

        return risk_level in (
            self.RISK_LEVEL_NORMAL,
            self.RISK_LEVEL_CAUTION,
            self.RISK_LEVEL_WARNING,
            self.RISK_LEVEL_DANGER,
        )
