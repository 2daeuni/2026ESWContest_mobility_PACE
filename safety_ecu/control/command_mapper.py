class CommandMapper:
    ACCEL_HAPTIC_DEADBAND_PERCENT = 3

    def map(
        self,
        risk_level,
        risk_available,
        failsafe_active=False,
        accelerator=None,
        pedal_misuse_active=False,
    ):

        # Fail-safe has the highest control priority.
        if failsafe_active:
            return {
                "haptic": {
                    "vibration_command": 0,
                    "intensity": 0,
                    "frequency": 0,
                },
                "motor": {
                    "control_mode": 2,
                    "output_limit": 0,
                    "command_flags": 0x03,
                },
            }

        if not risk_available:
            return {
                "haptic": {
                    "vibration_command": 0,
                    "intensity": 0,
                    "frequency": 0,
                },
                "motor": {
                    "control_mode": 2,
                    "output_limit": 0,
                    "command_flags": 1,
                },
            }

        haptic_enabled = (
            type(accelerator) in (int, float)
            and self.ACCEL_HAPTIC_DEADBAND_PERCENT < accelerator <= 100
            and pedal_misuse_active
        )

        # Level 0 - Normal
        if risk_level == 0:
            return {
                "haptic": {
                    "vibration_command": 0,
                    "intensity": 0,
                    "frequency": 0,
                },
                "motor": {
                    "control_mode": 0,
                    "output_limit": 100,
                    "command_flags": 1,
                },
            }

        # Level 1 - Caution
        if risk_level == 1:
            return {
                "haptic": {
                    "vibration_command": int(haptic_enabled),
                    "intensity": 30 if haptic_enabled else 0,
                    "frequency": 20 if haptic_enabled else 0,
                },
                "motor": {
                    "control_mode": 0,
                    "output_limit": 100,
                    "command_flags": 1,
                },
            }

        # Level 2 - Warning
        if risk_level == 2:
            return {
                "haptic": {
                    "vibration_command": int(haptic_enabled),
                    "intensity": 45 if haptic_enabled else 0,
                    "frequency": 20 if haptic_enabled else 0,
                },
                "motor": {
                    "control_mode": 1,
                    "output_limit": 30,
                    "command_flags": 1,
                },
            }

        # Level 3 - Danger
        if risk_level == 3:
            return {
                "haptic": {
                    "vibration_command": int(haptic_enabled),
                    "intensity": 65 if haptic_enabled else 0,
                    "frequency": 30 if haptic_enabled else 0,
                },
                "motor": {
                    "control_mode": 2,
                    "output_limit": 0,
                    "command_flags": 0x03,
                },
            }

        raise ValueError(
            f"Invalid risk level: {risk_level}"
        )
