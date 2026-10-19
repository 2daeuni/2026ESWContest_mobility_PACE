class CommandMapper:
    def map(
        self,
        risk_level,
        risk_available,
        failsafe_active=False,
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
                    "control_mode": 0,
                    "output_limit": 100,
                    "command_flags": 1,
                },
            }

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
                    "vibration_command": 1,
                    "intensity": 30,
                    "frequency": 20,
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
                    "vibration_command": 1,
                    "intensity": 45,
                    "frequency": 20,
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
                    "vibration_command": 1,
                    "intensity": 65,
                    "frequency": 30,
                },
                "motor": {
                    "control_mode": 2,
                    "output_limit": 0,
                    "command_flags": 1,
                },
            }

        raise ValueError(
            f"Invalid risk level: {risk_level}"
        )