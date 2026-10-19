class FailSafeManager:
    MODE_HARDWARE = "HARDWARE"
    MODE_CARLA = "CARLA"

    def __init__(self):
        self.active = False
        self.reason = None

    def evaluate(self, state, input_mode="HARDWARE"):

        # ToF hardware is retired; only CARLA requires an environment link.
        if input_mode == self.MODE_CARLA:
            if not state.carla_connected:
                self.active = True
                self.reason = "CARLA_COMMUNICATION_ERROR"
                return self.active

        elif input_mode != self.MODE_HARDWARE:
            self.active = True
            self.reason = "INVALID_INPUT_MODE"
            return self.active

        # Pedal communication failure
        if not state.pedal_connected:
            self.active = True
            self.reason = "PEDAL_COMMUNICATION_ERROR"
            return self.active

        # Pedal sensor error
        if (
            state.pedal_connected
            and state.pedal_sensor_status is not None
            and state.pedal_sensor_status != 0x00
        ):
            self.active = True
            self.reason = "PEDAL_SENSOR_ERROR"
            return self.active

        # No fault
        self.active = False
        self.reason = None

        return self.active