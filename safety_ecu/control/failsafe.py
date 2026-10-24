class FailSafeManager:
    MODE_HARDWARE = "HARDWARE"
    MODE_CARLA = "CARLA"

    def __init__(self):
        self.active = False
        self.reason = None
        self.waiting_reason = None

    def evaluate(self, state, input_mode="HARDWARE"):
        self.active = False
        self.reason = None
        self.waiting_reason = None

        if input_mode not in (self.MODE_CARLA, self.MODE_HARDWARE):
            self.active = True
            self.reason = "INVALID_INPUT_MODE"
            return self.active

        if state.pedal_connected and state.pedal_sensor_status not in (None, 0x00):
            self.active = True
            self.reason = "PEDAL_SENSOR_ERROR"
            return self.active

        if input_mode == self.MODE_CARLA and not state.carla_connected and state.carla_link_state in ("LOST", "RECOVERING"):
            self.active = True
            self.reason = "CARLA_COMMUNICATION_ERROR"
            return self.active

        if not state.pedal_connected and state.pedal_ever_connected:
            self.active = True
            self.reason = "PEDAL_COMMUNICATION_ERROR"
            return self.active

        if input_mode == self.MODE_CARLA and not state.carla_connected:
            self.waiting_reason = (
                "CARLA_RECOVERING" if state.carla_link_state == "RECOVERING"
                else "CARLA_WAITING"
            )
        elif not state.pedal_connected:
            self.waiting_reason = "PEDAL_WAITING"

        return self.active
