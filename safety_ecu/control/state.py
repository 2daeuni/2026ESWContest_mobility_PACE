import time


class SystemState:
    def __init__(self):
        # -------------------------------------------------
        # Pedal
        # -------------------------------------------------
        self.accelerator = 0
        self.brake = 0

        self.accelerator_rate = 0.0
        self.brake_rate = 0.0

        self.pedal_sensor_status = None
        self.pedal_alive_counter = None

        self.pedal_connected = False
        self.pedal_ever_connected = False

        # -------------------------------------------------
        # Environment / ToF
        # -------------------------------------------------
        self.distance_cm = None

        # ToF communication state
        self.tof_connected = False

        # Distance data validity
        self.distance_valid = False

        # Time when the latest valid distance
        # measurement was received
        self.last_distance_time = None

        # -------------------------------------------------
        # Vehicle
        # -------------------------------------------------
        self.vehicle_speed = None
        self.ttc = None

        # -------------------------------------------------
        # CARLA simulation
        # -------------------------------------------------
        self.carla_connected = False
        self.carla_link_state = "WAITING"
        self.carla_vehicle_speed = None
        self.carla_distance_cm = None
        self.carla_distance_valid = False
        self.carla_closing_speed_mps = None
        self.carla_ttc = None
        self.carla_obstacle_id = None
        self.carla_obstacle_present = False
        self.carla_gear = None
        self.carla_sequence = None
        self.carla_signed_speed_mps = None
        self.carla_escape_obstacle_id = None
        self.carla_escape_distance_m = None

        # -------------------------------------------------
        # Haptic ECU
        # -------------------------------------------------
        self.haptic_risk_level = 0
        self.haptic_vibration_status = 0
        self.haptic_intensity = 0
        self.haptic_frequency = 0

        self.haptic_fault_code = 0
        self.haptic_command_timeout = 1
        self.haptic_ecu_status = 0
        self.haptic_alive_counter = None

        self.haptic_connected = False
        self.haptic_available = False
        self.last_haptic_time = None

        # -------------------------------------------------
        # Risk
        # -------------------------------------------------
        self.bns = 0.0
        self.pms = 0.0

        self.total_risk = 0.0
        self.risk_level = 0

    def update_pedal(self, pedal):
        self.accelerator = pedal["accelerator"]
        self.brake = pedal["brake"]

        self.accelerator_rate = (
            pedal["accelerator_rate"]
        )

        self.brake_rate = (
            pedal["brake_rate"]
        )

        self.pedal_sensor_status = (
            pedal["sensor_status"]
        )

        self.pedal_alive_counter = (
            pedal["alive_counter"]
        )

        self.pedal_connected = True
        self.pedal_ever_connected = True

    def update_haptic(self, haptic):
        self.haptic_risk_level = (
            haptic["risk_level"]
        )

        self.haptic_vibration_status = (
            haptic["vibration_status"]
        )

        self.haptic_intensity = (
            haptic["intensity"]
        )

        self.haptic_frequency = (
            haptic["frequency"]
        )

        self.haptic_fault_code = (
            haptic["fault_code"]
        )

        self.haptic_command_timeout = (
            haptic["command_timeout"]
        )

        self.haptic_ecu_status = (
            haptic["ecu_status"]
        )

        self.haptic_alive_counter = (
            haptic["alive_counter"]
        )

        self.haptic_connected = True

        self.haptic_available = (
            self.haptic_fault_code == 0
            and self.haptic_command_timeout == 0
            and self.haptic_ecu_status == 1
        )

        self.last_haptic_time = (
            time.monotonic()
        )

    def update_distance(self, distance_cm):
        self.distance_cm = distance_cm

        self.tof_connected = True
        self.distance_valid = True

        self.last_distance_time = (
            time.monotonic()
        )

    def invalidate_distance(self):

        self.distance_valid = False

    def check_distance_freshness(
        self,
        timeout=0.5,
    ):

        if not self.distance_valid:
            return False

        if self.last_distance_time is None:
            self.distance_valid = False
            return False

        elapsed = (
            time.monotonic()
            - self.last_distance_time
        )

        if elapsed >= timeout:
            self.distance_valid = False
            return False

        return True

    def check_haptic_freshness(
        self,
        timeout=0.1,
    ):

        if not self.haptic_connected:
            return False

        if self.last_haptic_time is None:
            self.haptic_connected = False
            self.haptic_available = False
            return False

        elapsed = (
            time.monotonic()
            - self.last_haptic_time
        )

        if elapsed >= timeout:
            self.haptic_connected = False
            self.haptic_available = False
            return False

        return True

    def update_carla(self, receiver):
        self.carla_connected = receiver.ready
        self.carla_link_state = receiver.link_state
        self.carla_gear = receiver.gear
        self.carla_sequence = receiver.sequence
        self.carla_signed_speed_mps = receiver.signed_speed_mps
        self.carla_escape_obstacle_id = receiver.escape_obstacle_id
        self.carla_escape_distance_m = receiver.escape_distance_m

        self.carla_vehicle_speed = (
            receiver.vehicle_speed
        )

        self.carla_distance_cm = (
            receiver.distance_m * 100.0
            if receiver.distance_m is not None
            else None
        )

        self.carla_distance_valid = (
            receiver.ready
            and receiver.distance_m is not None
        )

        self.carla_obstacle_present = bool(receiver.ready and receiver.obstacle_id is not None)
        self.carla_obstacle_id = receiver.obstacle_id if self.carla_obstacle_present else None
        if self.carla_obstacle_present:
            self.carla_closing_speed_mps = receiver.closing_speed_mps
            self.carla_ttc = receiver.ttc
        else:
            # Fresh absence is safe; disconnected absence is unavailable.
            self.carla_distance_cm = None
            self.carla_distance_valid = False
            self.carla_closing_speed_mps = 0.0 if receiver.ready else None
            self.carla_ttc = None

    def invalidate_carla(self, link_state="LOST"):
        self.carla_connected = False
        self.carla_link_state = link_state
        self.carla_vehicle_speed = None
        self.carla_distance_cm = None
        self.carla_distance_valid = False
        self.carla_closing_speed_mps = None
        self.carla_ttc = None
        self.carla_obstacle_id = None
        self.carla_obstacle_present = False
        self.carla_gear = None
        self.carla_sequence = None
        self.carla_signed_speed_mps = None
        self.carla_escape_obstacle_id = None
        self.carla_escape_distance_m = None
