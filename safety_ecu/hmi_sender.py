"""Best-effort HMI telemetry; never sends CAN or changes control state."""

import json
import queue
import socket
import threading
import time


def build_packet(state, *, seq, input_mode, bns_available, pms_available,
                 risk_available, failsafe_active, failsafe_reason,
                 commands, motor_connected):
    """Snapshot actual state and final commands without modifying either."""
    fields = (
        "pedal_connected", "accelerator", "brake", "accelerator_rate",
        "brake_rate", "pedal_sensor_status", "haptic_connected",
        "tof_connected", "carla_connected", "haptic_fault_code",
        "haptic_available", "bns", "pms", "risk_level", "total_risk",
    )
    packet = {name: getattr(state, name) for name in fields}
    packet.update(
        seq=seq, operating_mode=input_mode,
        score_max=1.0,  # Existing HMI metadata: BNS/PMS/total are normalized, not 0..100.
        motor_connected=motor_connected,
        bns_available=bns_available, pms_available=pms_available,
        risk_available=risk_available, failsafe_active=failsafe_active,
        failsafe_reason=failsafe_reason,
        haptic_vibration_command=commands["haptic"]["vibration_command"],
        haptic_intensity=commands["haptic"]["intensity"],
        haptic_frequency=commands["haptic"]["frequency"],
        motor_control_mode=commands["motor"]["control_mode"],
        motor_output_limit=commands["motor"]["output_limit"],
    )
    if input_mode == "CARLA":
        packet.update(
            distance_valid=state.carla_distance_valid,
            distance_cm=state.carla_distance_cm,
            vehicle_speed=state.carla_vehicle_speed,
            vehicle_speed_unit="km/h", speed_source="CARLA",
            ttc=state.carla_ttc,
            # HMI stores relative_speed as km/h; source is closing m/s.
            relative_speed=(state.carla_closing_speed_mps * 3.6
                            if state.carla_closing_speed_mps is not None else None),
        )
    else:
        # Hardware vehicle_speed is currently unset; do not invent a source/unit.
        packet.update(distance_valid=state.distance_valid,
                      distance_cm=state.distance_cm,
                      vehicle_speed=state.vehicle_speed, ttc=state.ttc)
    return packet


class HmiSender:
    """10 Hz snapshots, one pending packet, nonblocking UDP in a worker."""

    def __init__(self):
        self.target = ("127.0.0.1", 5600)
        self._queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="hmi-sender", daemon=True)
        self._next_publish = 0.0
        self._seq = 0

    def start(self):
        self._thread.start()

    def publish(self, state, **values):
        now = time.monotonic()
        if now < self._next_publish:
            return
        self._next_publish = now + 0.1
        packet = build_packet(state, seq=self._seq, **values)
        self._seq += 1
        try:
            self._queue.put_nowait(packet)
        except queue.Full:
            pass  # Telemetry may be dropped; control must never wait.

    def _run(self):
        sock = None
        try:
            while not self._stop.is_set():
                try:
                    packet = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    if sock is None:
                        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        sock.setblocking(False)
                    payload = json.dumps(packet, separators=(",", ":"), allow_nan=False)
                    sock.sendto(payload.encode("utf-8"), self.target)
                except (OSError, TypeError, ValueError):
                    # Includes unavailable network, full send buffer, invalid JSON values.
                    # Drop this packet and retry on a later snapshot, without log flooding.
                    if sock is not None:
                        sock.close()
                        sock = None
        finally:
            if sock is not None:
                sock.close()

    def close(self):
        self._stop.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=0.3)
