"""Mirror the final MOTOR_COMMAND to Windows without blocking CAN transmission."""
import json
import socket
import time
import uuid


class CarlaControlSender:
    def __init__(self, host="192.168.0.57", port=5008):
        self.destination = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

        self.sequence = 0
        self.session_id = uuid.uuid4().hex
        self.session_started_at = None
        self.last_send = None
        self.last_error_log = None

    def send(self, motor, risk_level, failsafe, now=None):
        now = time.monotonic() if now is None else now

        if self.session_started_at is None:
            self.session_started_at = now

        if self.last_send is not None and now - self.last_send < 0.03:
            return

        self.last_send = now

        # Safe startup window for sender restart recovery
        startup_safe = now - self.session_started_at < 0.65

        if startup_safe:
            mode = 2
            limit = 0
            packet_failsafe = True
        else:
            mode = motor["control_mode"]
            limit = motor["output_limit"]
            packet_failsafe = bool(failsafe)

        packet = {
            "type": "safety_control",
            "session_id": self.session_id,
            "sequence": self.sequence,
            "mode": mode,
            "limit_percent": limit,
            "risk_level": risk_level,
            "failsafe": packet_failsafe,
        }

        self.sequence += 1

        try:
            self.socket.sendto(
                json.dumps(packet).encode("utf-8"),
                self.destination
            )
        except OSError as error:
            if self.last_error_log is None or now - self.last_error_log >= 0.5:
                print(f"[CARLA SAFETY TX] {error}")
                self.last_error_log = now

    def close(self):
        self.socket.close()
