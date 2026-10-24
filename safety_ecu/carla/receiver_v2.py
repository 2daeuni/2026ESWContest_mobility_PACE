import socket
import json
import math
import time

class CarlaReceiverV2:
    READY_CONFIRM_COUNT = 3

    def __init__(
        self,
        host="192.168.50.2",
        port=5006,
        timeout=0.5
    ):
        self.host = host
        self.port = port
        self.timeout = timeout

        self.sock = None
        self.connected = False
        self.ready = False
        self.ever_ready = False
        self.confirm_count = 0
        self.last_rx_time = None

        self.sequence = None
        self.vehicle_speed = None
        self.distance_m = None
        self.closing_speed_mps = None
        self.ttc = None
        self.obstacle_id = None
        self.gear = None
        self.signed_speed_mps = None
        self.escape_obstacle_id = None
        self.escape_distance_m = None

    def start(self):
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        try:
            self.sock.bind((self.host, self.port))
        except OSError:
            self.sock.close()
            self.sock = None
            raise
        self.sock.setblocking(False)

        print("[CARLA V2] UDP receiver started")

    def receive(self):
        if self.sock is None:
            return False

        try:
            data, address = self.sock.recvfrom(2048)
        except BlockingIOError:
            return False
        except OSError:
            self._invalidate()
            return False

        if address[0] != "192.168.50.1":
            return False

        if self.last_rx_time is not None and time.monotonic() - self.last_rx_time >= self.timeout:
            self._invalidate()

        try:
            payload = json.loads(data.decode("utf-8"))

            if not isinstance(payload, dict):
                return self._reject()

            if payload.get("source") != "CARLA":
                return self._reject()

            sequence = payload["sequence"]
            speed = payload["vehicle_speed_kmh"]
            distance = payload["distance_m"]
            closing = payload["closing_speed_mps"]
            ttc = payload["ttc_s"]
            obstacle_id = payload["obstacle_id"]
            gear = payload["gear"]
            signed_speed = payload.get("signed_speed_mps")
            escape_id = payload.get("escape_obstacle_id")
            escape_distance = payload.get("escape_distance_m")

            if gear not in ("D", "R", "P"):
                return self._reject()

            if type(sequence) is not int:
                return self._reject()

            if not 0 <= sequence <= 65535:
                return self._reject()

            def valid_number(value):
                return (
                    type(value) in (int, float)
                    and math.isfinite(value)
                )

            if not valid_number(speed) or speed < 0:
                return self._reject()
            if signed_speed is not None and not valid_number(signed_speed):
                return self._reject()
            if (escape_id is None) != (escape_distance is None):
                return self._reject()
            if escape_id is not None and (
                type(escape_id) is not int or escape_id < 0
                or not valid_number(escape_distance) or escape_distance < 0
            ):
                return self._reject()

            if distance is not None:
                if not valid_number(distance) or distance < 0:
                    return self._reject()

            if closing is not None:
                if not valid_number(closing):
                    return self._reject()

            if ttc is not None:
                if not valid_number(ttc) or ttc < 0:
                    return self._reject()

            if obstacle_id is not None:
                if type(obstacle_id) is not int:
                    return self._reject()

                if obstacle_id < 0:
                    return self._reject()

            if obstacle_id is None:
                if any(
                    value is not None
                    for value in (distance, closing, ttc)
                ):
                    return self._reject()
            else:
                if distance is None or closing is None:
                    return self._reject()

            if ttc is not None and closing <= 0 and not (
                ttc == 0 and distance == 0
            ):
                return self._reject()

            if self.sequence is not None:
                sequence_delta = (
                    sequence - self.sequence
                ) % 65536

                if not 1 <= sequence_delta <= 32767:
                    return self._reject()

        except (
            UnicodeDecodeError,
            json.JSONDecodeError,
            KeyError,
            TypeError,
            ValueError
        ):
            return self._reject()

        self.sequence = sequence
        self.vehicle_speed = speed
        self.distance_m = distance
        self.closing_speed_mps = closing
        self.ttc = ttc
        self.obstacle_id = obstacle_id
        self.gear = gear
        self.signed_speed_mps = signed_speed
        self.escape_obstacle_id = escape_id
        self.escape_distance_m = escape_distance

        self.last_rx_time = time.monotonic()
        self.connected = True
        if not self.ready:
            self.confirm_count += 1
            if self.confirm_count >= self.READY_CONFIRM_COUNT:
                self.ready = True
                self.ever_ready = True

        return True

    @property
    def link_state(self):
        if self.ready:
            return "READY"
        if self.ever_ready:
            return "RECOVERING" if self.connected else "LOST"
        return "WAITING"

    def _reject(self):
        if not self.ready:
            self.confirm_count = 0
        return False

    def check_freshness(self):
        if self.last_rx_time is None:
            self._invalidate()
            return False

        if time.monotonic() - self.last_rx_time >= self.timeout:
            self._invalidate()
            return False

        return True

    def _invalidate(self):
        self.connected = False
        self.ready = False
        self.confirm_count = 0
        self.last_rx_time = None
        self.sequence = None
        self.vehicle_speed = None
        self.distance_m = None
        self.closing_speed_mps = None
        self.ttc = None
        self.obstacle_id = None
        self.gear = None
        self.signed_speed_mps = None
        self.escape_obstacle_id = None
        self.escape_distance_m = None

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

        self._invalidate()
        print("[CARLA V2] UDP receiver closed")
