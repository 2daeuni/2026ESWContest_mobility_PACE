import socket
import json
import math
import time

class CarlaReceiverV2:
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
        self.last_rx_time = None

        self.sequence = None
        self.vehicle_speed = None
        self.distance_m = None
        self.closing_speed_mps = None
        self.ttc = None
        self.obstacle_id = None

    def start(self):
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.sock.bind((self.host, self.port))
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
            self.connected = False
            return False

        if address[0] != "192.168.0.57":
            return False

        try:
            payload = json.loads(data.decode("utf-8"))

            if not isinstance(payload, dict):
                return False

            if payload.get("source") != "CARLA":
                return False

            sequence = payload["sequence"]
            speed = payload["vehicle_speed_kmh"]
            distance = payload["distance_m"]
            closing = payload["closing_speed_mps"]
            ttc = payload["ttc_s"]
            obstacle_id = payload["obstacle_id"]

            if type(sequence) is not int:
                return False

            if not 0 <= sequence <= 65535:
                return False

            def valid_number(value):
                return (
                    type(value) in (int, float)
                    and math.isfinite(value)
                )

            if not valid_number(speed) or speed < 0:
                return False

            if distance is not None:
                if not valid_number(distance) or distance < 0:
                    return False

            if closing is not None:
                if not valid_number(closing):
                    return False

            if ttc is not None:
                if not valid_number(ttc) or ttc < 0:
                    return False

            if obstacle_id is not None:
                if type(obstacle_id) is not int:
                    return False

                if obstacle_id < 0:
                    return False

            if obstacle_id is None:
                if any(
                    value is not None
                    for value in (distance, closing, ttc)
                ):
                    return False
            else:
                if distance is None or closing is None:
                    return False

            if ttc is not None and closing <= 0:
                return False

            if self.sequence is not None:
                sequence_delta = (
                    sequence - self.sequence
                ) % 65536

                if not 1 <= sequence_delta <= 32767:
                    return False

        except (
            UnicodeDecodeError,
            json.JSONDecodeError,
            KeyError,
            TypeError,
            ValueError
        ):
            return False

        self.sequence = sequence
        self.vehicle_speed = speed
        self.distance_m = distance
        self.closing_speed_mps = closing
        self.ttc = ttc
        self.obstacle_id = obstacle_id

        self.last_rx_time = time.monotonic()
        self.connected = True

        return True

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
        self.last_rx_time = None
        self.sequence = None
        self.vehicle_speed = None
        self.distance_m = None
        self.closing_speed_mps = None
        self.ttc = None
        self.obstacle_id = None

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

        self._invalidate()
        print("[CARLA V2] UDP receiver closed")