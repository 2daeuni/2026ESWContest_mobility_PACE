import socket
import time
import math


class CarlaReceiver:
    def __init__(
        self,
        host="192.168.50.2",
        port=5005,
        timeout=0.5,
    ):
        self.host = host
        self.port = port
        self.timeout = timeout

        self.sock = None

        self.vehicle_speed = None
        self.last_rx_time = None
        self.connected = False

    def start(self):
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )

        self.sock.bind(
            (self.host, self.port)
        )

        self.sock.setblocking(False)

        print("[CARLA] UDP receiver started")

    def receive(self):
        if self.sock is None:
            return False

        try:
            data, address = self.sock.recvfrom(1024)

        except BlockingIOError:
            return False

        except OSError:
            self.connected = False
            return False

        if address[0] != "192.168.50.1":
            return False

        try:
            message = data.decode("utf-8")

            if not message.startswith("CARLA_SPEED="):
                return False

            speed = float(
                message.split("=", 1)[1]
            )

            if not math.isfinite(speed):
                return False

            if speed < 0:
                return False

        except (UnicodeDecodeError, ValueError):
            return False

        self.vehicle_speed = speed
        self.last_rx_time = time.monotonic()
        self.connected = True

        return True

    def check_freshness(self):
        if self.last_rx_time is None:
            self.connected = False
            self.vehicle_speed = None
            return False

        if (
            time.monotonic() - self.last_rx_time
            >= self.timeout
        ):
            self.connected = False
            self.vehicle_speed = None
            return False

        return True

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

        self.connected = False
        self.vehicle_speed = None
