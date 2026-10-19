"""
HMI 데이터 소스 (백그라운드 스레드).

- CanSniffer   : can0 를 수신 전용으로 열어 0x100/0x110/0x120/0x200/0x210 감시
                 (SocketCAN 은 여러 프로세스가 동시에 같은 인터페이스를 열 수 있어
                  Safety ECU 와 충돌 없음. HMI 는 송신하지 않음)
- UdpReceiver  : Safety ECU 의 hmi_publisher 가 보내는 JSON 상태 수신
- DemoSource   : 하드웨어 없이 UI 를 확인하기 위한 가상 시나리오

모든 소스는 sink(kind, payload, timestamp) 콜백 하나로만 데이터를 넘김.
sink 는 thread-safe queue 에 넣기만 하므로 Qt 객체를 다른 스레드에서 건드리지 않음.
"""

import json
import socket
import threading
import time

from hmi import config


class _Worker(threading.Thread):
    def __init__(self, sink, name):
        super().__init__(name=name, daemon=True)
        self.sink = sink
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    @property
    def stopped(self):
        return self._stop_event.is_set()


class CanSniffer(_Worker):
    RETRY_INTERVAL = 1.0

    def __init__(self, sink, channel=config.CAN_CHANNEL, interface=config.CAN_INTERFACE):
        super().__init__(sink, "can-sniffer")
        self.channel = channel
        self.interface = interface

    def _status(self, ok, text):
        self.sink("can_status", {"ok": ok, "text": text, "channel": self.channel}, time.monotonic())

    def run(self):
        try:
            import can  # python-can
        except ImportError:
            self._status(False, "python-can 미설치")
            return

        ids = [
            config.PEDAL_STATUS_ID,
            config.HAPTIC_COMMAND_ID,
            config.MOTOR_COMMAND_ID,
            config.HAPTIC_STATUS_ID,
            config.MOTOR_STATUS_ID,
        ]
        filters = [{"can_id": i, "can_mask": 0x7FF, "extended": False} for i in ids]

        last_error = None
        while not self.stopped:
            bus = None
            try:
                bus = can.Bus(
                    interface=self.interface,
                    channel=self.channel,
                    can_filters=filters,
                    receive_own_messages=False,
                )
                self._status(True, "{} 수신 중".format(self.channel))
                last_error = None

                while not self.stopped:
                    msg = bus.recv(timeout=0.2)
                    if msg is None:
                        continue
                    if msg.is_error_frame or msg.is_remote_frame or msg.is_extended_id:
                        continue
                    self.sink(
                        "can",
                        (msg.arbitration_id, bytes(msg.data)),
                        time.monotonic(),
                    )

            except Exception as error:  # 인터페이스 down, 케이블 분리 등
                text = "{} 열기 실패: {}".format(self.channel, error)
                if text != last_error:
                    self._status(False, text)
                    last_error = text
                self._stop_event.wait(self.RETRY_INTERVAL)

            finally:
                if bus is not None:
                    try:
                        bus.shutdown()
                    except Exception:
                        pass


class UdpReceiver(_Worker):
    """JSON UDP 수신. carla의 scene/map/ui_state는 backend가 type별로 분리한다.

    수신기는 패킷을 합치지 않고 그대로 전달한다. udp는 Safety ECU 상태이다.
    """

    def __init__(self, sink, host=config.UDP_BIND_HOST, port=config.UDP_PORT, kind="udp"):
        super().__init__(sink, "{}-receiver".format(kind))
        self.host = host
        self.port = port
        self.kind = kind

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((self.host, self.port))
        except OSError as error:
            self.sink(self.kind + "_status", {"ok": False, "text": "UDP {} 바인드 실패: {}".format(self.port, error)}, time.monotonic())
            return

        self.sink(self.kind + "_status", {"ok": True, "text": "UDP {} 대기 중".format(self.port)}, time.monotonic())
        sock.settimeout(0.2)

        while not self.stopped:
            try:
                payload, _addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                continue

            try:
                packet = json.loads(payload.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                continue

            if isinstance(packet, dict):
                self.sink(self.kind, packet, time.monotonic())

        sock.close()


class DemoSource(_Worker):
    """
    실제 ECU 없이 UI 확인용. 실제 경로와 똑같이 CAN 바이트 + UDP dict 를 만들어
    sink 로 넣기 때문에 디코더/병합 로직도 같이 검증됨.
    """

    def __init__(self, sink):
        super().__init__(sink, "demo-source")
        from hmi.demo_scenario import DemoScenario
        self.scenario = DemoScenario()

    def run(self):
        period = 0.01
        next_tick = time.monotonic()
        udp_divider = 0
        self.sink("can_status", {"ok": True, "text": "DEMO (가상 CAN)", "channel": "demo"}, time.monotonic())
        self.sink("udp_status", {"ok": True, "text": "DEMO (가상 Safety ECU)"}, time.monotonic())

        while not self.stopped:
            now = time.monotonic()
            frames, packet = self.scenario.step(period)
            for frame in frames:
                self.sink("can", frame, now)
            udp_divider += 1
            if udp_divider >= 3:
                udp_divider = 0
                self.sink("udp", packet, now)

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                self._stop_event.wait(delay)
            else:
                next_tick = time.monotonic()


class CameraReceiver(_Worker):
    """
    CARLA 추격 카메라 영상 수신 (TCP 서버, 기본 5602).
    PC 의 carla_hmi_bridge.py 가 접속해서 [4B header_len][4B jpeg_len][header JSON][JPEG] 를 계속 보냄.
    최신 프레임만 sink 로 넘김 (밀리면 중간 프레임은 버려짐).
    """

    MAX_JPEG = 8 * 1024 * 1024

    def __init__(self, sink, host=config.UDP_BIND_HOST, port=config.CAMERA_PORT):
        super().__init__(sink, "camera-receiver")
        self.host = host
        self.port = port

    def _recv_exact(self, conn, n):
        buf = bytearray()
        while len(buf) < n and not self.stopped:
            try:
                chunk = conn.recv(min(65536, n - len(buf)))
            except socket.timeout:
                continue
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf) if len(buf) == n else None

    def run(self):
        import struct

        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self.host, self.port))
            srv.listen(1)
        except OSError as error:
            self.sink("camera_status", {"ok": False, "text": "TCP {} 바인드 실패: {}".format(self.port, error)}, time.monotonic())
            return
        srv.settimeout(0.5)
        self.sink("camera_status", {"ok": True, "text": "TCP {} 대기 중".format(self.port)}, time.monotonic())

        while not self.stopped:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                continue
            conn.settimeout(0.5)
            self.sink("camera_status", {"ok": True, "text": "{} 에서 영상 수신".format(addr[0])}, time.monotonic())
            try:
                while not self.stopped:
                    head = self._recv_exact(conn, 8)
                    if head is None:
                        break
                    hlen, jlen = struct.unpack(">II", head)
                    if hlen > 1024 * 1024 or jlen > self.MAX_JPEG:
                        break
                    hb = self._recv_exact(conn, hlen)
                    jpeg = self._recv_exact(conn, jlen)
                    if hb is None or jpeg is None:
                        break
                    try:
                        header = json.loads(hb.decode("utf-8"))
                    except ValueError:
                        continue
                    self.sink("camera", (header, jpeg), time.monotonic())
            finally:
                conn.close()
                self.sink("camera_status", {"ok": True, "text": "TCP {} 대기 중 (연결 끊김)".format(self.port)}, time.monotonic())
        srv.close()
