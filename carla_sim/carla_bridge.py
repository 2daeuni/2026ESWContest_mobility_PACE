#!/usr/bin/env python3
"""Receive Pedal CAN and G29 input; send CARLA controls at 50 Hz.

Run with /home/pi/PACE/.venv/bin/python. This program never sends CAN
frames or configures interfaces. Its control UDP socket is send-only; port 5006
is reserved for the Safety ECU. G29 loss selects P and neutral steering.
"""

import select
import socket
import struct
import time

from evdev import InputDevice, ecodes, list_devices

try:
    from .g29_ffb import LaneReceiver, NativeAutoCenter
except ImportError:
    from g29_ffb import LaneReceiver, NativeAutoCenter


CAN_CHANNEL = "can0"  # Must already be configured for 500000 bit/s.
PEDAL_ID = 0x100
PEDAL_TIMEOUT = 0.20
UDP_DESTINATION = ("192.168.0.57", 5005)
SEND_PERIOD = 1.0 / 50
PRINT_PERIOD = 1.0 / 4
RETRY_PERIOD = 1.0
STEERING_DEADZONE = 0.01
DRIVE_KEY = 302
REVERSE_KEY = 303
CAN_FRAME = struct.Struct("=IB3x8s")
CAN_SFF_MASK = 0x7FF
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000


def normalize_steering(value, minimum, maximum):
    """Map the device's full ABS_X range to [-1, +1]."""
    if maximum <= minimum:
        raise ValueError("Invalid G29 steering range")
    normalized = max(-1.0, min(1.0, 2.0 * (value - minimum) / (maximum - minimum) - 1.0))
    return 0.0 if abs(normalized) <= STEERING_DEADZONE else normalized


def open_can():
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        # Use the same ID mask as candump can0,100:7FF. Validate frame
        # flags in Pedal.receive rather than including them in this filter.
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                       struct.pack("=II", PEDAL_ID, CAN_SFF_MASK))
        bus.bind((CAN_CHANNEL,))
        bus.setblocking(False)
        return bus
    except Exception:
        bus.close()
        raise


class G29:
    def __init__(self, device):
        self.device = device
        self.dropped = False
        self.refresh()

    def refresh(self):
        self.refresh_steering()
        self.keys = set(self.device.active_keys())

    def refresh_steering(self):
        # Query the kernel's current value, never initialize raw to min/zero.
        # Polling also catches state changes whose events were not observed.
        info = self.device.absinfo(ecodes.ABS_X)
        if info is None:
            raise ValueError("Missing steering axis")
        self.minimum, self.maximum = info.min, info.max
        self.raw = info.value
        self.steer = normalize_steering(self.raw, self.minimum, self.maximum)

    def print_diagnostics(self):
        print(f"[G29]\ndevice={self.device.path}\nname={self.device.name}\n"
              f"ABS_X min={self.minimum}\nABS_X max={self.maximum}\n"
              f"ABS_X current={self.raw}", flush=True)

    @property
    def gear(self):
        drive = DRIVE_KEY in self.keys
        reverse = REVERSE_KEY in self.keys
        # Ambiguous simultaneous selections also select Park.
        return "D" if drive and not reverse else "R" if reverse and not drive else "P"

    def read(self):
        try:
            events = self.device.read()
            for event in events:
                if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_DROPPED:
                    self.dropped = True
                    self.refresh_steering()
                    self.keys.clear()
                elif self.dropped:
                    if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_REPORT:
                        self.refresh()
                        self.dropped = False
                elif event.type == ecodes.EV_ABS and event.code == ecodes.ABS_X:
                    self.raw = event.value
                    self.steer = normalize_steering(self.raw, self.minimum, self.maximum)
                elif event.type == ecodes.EV_KEY and event.code in (DRIVE_KEY, REVERSE_KEY):
                    if event.value == 1:
                        self.keys.add(event.code)
                    elif event.value == 0:
                        self.keys.discard(event.code)
        except BlockingIOError:
            pass


def find_g29():
    for path in sorted(list_devices()):
        device = None
        try:
            device = InputDevice(path)
            name = (device.name or "").lower()
            if device.info.vendor != 0x046D or not (
                device.info.product == 0xC24F or "g29" in name
            ):
                device.close()
                continue
            capabilities = device.capabilities(absinfo=False)
            if ecodes.ABS_X not in capabilities.get(ecodes.EV_ABS, []):
                device.close()
                continue
            return G29(device)
        except (OSError, ValueError):
            if device is not None:
                device.close()
    return None


class Pedal:
    """Track valid 0x100 reception without sending any CAN frames."""

    def __init__(self):
        self.started = time.monotonic()
        self.last_rx = None
        self.accelerator, self.brake = 0, 100
        self.sensor_status = None
        self.alive = None
        self.previous_state = None

    def receive(self, frame, now):
        if len(frame) != CAN_FRAME.size:
            return
        # Linux classic can_frame: native-endian uint32 ID, uint8 DLC,
        # three padding bytes, eight data bytes (16 bytes total).
        can_id, dlc, data = CAN_FRAME.unpack(frame)
        flags = can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)
        arbitration_id = can_id & CAN_SFF_MASK
        if flags or arbitration_id != PEDAL_ID or dlc != 8:
            return
        acc, brk, acc_rate, brk_rate, status, alive = struct.unpack("<BBhhBB", data)
        if acc > 100 or brk > 100:
            return
        self.accelerator, self.brake = acc, brk
        self.sensor_status, self.alive = status, alive
        self.last_rx = now

    def disconnect(self):
        self.last_rx = None

    def state(self, now):
        if self.last_rx is None:
            return "WAITING" if now - self.started < PEDAL_TIMEOUT else "TIMEOUT"
        if now - self.last_rx >= PEDAL_TIMEOUT:
            return "TIMEOUT"
        if self.sensor_status != 0x00:
            return f"SENSOR ERROR status=0x{self.sensor_status:02X}"
        return "OK"

    def controls(self, now):
        state = self.state(now)
        if state != self.previous_state:
            print(f"[PEDAL] {state}", flush=True)
            self.previous_state = state
        return (self.accelerator, self.brake) if state == "OK" else (0, 100)


def main():
    bus = None
    wheel = None
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)
    lane_receiver = LaneReceiver()
    ffb = None
    next_ffb_retry = 0.0
    pedal = Pedal()
    pedal.controls(time.monotonic())
    g29_diagnostics_printed = False
    next_can_retry = next_wheel_retry = 0.0
    next_send = next_print = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            if lane_receiver.socket is None and now >= next_ffb_retry:
                next_ffb_retry = now + RETRY_PERIOD
                try:
                    lane_receiver.open()
                except OSError as exc:
                    print(f"[FFB] receiver unavailable: {exc}", flush=True)
            lane_receiver.poll(now)
            if bus is None and now >= next_can_retry:
                next_can_retry = now + RETRY_PERIOD
                try:
                    bus = open_can()
                except OSError:
                    pass
            if wheel is None and now >= next_wheel_retry:
                next_wheel_retry = now + RETRY_PERIOD
                try:
                    wheel = find_g29()
                    if wheel is not None:
                        ffb = NativeAutoCenter(wheel.device)
                    if wheel is not None and not g29_diagnostics_printed:
                        wheel.print_diagnostics()
                        g29_diagnostics_printed = True
                except OSError:
                    pass

            readers = ([bus] if bus is not None else [])
            if wheel is not None:
                readers.append(wheel.device)
            ready, _, _ = select.select(readers, [], [], max(0.0, next_send - time.monotonic()))

            if bus is not None and bus in ready:
                try:
                    # Bound draining so an overloaded bus cannot starve UDP.
                    for _ in range(256):
                        try:
                            frame = bus.recv(CAN_FRAME.size)
                        except BlockingIOError:
                            break
                        pedal.receive(frame, time.monotonic())
                except OSError:
                    bus.close()
                    bus = None
                    pedal.disconnect()

            if wheel is not None and wheel.device in ready:
                try:
                    wheel.read()
                except (OSError, ValueError):
                    if ffb is not None:
                        ffb.close()
                        ffb = None
                    wheel.device.close()
                    wheel = None

            now = time.monotonic()
            if now >= next_send:
                if wheel is not None:
                    try:
                        wheel.refresh_steering()
                    except (OSError, ValueError):
                        if ffb is not None:
                            ffb.close()
                            ffb = None
                        wheel.device.close()
                        wheel = None
                acc, brk = pedal.controls(time.monotonic())
                steer, gear = (wheel.steer, wheel.gear) if wheel is not None else (0.0, "P")
                packet = f"{acc},{brk},{steer:.4f},{gear}".encode("ascii")
                try:
                    udp.sendto(packet, UDP_DESTINATION)
                except OSError:
                    # Network loss must not stop input processing or retries.
                    pass
                if now >= next_print:
                    print(f"ACC {acc}%  BRK {brk}%  STEER {steer:+.3f}  GEAR {gear}", flush=True)
                    next_print = now + PRINT_PERIOD
                next_send += SEND_PERIOD
                if next_send <= now:
                    next_send = now + SEND_PERIOD
    except KeyboardInterrupt:
        pass
    finally:
        if ffb is not None:
            ffb.close()
        lane_receiver.close()
        if bus is not None:
            bus.close()
        if wheel is not None:
            wheel.device.close()
        udp.close()


if __name__ == "__main__":
    main()
