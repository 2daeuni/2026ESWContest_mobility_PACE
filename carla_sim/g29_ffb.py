"""Optional G29 output. Importing this module never opens or drives hardware."""
import json
import math
import socket

from evdev import ecodes, ff

G29_BASE_AUTOCENTER_PERCENT = 35
LANE_FFB_TIMEOUT_SEC = 0.20
ENABLE_LANE_FFB = False  # G29 follows position only; CARLA vehicle LFA is independent.
LANE_FFB_MAX_PERCENT = 15
LANE_FFB_DIRECTION_SIGN = 1.0
FFB_MAX_STEP_PERCENT = 2
CENTER_FFB_MAX_PERCENT = 35
CENTER_FFB_MIN_PERCENT = 6
CENTER_FFB_DEADZONE_NORM = 0.005
CENTER_FFB_MAX_STEP_PERCENT = 2
CENTER_FFB_DIRECTION_SIGN = 1.0
TOTAL_FFB_MAX_PERCENT = 50
# Global bound from carla_pedal_control_v17_safety_json_5006.py:
# LFA_MAX_CORR_LOW/MID/HIGH = 0.16/0.13/0.10.
# Override, engagement, filtering and rate limiting cannot increase this bound.
LFA_MAX_CORRECTION = 0.16
LANE_FFB_ADDRESS = ("192.168.50.2", 5007)


def diagnose_capabilities(device):
    """Read only: no upload, playback, gain or autocenter writes."""
    codes = device.capabilities(absinfo=False).get(ecodes.EV_FF, [])
    return {name: getattr(ecodes, name) in codes for name in
            ("FF_AUTOCENTER", "FF_CONSTANT", "FF_SPRING", "FF_GAIN")}


def normalize_center_steering(value, minimum, maximum):
    """Same full-range mapping as bridge input, without its 0.01 input deadzone.

    This value is used ONLY for output force; UDP steering is unchanged.
    """
    if maximum <= minimum:
        raise ValueError("Invalid G29 steering range")
    value = 2.0 * (value - minimum) / (maximum - minimum) - 1.0
    if not math.isfinite(value):
        raise ValueError("Non-finite steering")
    return max(-1.0, min(1.0, value))


def center_force_percent(steer_norm):
    if not math.isfinite(steer_norm):
        raise ValueError("Non-finite steering")
    steer_norm = max(-1.0, min(1.0, steer_norm))
    if abs(steer_norm) <= CENTER_FFB_DEADZONE_NORM:
        return 0.0
    magnitude = (abs(steer_norm) - CENTER_FFB_DEADZONE_NORM) / (1.0 - CENTER_FFB_DEADZONE_NORM)
    percent = CENTER_FFB_MIN_PERCENT + magnitude * (CENTER_FFB_MAX_PERCENT - CENTER_FFB_MIN_PERCENT)
    return -math.copysign(percent, steer_norm) * CENTER_FFB_DIRECTION_SIGN


def clamp_total_force(level):
    limit = int(32767 * TOTAL_FFB_MAX_PERCENT / 100)
    return max(-limit, min(limit, level))


def slew(current, target, percent):
    step = int(32767 * percent / 100)
    return current + max(-step, min(step, target - current))


class LaneReceiver:
    def __init__(self):
        self.socket = None
        self.packet = None
        self.last_rx = None
        self.sequence = None

    def open(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setblocking(False)
            sock.bind(LANE_FFB_ADDRESS)
        except Exception:
            sock.close()
            raise
        self.socket = sock

    def invalidate(self):
        self.packet = None
        self.last_rx = None

    def accept(self, data, now):
        try:
            p = json.loads(data)
            if not isinstance(p, dict) or p.get("type") != "lane_ffb":
                raise ValueError("packet type")
            for key in ("lane_assist_enabled", "lane_assist_active"):
                if type(p[key]) is not bool:
                    raise ValueError(key)
            seq = p["sequence"]
            corr = p["lane_assist_correction"]
            if type(seq) is not int or seq < 0:
                raise ValueError("sequence")
            if type(corr) not in (int, float) or not math.isfinite(corr):
                raise ValueError("correction")
            if not isinstance(p["lane_assist_state"], str):
                raise ValueError("state")
            # Ignore duplicate/reordered packets; allow sender restart after timeout.
            if (self.sequence is not None and seq <= self.sequence and
                    self.last_rx is not None and now - self.last_rx < LANE_FFB_TIMEOUT_SEC):
                return
            self.packet, self.last_rx, self.sequence = p, now, seq
        except (ValueError, TypeError, KeyError, UnicodeError, OverflowError, RecursionError):
            self.invalidate()

    def poll(self, now):
        if self.socket is None:
            return
        try:
            for _ in range(64):
                try:
                    data, _ = self.socket.recvfrom(4096)
                except BlockingIOError:
                    break
                self.accept(data, now)
        except OSError:
            self.invalidate()
            self.socket.close()
            self.socket = None

    def close(self):
        if self.socket is not None:
            self.socket.close()


class NativeAutoCenter:
    """Driving output: native centering only; never uploads constant effects."""

    def __init__(self, device):
        self.device = device
        self.supported = False
        try:
            self.supported = diagnose_capabilities(device)["FF_AUTOCENTER"]
            if self.supported:
                device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER,
                             int(65535 * G29_BASE_AUTOCENTER_PERCENT / 100))
                print("[FFB] native Auto Center=35% software=OFF lane=OFF", flush=True)
            else:
                print("[FFB] native Auto Center unsupported", flush=True)
        except Exception as exc:
            self.close()
            print(f"[FFB] native Auto Center unavailable: {exc}", flush=True)

    def close(self):
        if self.supported:
            try:
                self.device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
            except Exception:
                pass  # Disconnected devices may reject the reset.


class WheelFFB:
    def __init__(self, device, correction_limit=LFA_MAX_CORRECTION):
        self.device = device
        self.limit = correction_limit
        self.effect_id = -1
        self.applied = self.target = 0
        self.center = 0
        self.center_target = self.center_applied = 0
        self.lane_target = self.lane_applied = 0
        self.failed = False
        self.next_log = 0
        self.capabilities = {}
        try:
            self.capabilities = diagnose_capabilities(device)
            print(f"[FFB capability] {self.capabilities}", flush=True)
            if self.capabilities["FF_AUTOCENTER"]:
                device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
            # Leave global FF_GAIN untouched; do not amplify other effects.
        except Exception as exc:
            self.fail(exc)

    def stop(self):
        if self.effect_id >= 0:
            self.device.write(ecodes.EV_FF, self.effect_id, 0)
        self.applied = self.target = 0
        self.center_target = self.center_applied = 0
        self.lane_target = self.lane_applied = 0

    def fail(self, exc):
        print(f"[FFB] disabled: {exc}", flush=True)
        self.failed = True
        try:
            self.stop()
        except Exception:
            pass
        self.applied = self.target = 0
        self.center_target = self.center_applied = 0
        self.lane_target = self.lane_applied = 0

    def update(self, receiver, now, steer_norm=0.0):
        p = receiver.packet or {}
        fresh = receiver.last_rx is not None and now - receiver.last_rx < LANE_FFB_TIMEOUT_SEC
        enabled = p.get("lane_assist_enabled", False)
        active = (ENABLE_LANE_FFB and fresh and enabled and p.get("lane_assist_active", False) and
                  p.get("lane_assist_state") == "ACTIVE")
        try:
            if not self.failed:
                if not self.capabilities.get("FF_CONSTANT"):
                    self.stop()
                else:
                    self.center_target = int(32767 * center_force_percent(steer_norm) / 100)
                    self.center_applied = slew(self.center_applied, self.center_target,
                                               CENTER_FFB_MAX_STEP_PERCENT)
                    if active and self.limit:
                        normalized = max(-1., min(1., p["lane_assist_correction"] / self.limit))
                        self.lane_target = int(normalized * 32767 * LANE_FFB_MAX_PERCENT / 100 * LANE_FFB_DIRECTION_SIGN)
                        self.lane_applied = slew(self.lane_applied, self.lane_target, FFB_MAX_STEP_PERCENT)
                    else:
                        # Remove lane contribution immediately, retaining the spring.
                        self.lane_target = self.lane_applied = 0
                    # Default driving policy: UDP5007 cannot affect wheel force.
                    # Keep the optional lane implementation isolated behind its flag.
                    if ENABLE_LANE_FFB:
                        self.target = clamp_total_force(self.center_target + self.lane_target)
                        level = clamp_total_force(self.center_applied + self.lane_applied)
                    else:
                        self.target = self.center_target
                        level = self.center_applied
                    if level == 0:
                        if self.effect_id >= 0:
                            self.device.write(ecodes.EV_FF, self.effect_id, 0)
                        self.applied = 0
                    else:
                        # One combined effect; finite replay bounds force after a stalled loop.
                        effect = ff.Effect(ecodes.FF_CONSTANT, self.effect_id, 0xC000,
                                           ff.Trigger(0, 0), ff.Replay(100, 0),
                                           ff.EffectType(ff_constant_effect=ff.Constant(level, ff.Envelope(0, 0, 0, 0))))
                        self.effect_id = self.device.upload_effect(effect)
                        self.device.write(ecodes.EV_FF, self.effect_id, 1)
                        self.applied = level
        except Exception as exc:
            self.fail(exc)
        if now >= self.next_log:
            print(f"[FFB] NATIVE_CENTER=0% CENTER={100*self.center_applied/32767:+.1f}% "
                  f"LANE={100*self.lane_applied/32767:+.1f}% J={'ON' if enabled else 'OFF'} "
                  f"STATE={p.get('lane_assist_state', 'OFF')} CORR={p.get('lane_assist_correction', 0):+.4f} "
                  f"TARGET={100*self.target/32767:+.1f}% APPLIED={100*self.applied/32767:+.1f}% FRESH={int(fresh)}", flush=True)
            self.next_log = now + .5

    def close(self):
        # Attempt each cleanup even when another operation fails/disconnects.
        for action in (self.stop,
                       lambda: self.device.erase_effect(self.effect_id) if self.effect_id >= 0 else None,
                       lambda: self.device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
                       if self.capabilities.get("FF_AUTOCENTER") else None):
            try:
                action()
            except Exception:
                pass


def print_device_diagnostics(device):
    """Capability queries only; safe before either hardware test."""
    caps = device.capabilities(absinfo=False)
    print(f"[G29 device] path={device.path} name={device.name}", flush=True)
    print(f"vendor=0x{device.info.vendor:04X} product=0x{device.info.product:04X}", flush=True)
    print(f"ABS_X={device.absinfo(ecodes.ABS_X)}", flush=True)
    print(f"EV_FF={ecodes.EV_FF in caps}", flush=True)
    support = diagnose_capabilities(device)
    for name, supported in support.items():
        print(f"{name}={supported}", flush=True)
    return support


def cleanup_test(device, effect_id, reset_center):
    """Attempt every cleanup operation, including after partial initialization."""
    actions = []
    if effect_id >= 0:
        actions.extend([
            ("constant stop", lambda: device.write(ecodes.EV_FF, effect_id, 0)),
            ("constant erase", lambda: device.erase_effect(effect_id)),
        ])
    if reset_center:
        actions.append(("autocenter reset", lambda: device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)))
    for name, action in actions:
        try:
            action()
        except OSError as exc:
            print(f"[FFB cleanup] {name} failed: {exc}", flush=True)


def test_center(device):
    """Only AUTOCENTER: no constant-effect upload or playback."""
    import time
    try:
        print("[FFB test] Auto Center 35%, 10 seconds; lane constant effect is not used", flush=True)
        device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER,
                     int(65535 * G29_BASE_AUTOCENTER_PERCENT / 100))
        time.sleep(10)
    finally:
        cleanup_test(device, -1, True)


def test_software_center(wheel):
    """Ten-second, 50 Hz center-only test; no socket and no lane packets."""
    import time
    print("[CENTER TEST] native Auto Center OFF; lane OFF; 10 seconds", flush=True)
    controller = WheelFFB(wheel.device)
    receiver = LaneReceiver()  # No open(): never binds or receives UDP.
    try:
        if controller.failed or not controller.capabilities.get("FF_CONSTANT"):
            raise OSError("Software center unavailable")
        end = time.monotonic() + 10.0
        next_log = 0.0
        while time.monotonic() < end:
            started = time.monotonic()
            wheel.refresh_steering()
            steer = normalize_center_steering(wheel.raw, wheel.minimum, wheel.maximum)
            controller.next_log = float('inf')
            controller.update(receiver, started, steer)
            if controller.failed:
                raise OSError("Software center output failed")
            if started >= next_log:
                print(f"[CENTER TEST] ABS_X={wheel.raw} STEER={steer:+.3f} "
                      f"CENTER_FORCE={100*controller.center_applied/32767:+.1f}% "
                      f"FINAL={100*controller.applied/32767:+.1f}%", flush=True)
                next_log = started + .2
            time.sleep(max(0.0, .02 - (time.monotonic() - started)))
    finally:
        controller.close()


def test_direction(device):
    """Confirmed +/-15% direction pulses with autocenter disabled."""
    import time
    print("방향 확인 테스트입니다.", flush=True)
    print("Auto Center를 잠시 해제하고 ±15% Constant Force를 적용합니다.", flush=True)
    print("핸들에서 손을 떼고 주변에 사람이 없는지 확인하십시오.", flush=True)
    if input("계속하려면 YES 입력 > ").strip() != "YES":
        print("취소했습니다. Force를 적용하지 않았습니다.", flush=True)
        return
    effect_id = -1
    stage = "Auto Center OFF write"
    try:
        device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
        print("[FFB test] Auto Center OFF (EV_FF write success)", flush=True)
        print("[FFB test] CENTER 0%", flush=True)
        time.sleep(1)
        for sign, neutral_seconds in ((1, 1.5), (-1, 1)):
            level = int(sign * 32767 * .15 * LANE_FFB_DIRECTION_SIGN)
            print(f"[FFB test] {sign * 15:+d}% -> level={level}", flush=True)
            effect = ff.Effect(ecodes.FF_CONSTANT, effect_id, 0xC000,
                               ff.Trigger(0, 0), ff.Replay(1000, 0),
                               ff.EffectType(ff_constant_effect=ff.Constant(
                                   level, ff.Envelope(0, 0, 0, 0))))
            stage = f"effect upload level={level}"
            returned_id = device.upload_effect(effect)
            print(f"[FFB test] upload success: returned effect id={returned_id}", flush=True)
            if not isinstance(returned_id, int) or returned_id < 0:
                raise ValueError(f"Invalid effect id: {returned_id}")
            effect_id = returned_id
            stage = f"effect play EV_FF write id={effect_id} value=1 level={level}"
            device.write(ecodes.EV_FF, effect_id, 1)
            print(f"[FFB test] play event sent: EV_FF write success id={effect_id} value=1 level={level}", flush=True)
            print(f"[FFB test] {sign * 15:+d}% for 1 second", flush=True)
            stage = "pulse wait"
            time.sleep(1)
            stage = f"effect stop EV_FF write id={effect_id} value=0"
            device.write(ecodes.EV_FF, effect_id, 0)
            print(f"[FFB test] stop event sent: EV_FF write success id={effect_id} value=0", flush=True)
            print("[FFB test] CENTER 0%", flush=True)
            stage = "neutral wait"
            time.sleep(neutral_seconds)
    except BaseException as exc:
        print(f"[FFB test] exception during {stage}: {type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        actions = []
        if effect_id >= 0:
            actions.extend([
                ("constant stop", lambda: device.write(ecodes.EV_FF, effect_id, 0)),
                ("constant erase", lambda: device.erase_effect(effect_id)),
            ])
        actions.append(("Auto Center OFF", lambda: device.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)))
        clean = True
        for name, action in actions:
            try:
                action()
                print(f"[FFB test] cleanup {name}: success", flush=True)
            except OSError as exc:
                clean = False
                print(f"[FFB test] cleanup {name}: {type(exc).__name__}: {exc}", flush=True)
        print("[FFB test] cleanup complete" if clean else
              "[FFB test] cleanup incomplete; see errors above", flush=True)


def main(argv=None):
    import argparse
    import signal
    # Reuse precisely the driving bridge's G29 selection and ABS_X validation.
    try:
        from .carla_bridge import find_g29
    except ImportError:
        from carla_bridge import find_g29

    parser = argparse.ArgumentParser(description="G29 capability and isolated hardware tests. Stop carla_bridge before testing.")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--diagnose', action='store_true', help='read-only capability diagnostics (default)')
    modes.add_argument('--test-center', action='store_true', help='35%% autocenter only, 10 seconds')
    modes.add_argument('--test-direction', action='store_true', help='requires successful center test and YES; autocenter OFF, +/-15%% pulses')
    modes.add_argument('--test-software-center', action='store_true', help='native center OFF; software spring only, 10 seconds')
    args = parser.parse_args(argv)
    wheel = None
    previous_term = signal.getsignal(signal.SIGTERM)

    def terminate(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    try:
        wheel = find_g29()
        if wheel is None:
            print("[G29 device] not found or inaccessible; no force applied", flush=True)
            return 1
        support = print_device_diagnostics(wheel.device)
        if args.test_center or args.test_direction or args.test_software_center:
            if not support['FF_AUTOCENTER']:
                print("FF_AUTOCENTER unsupported; test refused", flush=True)
                return 1
            if (args.test_direction or args.test_software_center) and not support['FF_CONSTANT']:
                print("FF_CONSTANT unsupported; test refused", flush=True)
                return 1
            if args.test_center:
                test_center(wheel.device)
            elif args.test_software_center:
                test_software_center(wheel)
            else:
                test_direction(wheel.device)
        return 0
    except (KeyboardInterrupt, EOFError):
        print("[FFB test] interrupted/cancelled; cleanup attempted", flush=True)
        return 130
    except OSError as exc:
        print(f"[FFB test] device error: {exc}", flush=True)
        return 1
    finally:
        if wheel is not None:
            wheel.device.close()
        signal.signal(signal.SIGTERM, previous_term)


if __name__ == '__main__':
    raise SystemExit(main())
