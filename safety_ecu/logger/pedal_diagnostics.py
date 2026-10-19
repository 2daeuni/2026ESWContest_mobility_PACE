"""Read-only PEDAL_STATUS tracing, independent of control validation."""

import struct
from datetime import datetime, timezone


class PedalDiagnostics:
    def __init__(self):
        self.previous_status = None

    def capture(self, data, can_timestamp):
        timestamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        raw = bytes(data)
        status = raw[6] if len(raw) >= 7 else None
        previous = self.previous_status
        transition = status == 0x04 and previous != 0x04
        if status is not None:
            self.previous_status = status
        return {
            "timestamp": timestamp,
            "can_timestamp": can_timestamp,
            "raw": raw,
            "status": status,
            "previous": previous,
            "transition": transition,
        }

    def report(self, sample, accepted, state, failsafe):
        raw = sample["raw"]
        status = sample["status"]
        previous = sample["previous"]
        values = ""
        if len(raw) == 8:
            acc_rate, brake_rate = struct.unpack_from("<hh", raw, 2)
            values = (
                f"ACC={raw[0]}% BRAKE={raw[1]}% "
                f"ACC_RATE={acc_rate * 0.1:.1f}%/s "
                f"BRAKE_RATE={brake_rate * 0.1:.1f}%/s ALIVE={raw[7]} "
            )
        status_text = "MISSING" if status is None else f"0x{status:02X}"
        previous_text = "UNKNOWN" if previous is None else f"0x{previous:02X}"
        state_status = state.pedal_sensor_status
        state_text = "NONE" if state_status is None else f"0x{state_status:02X}"
        event = "STATUS_04_ENTER" if sample["transition"] else "RX"
        print(
            f"[PEDAL-DIAG] {event} TS={sample['timestamp']} "
            f"CAN_TS={sample['can_timestamp']:.6f} ID=0x100 "
            f"{values}BYTE6={status_text} PREV={previous_text} "
            f"ACCEPTED={int(accepted)} STATE_STATUS={state_text} "
            f"FAILSAFE_ACTIVE={int(failsafe.active)} "
            f"FAILSAFE_REASON={failsafe.reason} RAW={raw.hex()}"
        )
