"""
CAN 프레임 디코더 (HMI 용, 수신 전용).

레이아웃 출처
- 0x100 PEDAL_STATUS   : safety_ecu/canbus/decoder.py decode_pedal_status
- 0x110 HAPTIC_COMMAND : safety_ecu/canbus/sender.py send_haptic_command
- 0x120 MOTOR_COMMAND  : safety_ecu/canbus/sender.py send_motor_command
- 0x200 HAPTIC_STATUS  : safety_ecu/canbus/decoder.py decode_haptic_status

Safety ECU 디코더와 달리 HMI 는 "표시" 목적이므로 범위 밖 값이 와도
예외로 죽지 않고 clamp + valid=False 로 표시만 함.
"""

import struct

from hmi import config


PEDAL_SENSOR_BITS = {
    0x01: "ACCEL_ADC_RANGE",
    0x02: "BRAKE_ADC_RANGE",
    0x04: "ACCEL_DISCONNECT",
    0x08: "BRAKE_DISCONNECT",
    0x10: "ACCEL_CALIBRATION",
    0x20: "BRAKE_CALIBRATION",
}

HAPTIC_FAULT_NAMES = {
    0: "NO_FAULT",
    1: "ACTUATOR",
    2: "DRIVER",
    3: "INVALID_COMMAND",
    4: "COMMUNICATION",
}

MOTOR_MODE_NAMES = {      # 0x120 B0 (motor_ecu main.c 기준)
    0: "NORMAL",
    1: "LIMIT",
    2: "STOP",
}

MOTOR_STATE_NAMES = {     # 0x210 B4
    0: "STOP",
    1: "RUNNING",
    2: "OUTPUT_LIMITED",
    3: "FAULT",
}

MOTOR_FAULT_BITS = {      # 0x210 B5
    0x01: "MOTOR_DRIVER_FAULT",
    0x02: "PEDAL_TIMEOUT",
    0x04: "COMMAND_TIMEOUT",
    0x08: "ESTOP_ACTIVE",
}


def _clamp(value, low, high):
    return max(low, min(high, value))


def sensor_errors(status):
    if status == 0:
        return []
    return [name for bit, name in PEDAL_SENSOR_BITS.items() if status & bit]


def decode_pedal_status(data):
    acc = data[0]
    brk = data[1]
    acc_rate = struct.unpack_from("<h", data, 2)[0] * 0.1
    brk_rate = struct.unpack_from("<h", data, 4)[0] * 0.1
    status = data[6]
    return {
        "accelerator": _clamp(acc, 0, 100),
        "brake": _clamp(brk, 0, 100),
        "accelerator_rate": acc_rate,
        "brake_rate": brk_rate,
        "sensor_status": status,
        "sensor_errors": sensor_errors(status),
        "alive": data[7],
        "valid": acc <= 100 and brk <= 100,
    }


def decode_haptic_command(data):
    return {
        "risk_level": data[0],
        "vibration_command": data[1],
        "intensity": data[2],
        "frequency": data[3],
        "alive": data[7],
        "valid": data[0] <= 3 and data[1] in (0, 1),
    }


def decode_motor_command(data):
    return {
        "control_mode": data[0],
        "output_limit": _clamp(data[1], 0, 100),
        "risk_level": data[2],
        "command_flags": data[3],
        "estop": bool(data[3] & 0x02),
        "alive": data[7],
        "valid": data[0] in MOTOR_MODE_NAMES and data[1] <= 100,
    }


def decode_motor_status(data):
    """
    0x210 MOTOR_STATUS (motor_ecu/Core/Src/main.c, safety_ecu decoder.py 와 동일)
      B0 Actual PWM %, B1 Applied Output Limit %, B2~3 차속 uint16 LE 0.01 m/s (0xFFFF = Invalid),
      B4 Motor State, B5 Fault Flags, B6 Applied Command Counter, B7 Alive
    """
    raw = struct.unpack_from("<H", data, 2)[0]
    speed_mps = None if raw == 0xFFFF else raw * 0.01
    faults = data[5]
    return {
        "actual_pwm": _clamp(data[0], 0, 100),
        "applied_limit": _clamp(data[1], 0, 100),
        "speed_mps": speed_mps,
        "speed_kmh": None if speed_mps is None else speed_mps * 3.6,
        "motor_state": data[4],
        "motor_state_name": MOTOR_STATE_NAMES.get(data[4], str(data[4])),
        "fault_flags": faults,
        "fault_errors": [n for b, n in MOTOR_FAULT_BITS.items() if faults & b],
        "estop_active": bool(faults & 0x08),
        "applied_counter": data[6],
        "alive": data[7],
        "valid": data[0] <= 100 and data[1] <= 100 and data[4] in MOTOR_STATE_NAMES,
    }


def decode_haptic_status(data):
    return {
        "risk_level": data[0],
        "vibration_status": data[1],
        "intensity": data[2],
        "frequency": data[3],
        "fault_code": data[4],
        "command_timeout": data[5],
        "ecu_status": data[6],
        "alive": data[7],
        "valid": data[0] <= 3 and data[1] in (0, 1),
    }


def decode_generic(data):
    return {"alive": data[7] if len(data) >= 8 else None, "valid": True}


DECODERS = {
    config.PEDAL_STATUS_ID: ("PEDAL_STATUS", decode_pedal_status),
    config.HAPTIC_COMMAND_ID: ("HAPTIC_COMMAND", decode_haptic_command),
    config.MOTOR_COMMAND_ID: ("MOTOR_COMMAND", decode_motor_command),
    config.HAPTIC_STATUS_ID: ("HAPTIC_STATUS", decode_haptic_status),
    config.MOTOR_STATUS_ID: ("MOTOR_STATUS", decode_motor_status),
}


def decode(can_id, data):
    """(name, dict) 반환. 모르는 ID 거나 DLC != 8 이면 None."""
    entry = DECODERS.get(can_id)
    if entry is None or len(data) != 8:
        return None
    name, fn = entry
    return name, fn(bytes(data))
