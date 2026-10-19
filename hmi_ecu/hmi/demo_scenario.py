"""
데모 시나리오: 정상 주행 → 전방 차량 접근(L1) → 경고(L2) → 가속 페달 오조작(L3)
→ 브레이크 전환 → 정상 주행 복귀 (약 40초 반복)

실제 ECU 와 같은 CAN 바이트 / Safety ECU 와 같은 UDP dict 를 만들어냄.
tools/sim_bus.py 에서도 이 클래스를 재사용해서 vcan0 + UDP 로 송신함.
"""

import struct

from hmi import config


HAPTIC_TABLE = {  # control/command_mapper.py 와 동일
    0: (0, 0, 0),
    1: (1, 30, 20),
    2: (1, 45, 20),
    3: (1, 65, 30),
}


def _lerp(a, b, x):
    x = max(0.0, min(1.0, x))
    return a + (b - a) * x


class DemoScenario:
    LOOP = 40.0

    def __init__(self):
        self.t = 0.0
        self.speed = 40.0          # km/h
        self.lead_speed = 40.0
        self.distance = 35.0       # m
        self.alive = {k: 0 for k in ("pedal", "hcmd", "mcmd", "hsta", "msta")}
        self.level = 0
        self.prev_acc = 0.0

    # ------------------------------------------------------------
    def _script(self, t):
        """시간 → (accel %, brake %, ego km/h, 전방거리 m, environment_ok)  (스크립트 기반 운동)"""
        if t < 6:
            return 22, 0, 40, 35, True
        if t < 12:
            return 22, 0, 40, _lerp(35, 22, (t - 6) / 6), True
        if t < 16:
            return 18, 0, 38, _lerp(22, 14, (t - 12) / 4), True
        if t < 20:          # 오조작: 브레이크 대신 가속 페달을 깊게 밟음
            return _lerp(18, 86, (t - 16) / 0.6), 0, _lerp(38, 46, (t - 16) / 4), _lerp(14, 6.5, (t - 16) / 4), True
        if t < 23:          # 햅틱 경고 후 브레이크로 전환
            return 0, _lerp(0, 62, (t - 20) / 0.4), _lerp(46, 10, (t - 20) / 3), _lerp(6.5, 6.0, (t - 20) / 3), True
        if t < 28:
            return 15, 0, _lerp(10, 32, (t - 23) / 5), _lerp(6.0, 30, (t - 23) / 5), True
        if t < 32:
            return 18, 0, 35, _lerp(30, 34, (t - 28) / 4), True
        if t < 36:          # 정상 순항
            return 18, 0, 35, 34, True
        return 20, 0, 38, 35, True

    def _level(self, ttc, dist, acc):
        if dist is None:
            return 0
        lv = 0
        if dist < 25 or (ttc is not None and ttc < 4.0):
            lv = 1
        if dist < 16 or (ttc is not None and ttc < 2.5):
            lv = 2
        if (dist < 12 or (ttc is not None and ttc < 1.6)) and acc > 30:
            lv = 3
        return lv

    # ------------------------------------------------------------
    def step(self, dt):
        self.t += dt
        t = self.t % self.LOOP

        acc, brk, ego, dist, environment_ok = self._script(t)
        prev_dist = self.distance
        self.speed = ego
        self.distance = dist
        closing_mps = (prev_dist - dist) / dt if dt > 0 else 0.0
        self.lead_speed = ego - closing_mps * 3.6

        closing = closing_mps
        ttc = self.distance / closing if closing > 0.3 else None
        if ttc is not None and ttc > 20:
            ttc = None

        dist_valid = environment_ok
        self.level = self._level(ttc, self.distance if dist_valid else None, acc) if dist_valid else 0
        failsafe = not environment_ok

        acc_rate = (acc - self.prev_acc) / dt if dt > 0 else 0.0
        self.prev_acc = acc

        if failsafe:
            vib, inten, freq = 0, 0, 0
            mode, out_limit, flags = 2, 0, 0x03
        else:
            vib, inten, freq = HAPTIC_TABLE[self.level]
            mode, out_limit, flags = (1, 30, 1) if self.level == 3 else (0, 100, 1)

        bns = min(100.0, max(0.0, (40 - self.distance) * 2.5 + (0 if ttc is None else max(0, 6 - ttc) * 10)))
        pms = min(100.0, max(0.0, acc * 0.6 + max(0.0, acc_rate) * 0.15))
        total = min(100.0, bns * 0.55 + pms * 0.45)

        frames = []
        for key in self.alive:
            self.alive[key] = (self.alive[key] + 1) & 0xFF

        frames.append((config.PEDAL_STATUS_ID, bytes([
            int(acc), int(brk),
            *struct.pack("<h", int(max(-32768, min(32767, acc_rate * 10)))),
            *struct.pack("<h", 0),
            0x00, self.alive["pedal"],
        ])))
        frames.append((config.HAPTIC_COMMAND_ID, bytes([
            self.level, vib, inten, freq, 0, 0, 0, self.alive["hcmd"]])))
        frames.append((config.MOTOR_COMMAND_ID, bytes([
            mode, out_limit, self.level, flags, 0, 0, 0, self.alive["mcmd"]])))
        frames.append((config.HAPTIC_STATUS_ID, bytes([
            self.level, vib, inten, freq, 0, 0, 1, self.alive["hsta"]])))
        # 0x210: PWM, 적용 제한, 차속 0.01 m/s (uint16 LE), 상태, 고장, 적용 명령 카운터, alive
        pwm = 0 if mode == 2 else int(min(acc, out_limit))
        state = 0 if mode == 2 else (2 if mode == 1 else (1 if pwm > 0 else 0))
        frames.append((config.MOTOR_STATUS_ID, bytes([
            pwm, out_limit, *struct.pack("<H", int(self.speed / 3.6 * 100)), state,
            0x08 if mode == 2 else 0x00, self.alive["mcmd"], self.alive["msta"]])))

        packet = {
            "v": 1,
            "accelerator": int(acc),
            "brake": int(brk),
            "accelerator_rate": round(acc_rate, 1),
            "brake_rate": 0.0,
            "pedal_connected": True,
            "pedal_sensor_status": 0,
            "distance_valid": dist_valid,
            "distance_cm": round(self.distance * 100, 1) if dist_valid else None,
            "vehicle_speed": round(self.speed / 3.6, 2),      # m/s (노션 규격)
            "vehicle_speed_unit": "m/s",
            "speed_source": "CARLA",
            "ttc": round(ttc, 2) if (ttc is not None and dist_valid) else None,
            "relative_speed": round(self.lead_speed - self.speed, 1),
            "gear": "D",
            "carla_connected": True,
            "bns": round(bns, 1),
            "bns_available": dist_valid,
            "pms": round(pms, 1),
            "pms_available": True,
            "total_risk": round(total, 1),
            "risk_level": self.level,
            "risk_available": dist_valid,
            "score_max": 100.0,
            "failsafe_active": failsafe,
            "failsafe_reason": "CARLA_COMMUNICATION_ERROR" if failsafe else None,
            "haptic_vibration_command": vib,
            "haptic_intensity": inten,
            "haptic_frequency": freq,
            "motor_control_mode": mode,
            "motor_output_limit": out_limit,
            "motor_command_flags": flags,
            "haptic_connected": True,
            "haptic_available": True,
            "haptic_fault_code": 0,
        }
        return frames, packet
