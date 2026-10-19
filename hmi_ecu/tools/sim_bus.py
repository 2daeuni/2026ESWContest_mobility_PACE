#!/usr/bin/env python3
"""
하드웨어 없이 HMI 통합 테스트용 가상 ECU.

모드
  full   : Pedal/Haptic/Motor ECU 의 CAN 프레임 + Safety ECU 의 UDP 상태를 모두 흉내
           → HMI 단독 테스트 (Safety ECU 안 돌려도 됨)
  can    : CAN 프레임만 송신 (UDP 없음) → Safety ECU 가 죽었을 때 HMI fallback 확인
  pedal  : Pedal ECU(0x100) + Haptic ECU 상태(0x200) + Motor ECU 상태(0x210) 송신
           → 실제 safety_ecu/main.py 와 같이 돌려서 Safety → HMI 경로 확인
  nomotor: pedal 과 같지만 0x210 없음 (Motor ECU 미연결 → Safety 가 정지 명령 보내는지 확인)

라즈베리파이에서 가상 CAN 만들기
  sudo modprobe vcan
  sudo ip link add dev vcan0 type vcan
  sudo ip link set up vcan0

예시
  python3 tools/sim_bus.py --channel vcan0                # full
  python3 main.py --can vcan0 --windowed                  # 다른 터미널
"""

import argparse
import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import can  # noqa: E402

from hmi import config  # noqa: E402
from hmi.demo_scenario import DemoScenario  # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["full", "can", "pedal", "nomotor"], default="full")
    p.add_argument("--channel", default="vcan0")
    p.add_argument("--interface", default="socketcan")
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=config.UDP_PORT)
    p.add_argument("--duration", type=float, default=0, help="0 = 무한")
    args = p.parse_args()

    bus = can.Bus(interface=args.interface, channel=args.channel)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    scenario = DemoScenario()

    allowed = {
        "full": None,
        "can": None,
        "pedal": {config.PEDAL_STATUS_ID, config.HAPTIC_STATUS_ID, config.MOTOR_STATUS_ID},
        "nomotor": {config.PEDAL_STATUS_ID, config.HAPTIC_STATUS_ID},
    }[args.mode]

    period = 0.01
    tick = 0
    start = time.monotonic()
    next_t = start
    print("[SIM] mode={} channel={} ({})".format(args.mode, args.channel, args.interface))

    try:
        while True:
            frames, packet = scenario.step(period)
            for can_id, data in frames:
                if allowed is not None and can_id not in allowed:
                    continue
                try:
                    bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))
                except can.CanError as error:
                    print("[SIM] CAN TX error:", error)

            if args.mode == "full" and tick % 3 == 0:
                packet["seq"] = tick
                udp.sendto(json.dumps(packet).encode(), (args.udp_host, args.udp_port))

            if tick % 100 == 0:
                print("[SIM] t={:5.1f}s level={} acc={} dist={}".format(
                    scenario.t, packet["risk_level"], packet["accelerator"], packet["distance_cm"]))

            tick += 1
            if args.duration and time.monotonic() - start > args.duration:
                break
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
