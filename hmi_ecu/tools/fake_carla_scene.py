#!/usr/bin/env python3
"""
CARLA 없이 HMI 의 CARLA 장면 표시를 확인하는 가짜 장면 송신기.
carla_hmi_bridge.py 와 같은 형식의 패킷을 보냄 (3차선 완만한 커브, 주변 차량 여러 대).

  python3 tools/fake_carla_scene.py                 # → 127.0.0.1:5601
  python3 tools/fake_carla_scene.py --host 192.168.0.50
"""

import argparse
import json
import math
import socket
import time

TYPES = ["vehicle.audi.a2", "vehicle.lincoln.mkz_2020", "vehicle.toyota.prius",
         "vehicle.nissan.patrol", "vehicle.mercedes.coupe_2020", "vehicle.carlamotors.carlacola"]


def curve(x, t):
    k = 0.0012 * math.sin(t * 0.15)          # 곡률이 천천히 바뀜
    return k * x * x


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5601)
    p.add_argument("--rate", type=float, default=20)
    p.add_argument("--duration", type=float, default=0)
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lane_w = 3.5
    # (차선 오프셋, 초기 x, 상대속도 m/s, 차종)
    traffic = [
        (0, 28.0, -0.6, 1),     # 같은 차선 앞차 (서서히 가까워짐)
        (-1, 12.0, 1.2, 0),
        (-1, 55.0, 0.4, 3),
        (1, 6.0, -0.3, 2),
        (1, 38.0, 0.8, 5),
        (0, 85.0, 0.2, 4),
        (1, -12.0, 0.5, 0),
    ]
    start = time.monotonic()
    seq = 0
    while True:
        t = time.monotonic() - start
        vehicles = []
        for i, (lane, x0, vrel, ti) in enumerate(traffic):
            x = x0 + vrel * t
            span = 140.0
            x = ((x + 20) % span) - 20      # 범위 밖으로 나가면 반대편에서 다시
            y = curve(x, t) + lane * lane_w
            yaw = math.degrees(math.atan(2 * 0.0012 * math.sin(t * 0.15) * x))
            vehicles.append({"id": 100 + i, "kind": "car", "type_id": TYPES[ti],
                             "x": round(x, 2), "y": round(y, 2), "yaw": round(yaw, 1),
                             "length": 4.6 if ti != 5 else 6.0, "width": 1.9 if ti != 5 else 2.3,
                             "speed_kmh": round(80 + vrel * 3.6, 1)})
        vehicles.append({"id": 200, "kind": "walker", "type_id": "walker.pedestrian.0001",
                         "x": 60.0, "y": curve(60, t) + 7.5, "yaw": 90, "length": 0.5, "width": 0.5})

        center = [[x, round(curve(x, t), 3)] for x in range(-12, 56, 2)]
        # 55 m 앞 교차로에서 왼쪽으로 꺾이는 차선 (브리지가 실제로 보내는 모양 재현)
        for k in range(1, 20):
            ang = min(math.pi / 2, k * 0.12)
            center.append([round(55 + 12 * math.sin(ang), 2), round(curve(55, t) - 12 * (1 - math.cos(ang)), 2)])
        lead = min((v for v in vehicles if v["kind"] == "car" and v["x"] > 0
                    and abs(v["y"] - curve(v["x"], t)) < lane_w / 2), key=lambda v: v["x"], default=None)
        packet = {
            "v": 1, "type": "carla_scene", "seq": seq, "map": "FAKE",
            "ego": {"id": 1, "type_id": "vehicle.tesla.model3", "length": 4.8, "width": 1.9,
                    "speed_kmh": 80.0, "gear": "D"},
            "lane": {"center": center, "width": lane_w, "lanes_left": 1, "lanes_right": 1, "junction": False},
            "lead": {"id": lead["id"] if lead else None,
                     "distance": round(lead["x"] - 4.8 / 2 - lead["length"] / 2, 2) if lead else None},
            "vehicles": vehicles,
        }
        sock.sendto(json.dumps(packet).encode(), (args.host, args.port))
        seq += 1
        if args.duration and t > args.duration:
            break
        time.sleep(1.0 / args.rate)


if __name__ == "__main__":
    main()
