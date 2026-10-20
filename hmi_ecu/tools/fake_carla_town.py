#!/usr/bin/env python3
"""
CARLA 없이 "실제 도로 맵" 표시를 확인하는 가짜 송신기 (carla_hmi_bridge 와 같은 형식).
십자 교차로 + 왕복 4차선(중앙선 노란 이중선) + 건물/벽 + 주변 차.
자차가 동쪽으로 달리다가 끝에서 처음으로 돌아감.

  python3 tools/fake_carla_town.py                 # → 127.0.0.1:5601
  python3 tools/fake_carla_town.py --host 192.168.0.53
"""

import argparse
import json
import math
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "carla_tools"))

from map_stream import MapCache, map_packets, map_xf, to_frame  # noqa: E402

LW = 3.5


def lane(points_center, yaw_deg, lm, rm, j=False):
    """중심선 점들 + 진행 방향 → 왼/오른 경계 (CARLA: 오른쪽 벡터 = (−sin, cos) 가 아니라 (cos(y+90), sin(y+90)))"""
    ry = math.radians(yaw_deg + 90)
    rx, ryy = math.cos(ry), math.sin(ry)
    L = [(x - rx * LW / 2, y - ryy * LW / 2) for x, y in points_center]
    R = [(x + rx * LW / 2, y + ryy * LW / 2) for x, y in points_center]
    return {"L": L, "R": R, "lm": lm, "rm": rm, "j": j}


def build_town():
    mc = MapCache()
    xs = [x * 2.0 for x in range(-100, 301)]
    # 동서 도로: 동쪽 방향 2차선 (y = +1.75, +5.25), 서쪽 방향 2차선 (y = -1.75, -5.25)  ※ CARLA y 는 오른쪽(남쪽)
    for x0, x1 in ((-200, 52), (68, 600)):
        seg = [x for x in xs if x0 <= x <= x1]
        mc.lanes.append(lane([(x, 1.75) for x in seg], 0, ("double", "y"), ("dash", "w")))
        mc.lanes.append(lane([(x, 5.25) for x in seg], 0, ("dash", "w"), ("solid", "w")))
        mc.lanes.append(lane([(x, -1.75) for x in reversed(seg)], 180, ("double", "y"), ("dash", "w")))
        mc.lanes.append(lane([(x, -5.25) for x in reversed(seg)], 180, ("dash", "w"), ("solid", "w")))
    # 교차로 내부 (차선 표시 없음)
    seg = [x for x in xs if 52 <= x <= 68]
    for y in (-5.25, -1.75, 1.75, 5.25):
        mc.lanes.append(lane([(x, y) for x in seg], 0, ("none", "w"), ("none", "w"), j=True))
    # 남북 도로 (x = 60)
    ys = [y * 2.0 for y in range(-60, 61)]
    for y0, y1 in ((-120, -8), (8, 120)):
        seg = [y for y in ys if y0 <= y <= y1]
        mc.lanes.append(lane([(61.75, y) for y in seg], 90, ("double", "y"), ("solid", "w")))
        mc.lanes.append(lane([(58.25, y) for y in reversed(seg)], 270, ("double", "y"), ("solid", "w")))
    # 건물 / 벽 / 기둥
    for bx in range(-60, 560, 34):
        if 30 < bx < 90:
            continue
        mc.boxes.append({"x": bx, "y": 22, "yaw": 0, "l": 24, "w": 18, "h": 15})
        mc.boxes.append({"x": bx + 12, "y": -22, "yaw": 0, "l": 20, "w": 16, "h": 12})
    for bx in range(-60, 560, 15):
        mc.boxes.append({"x": bx, "y": 9.3, "yaw": 0, "l": 0.3, "w": 0.3, "h": 4})   # 가로등
    mc.boxes.append({"x": 120, "y": 11.5, "yaw": 0, "l": 30, "w": 0.4, "h": 1.2})   # 낮은 벽
    return mc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5601)
    ap.add_argument("--rate", type=float, default=20)
    ap.add_argument("--speed", type=float, default=40, help="km/h")
    ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--approach", action="store_true", help="앞에 멈춘 차로 가속 페달 밟은 채 접근 (경고 테스트)")
    ap.add_argument("--reverse", action="store_true", help="후진 기어(R), 뒤에 멈춘 차 (후방 화면 테스트)")
    args = ap.parse_args()

    mc = build_town()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)
    start = time.monotonic()
    anchor, ver, sent = None, 0, 0.0
    seq = 0
    while True:
        t = time.monotonic() - start
        ex = -60 + (t * args.speed / 3.6) % 480
        ey, eyaw = 5.25, 0.0
        pose = (ex, ey, eyaw)
        if anchor is None or math.hypot(ex - anchor[0], ey - anchor[1]) > 40:
            anchor, ver, sent = pose, ver + 1, 0.0
        if time.monotonic() - sent > 2.0:
            lanes, boxes = mc.crop(anchor)
            for pkt in map_packets(ver, lanes, boxes):
                sock.sendto(json.dumps(pkt, separators=(",", ":")).encode(), target)
            sent = time.monotonic()

        if args.reverse:
            ex = 120.0
        lead_x = ex + 22
        if args.approach:
            lead_x = 260.0
            if lead_x - ex < 6.5:
                ex = lead_x - 6.5
        traffic = [(lead_x, 5.25, 0), (ex - 6.6, 5.25, 0)] if args.reverse else [(lead_x, 5.25, 0), (ex + 9, 1.75, 0), (ex + 45, 1.75, 0), (ex - 12, 1.75, 0),
                   (ex + 30 - (t * 20) % 120, -1.75, 180), (ex + 70 - (t * 15) % 140, -5.25, 180)]
        vehicles = []
        for i, (vx, vy, vyaw) in enumerate(traffic):
            x, y = to_frame(ex, ey, eyaw, vx, vy)
            vehicles.append({"id": 100 + i, "kind": "car", "type_id": "vehicle.audi.a2" if i % 2 else "vehicle.lincoln.mkz_2020",
                             "x": round(x, 2), "y": round(y, 2), "yaw": vyaw - eyaw, "length": 4.7, "width": 1.9})
        vehicles.append({"id": 200, "kind": "walker", "type_id": "walker.pedestrian.0001",
                         "x": round(to_frame(ex, ey, eyaw, 75, 10)[0], 2), "y": round(to_frame(ex, ey, eyaw, 75, 10)[1], 2),
                         "yaw": 90, "length": 0.5, "width": 0.5})
        center = [[x, 0.0] for x in range(-12, 82, 4)]
        lead = vehicles[0]
        packet = {
            "v": 1, "type": "carla_scene", "seq": seq, "map": "FAKE_TOWN",
            "ego": {"id": 1, "type_id": "vehicle.tesla.model3", "length": 4.8, "width": 1.9,
                    "speed_kmh": 3.0 if args.reverse else args.speed, "gear": "R" if args.reverse else "D", "throttle": 0.6 if args.approach else 0.3, "brake": 0.0},
            "ego_v_fwd": -0.8 if args.reverse else args.speed / 3.6,
            "rear": {"id": 101, "distance": round(6.6 - 4.75 - 0.6 * math.sin(t), 2)} if args.reverse else {"id": None, "distance": None},
            "lane": {"center": center, "width": LW, "lanes_left": 1, "lanes_right": 0, "on_road": True},
            "lead": {"id": lead["id"], "distance": round(lead["x"] - 4.75, 2),
                     "closing": round(args.speed / 3.6, 2) if args.approach else 0.0},
            "vehicles": vehicles,
            "map_ver": ver, "map_xf": map_xf(anchor, pose),
        }
        sock.sendto(json.dumps(packet).encode(), target)
        seq += 1
        if args.duration and t > args.duration:
            break
        time.sleep(1.0 / args.rate)


if __name__ == "__main__":
    main()
