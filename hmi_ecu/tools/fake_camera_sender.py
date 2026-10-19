#!/usr/bin/env python3
"""
CARLA 없이 HMI 의 카메라 영상 경로(TCP 5602)를 확인하는 가짜 영상 송신기.

- PIL 로 간단한 원근 도로 + 앞차를 그려 JPEG 로 보냄 (영상 품질 확인용이 아니라 "연결/오버레이 정렬" 확인용)
- 오버레이는 carla_tools/camera_stream.py 의 build_overlay() 를 그대로 사용
  → 그려진 차선과 HMI 의 파란 경로가 정확히 겹치면 투영 계산이 맞는 것

  python3 tools/fake_camera_sender.py                     # → 127.0.0.1:5602
  python3 tools/fake_camera_sender.py --host 192.168.0.50 --gap 18
"""

import argparse
import io
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "carla_tools"))

from camera_stream import FrameSender, build_overlay, intrinsics, world_to_pixel  # noqa: E402


def camera_inverse(back, up, pitch_deg):
    """카메라(자차 기준 x=-back, z=up, pitch) 의 world→camera 4x4 행렬 (CARLA get_inverse_matrix 와 같은 형태)"""
    p = math.radians(pitch_deg)
    fwd = (math.cos(p), 0.0, math.sin(p))
    right = (0.0, 1.0, 0.0)
    upv = (-math.sin(p), 0.0, math.cos(p))
    pos = (-back, 0.0, up)

    def row(a):
        return [a[0], a[1], a[2], -(a[0] * pos[0] + a[1] * pos[1] + a[2] * pos[2])]
    return [row(fwd), row(right), row(upv), [0.0, 0.0, 0.0, 1.0]]


def box_vertices(cx, cy, l, w, h):
    out = []
    for dx in (-l / 2, l / 2):
        for dy in (-w / 2, w / 2):
            for z in (0.0, h):
                out.append((cx + dx, cy + dy, z))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5602)
    ap.add_argument("--fps", type=float, default=20)
    ap.add_argument("--gap", type=float, default=0, help="앞차 거리 고정 (0 이면 40→8 m 왕복)")
    ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    args = ap.parse_args()

    from PIL import Image, ImageDraw

    W, H = args.width, args.height
    K = intrinsics(W, H, 75.0)
    inv = camera_inverse(back=6.4, up=4.1, pitch_deg=-10.0)    # ChaseCamera 기본값과 같은 위치
    lane = 3.5

    sender = FrameSender(args.host, args.port)
    sender.start()
    start = time.monotonic()
    frame = 0
    print("[fake-camera] → {}:{} (HMI 가 먼저 켜져 있어야 연결됨)".format(args.host, args.port))
    try:
        while True:
            t = time.monotonic() - start
            gap = args.gap or (24 + 16 * math.cos(t * 0.35))
            lead_x = 2.4 + gap + 2.3

            img = Image.new("RGB", (W, H), (150, 170, 190))
            d = ImageDraw.Draw(img)
            horizon = world_to_pixel(inv, K, (2000, 0, 0))[1]
            d.rectangle([0, horizon, W, H], fill=(92, 104, 92))

            def poly(points, fill):
                px = [world_to_pixel(inv, K, p) for p in points]
                px = [(q[0], q[1]) for q in px if q]
                if len(px) >= 3:
                    d.polygon(px, fill=fill)

            far = 300
            poly([(0, -lane * 1.5, 0), (far, -lane * 1.5, 0), (far, lane * 1.5, 0), (0, lane * 1.5, 0)], (70, 72, 76))
            for y in (-lane * 0.5, lane * 0.5):
                off = (t * 22) % 12
                x = -off
                while x < 120:
                    poly([(x, y - 0.08, 0), (x + 4, y - 0.08, 0), (x + 4, y + 0.08, 0), (x, y + 0.08, 0)], (235, 235, 235))
                    x += 12
            for y in (-lane * 1.5, lane * 1.5):
                poly([(0, y - 0.1, 0), (far, y - 0.1, 0), (far, y + 0.1, 0), (0, y + 0.1, 0)], (230, 230, 230))

            # 앞차 (뒷면 + 지붕만 대충)
            lv = box_vertices(lead_x, 0.0, 4.6, 1.9, 1.5)
            back = [(lead_x - 2.3, -0.95, 0.2), (lead_x - 2.3, 0.95, 0.2), (lead_x - 2.3, 0.95, 1.5), (lead_x - 2.3, -0.95, 1.5)]
            roof = [(lead_x - 2.3, -0.95, 1.5), (lead_x - 2.3, 0.95, 1.5), (lead_x + 2.3, 0.95, 1.5), (lead_x + 2.3, -0.95, 1.5)]
            poly(roof, (176, 60, 60))
            poly(back, (140, 40, 40))

            # 자차 (카메라 앞쪽 아래에 보이는 차체)
            poly([(-2.4, -0.95, 1.45), (-2.4, 0.95, 1.45), (2.4, 0.95, 1.45), (2.4, -0.95, 1.45)], (210, 214, 220))

            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=80)

            left = [(x, -lane / 2, 0.05) for x in range(0, 62, 2)]
            right = [(x, lane / 2, 0.05) for x in range(0, 62, 2)]
            src = {"lane_left": left, "lane_right": right, "lead_vertices": lv, "lead_gap": round(gap, 2), "others": []}
            header = {"v": 1, "type": "camera_frame", "frame": frame, "t": t, "w": W, "h": H, "mode": "fake",
                      "overlay": build_overlay(inv, K, W, H, src)}
            sender.offer(header, buf.getvalue())
            frame += 1
            if args.duration and t > args.duration:
                break
            time.sleep(1.0 / args.fps)
    except KeyboardInterrupt:
        pass
    finally:
        sender.stop()


if __name__ == "__main__":
    main()
