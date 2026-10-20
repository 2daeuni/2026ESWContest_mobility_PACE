#!/usr/bin/env python3
"""
CARLA → PACE HMI 장면 브리지  (CARLA 가 돌아가는 PC 에서 실행)

기존 CARLA 주행 스크립트(G29 페달/조향, Safety ECU 와 UDP 통신)는 그대로 두고,
같은 CARLA 서버에 "두 번째 클라이언트"로 붙어서 읽기만 함 (차량 제어/틱 안 함).

  자차 기준 주변 차량·보행자 위치, 차선 중심선(곡선), 앞차 거리, 자차 속도/기어
  → UDP JSON 20 Hz → 라즈베리파이 HMI (기본 포트 5601)
  → HMI 가 Tesla 화면처럼 실제 CARLA 차량 배치를 그대로 그림

카메라 영상 (기본 켜짐): 자차 운전석 앞(R 기어면 뒤 범퍼)에 카메라를 붙여 CARLA 실제 렌더링 영상을 TCP 5602 로 같이 보냄
  --camera real   CARLA 화면 그대로 (기본, R 기어면 후방 카메라)
  --camera clean  테슬라식 정리 화면 (실제 3D 차 + 단색 도로, 배경 제거)
  --camera off    영상 없이 장면 데이터만 (HMI 는 3D 주행 화면으로 표시)

실행 예
  python carla_hmi_bridge.py --hmi-host 192.168.50.2
  python carla_hmi_bridge.py --hmi-host 192.168.50.2 --role-name hero
  python carla_hmi_bridge.py --list                   # 월드의 차량 목록 확인

자차 찾는 순서
  1. --actor-id
  2. role_name 이 --role-name (기본: hero, ego, ego_vehicle) 중 하나인 차량
  3. 관전자(spectator) 카메라에 가장 가까운 차량 (보통 주행 스크립트가 카메라를 자차에 붙여 둠)
"""

import argparse
import json
import math
import os
import socket
import sys
import time

# Safety 로 보내는 것과 같은 근거리 레이더 방식 장애물 탐지 (carla_sim/obstacle_sensing.py) — 있으면 사용
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_HERE, "..", "..", "carla_sim"), _HERE):
    if os.path.isfile(os.path.join(_p, "obstacle_sensing.py")) and _p not in sys.path:
        sys.path.insert(0, _p)
        break
try:
    from obstacle_sensing import find_obstacle_for_safety
except ImportError:
    find_obstacle_for_safety = None


# =====================================================================
# 순수 계산 함수 (carla 없이 테스트 가능)
# =====================================================================
def to_ego(ego_x, ego_y, ego_yaw_deg, px, py):
    """월드 좌표 → 자차 좌표 (x 앞 +, y 오른쪽 +). CARLA 월드도 왼손 좌표계(y 오른쪽)."""
    dx, dy = px - ego_x, py - ego_y
    yaw = math.radians(ego_yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c + dy * s, -dx * s + dy * c


def to_ego_3d(ego_tf, px, py, pz, inverse=None):
    """World point -> full ego-local XYZ; CARLA returns a nested 4x4 matrix.

    Unlike to_ego(), this rendering transform includes ego pitch and roll.
    Pass a cached inverse when converting a polyline or multiple actors.
    """
    inv = inverse if inverse is not None else ego_tf.get_inverse_matrix()
    point = (px, py, pz, 1.0)
    return tuple(sum(inv[row][col] * point[col] for col in range(4)) for row in range(3))


def relative_rotation(ego_inverse, actor_tf):
    """CARLA Euler yaw/pitch/roll of R_ego^T R_actor, for rendering only.

    Compose rotations before extracting angles: Euler subtraction is not exact
    when yaw, pitch and roll are combined. CARLA forward is column 0.
    """
    actor = actor_tf.get_matrix()
    r = [[sum(ego_inverse[i][k] * actor[k][j] for k in range(3))
          for j in range(3)] for i in range(3)]
    pitch = math.atan2(r[2][0], math.hypot(r[0][0], r[1][0]))
    if math.hypot(r[0][0], r[1][0]) > 1e-8:
        yaw = math.atan2(r[1][0], r[0][0])
        roll = math.atan2(-r[2][1], r[2][2])
    else:
        yaw = math.atan2(-r[0][1], r[1][1])
        roll = 0.0
    return tuple(wrap_deg(math.degrees(a)) for a in (yaw, pitch, roll))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def interp_center_y(center, x):
    """차선 중심 polyline 에서 x 위치의 y. 범위 밖이면 끝점 기준 외삽 없이 끝값."""
    if not center:
        return 0.0
    if x <= center[0][0]:
        return center[0][1]
    for (x0, y0), (x1, y1) in zip(center, center[1:]):
        if x0 <= x <= x1:
            if x1 == x0:
                return y0
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return center[-1][1]


def _road_z(center3d, x):
    """자차 좌표 차선 중심선(3D) 의 x 위치 높이. 없으면 None"""
    if not center3d:
        return None
    if x <= center3d[0][0]:
        return center3d[0][2]
    for a, b in zip(center3d, center3d[1:]):
        if a[0] <= x <= b[0]:
            t = 0.0 if b[0] == a[0] else (x - a[0]) / (b[0] - a[0])
            return a[2] + (b[2] - a[2]) * t
    return center3d[-1][2]


def choose_lead(objects, center, lane_width, ego_length, center3d=None, vertical_limit=2.5):
    """같은 차선 앞쪽에서 가장 가까운 물체. (id, 범퍼 간 거리) 또는 (None, None).
    높이 차이가 큰 물체(다리 아래·위 다른 층)는 제외: 차선 3D 높이와 비교, 없으면 경사 12 % 까지 허용"""
    best = None
    for o in objects:
        if o["x"] <= 0:
            continue
        oz = o.get("local_z")
        if oz is not None:
            rz = _road_z(center3d, o["x"])
            ref = rz if rz is not None else 0.0
            lim = vertical_limit if rz is not None else vertical_limit + 0.12 * o["x"]
            if abs(oz - ref) > lim:
                continue
        cy = interp_center_y(center, o["x"])
        if abs(o["y"] - cy) > lane_width * 0.5 + o.get("width", 1.8) * 0.25:
            continue
        gap = o["x"] - ego_length / 2.0 - o.get("length", 4.5) / 2.0
        if best is None or gap < best[1]:
            best = (o["id"], max(0.0, gap))
    return best if best else (None, None)


# =====================================================================
# CARLA 읽기
# =====================================================================
def find_ego(world, args):
    vehicles = list(world.get_actors().filter("vehicle.*"))
    if args.actor_id is not None:
        for v in vehicles:
            if v.id == args.actor_id:
                return v
        return None
    names = [n.strip() for n in args.role_name.split(",") if n.strip()]
    for v in vehicles:
        if v.attributes.get("role_name", "") in names:
            return v
    if not vehicles:
        return None
    spec = world.get_spectator().get_location()
    return min(vehicles, key=lambda v: v.get_location().distance(spec))


def road_grade_deg(wp, ego_yaw, sample_distance=4.0):
    """Road-only grade from world waypoint heights, independent of actor tilt.

    A +/-4m chord averages the road profile rather than relying on vehicle body
    pitch or ego-local geometry. At a road end use the available half-chord;
    if no horizontal chord exists, publish zero rather than an invalid angle.
    """
    opposite = abs(wrap_deg(wp.transform.rotation.yaw - ego_yaw)) > 90

    def pick(candidates):
        candidates = [p for p in candidates
                      if abs(wrap_deg(p.transform.rotation.yaw + (180 if opposite else 0) - ego_yaw)) <= 60]
        return min(candidates, key=lambda p: abs(wrap_deg(
            p.transform.rotation.yaw + (180 if opposite else 0) - ego_yaw))) if candidates else wp

    front = pick(wp.previous(sample_distance) if opposite else wp.next(sample_distance))
    rear = pick(wp.next(sample_distance) if opposite else wp.previous(sample_distance))
    f, r = front.transform.location, rear.transform.location
    horizontal = math.hypot(f.x - r.x, f.y - r.y)
    dz = f.z - r.z
    if math.isfinite(horizontal) and math.isfinite(dz) and horizontal > 1e-6:
        return round(math.degrees(math.atan2(dz, horizontal)), 3)
    # No usable world-height segment. Never substitute actor/body pitch.
    return 0.0


def lane_info(carla, world_map, ego_tf, ahead, behind, step):
    """자차 차선 중심선 (자차 좌표), 차선 폭, 좌우 같은 방향 차선 수."""
    wp = world_map.get_waypoint(ego_tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        return None

    ex, ey, eyaw = ego_tf.location.x, ego_tf.location.y, ego_tf.rotation.yaw

    # 자차가 차선 진행 방향과 반대로 서 있으면(후진·유턴·역방향 스폰) 앞/뒤를 바꿔서 따라감
    opposite = abs(wrap_deg(wp.transform.rotation.yaw - eyaw)) > 90
    ahead_fn = (lambda c, d: c.previous(d)) if opposite else (lambda c, d: c.next(d))
    behind_fn = (lambda c, d: c.next(d)) if opposite else (lambda c, d: c.previous(d))
    face = (lambda c: wrap_deg(c.transform.rotation.yaw + 180)) if opposite else (lambda c: c.transform.rotation.yaw)

    back = []
    w = wp
    dist = 0.0
    while dist < behind:
        prev = behind_fn(w, step)
        if not prev:
            break
        w = prev[0]
        dist += step
        back.append(w)
    back.reverse()

    fwd = [wp]
    w = wp
    dist = 0.0
    while dist < ahead:
        nxt = ahead_fn(w, step)
        if not nxt:
            break
        # 갈림길: 자차 진행 방향과 가장 비슷한 쪽 (교차로에서 옆 도로로 새지 않게)
        w = min(nxt, key=lambda c: abs(wrap_deg(face(c) - eyaw)))
        # 자차 방향에서 60° 넘게 꺾이면 그 앞은 화면에 의미 없음 → 중단
        if abs(wrap_deg(face(w) - eyaw)) > 60:
            break
        dist += step
        fwd.append(w)

    inverse = ego_tf.get_inverse_matrix()
    center = []
    center3d = []
    for p in back + fwd:
        x, y = to_ego(ex, ey, eyaw, p.transform.location.x, p.transform.location.y)
        center.append([round(x, 2), round(y, 2)])
        loc = p.transform.location
        center3d.append([round(c, 3) for c in to_ego_3d(ego_tf, loc.x, loc.y, loc.z, inverse)])
    center.sort(key=lambda c: c[0])
    center3d.sort(key=lambda c: c[0])
    # 앞쪽이 짧게 끊기면(교차로·막다른 길) 마지막 방향으로 곧게 연장 → 도로가 갑자기 사라지지 않게
    if len(center) >= 2 and center[-1][0] < ahead * 0.5:
        (x0, y0), (x1, y1) = center[-2], center[-1]
        dx, dy = x1 - x0, y1 - y0
        n = math.hypot(dx, dy) or 1.0
        dx, dy = dx / n, dy / n
        if dx < 0.3:
            dx, dy = 1.0, 0.0
        # Extend solely in the same full local frame; legacy XY stays unchanged.
        tail = center3d[-1]
        direction = [tail[i] - center3d[-2][i] for i in range(3)]
        norm3d = math.sqrt(sum(c*c for c in direction)) or 1.0
        direction = [c / norm3d for c in direction]
        k = 1
        while center[-1][0] < ahead * 0.6 and k < 40:
            center.append([round(x1 + dx * step * k, 2), round(y1 + dy * step * k, 2)])
            center3d.append([round(tail[i] + direction[i] * step * k, 3) for i in range(3)])
            k += 1

    def count_side(getter):
        n = 0
        cur = wp
        while n < 3:
            nb = getter(cur)
            if nb is None or nb.lane_type != carla.LaneType.Driving:
                break
            if (nb.lane_id > 0) != (wp.lane_id > 0):    # 반대 방향 차선이면 중단
                break
            n += 1
            cur = nb
        return n

    return {
        "center": center,
        "center3d": center3d,
        "road_grade_deg": road_grade_deg(wp, eyaw),
        "width": round(wp.lane_width, 2),
        "lanes_left": count_side(lambda c: c.get_left_lane()),
        "lanes_right": count_side(lambda c: c.get_right_lane()),
        "junction": wp.is_junction,
        # 자차가 차선 위에 있는지 (인도·광장·주차장이면 False → HMI 가 자차 차선 강조를 끔)
        "on_road": ego_tf.location.distance(wp.transform.location) < wp.lane_width * 0.8,
    }


def lane_world_edges(carla, world_map, ego_tf, ahead=60.0, step=2.0):
    """자차 차선 좌/우 경계 (월드 좌표, 바닥 +5 cm). 카메라 AR 오버레이용."""
    wp = world_map.get_waypoint(ego_tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        return [], []
    eyaw = ego_tf.rotation.yaw
    left, right = [], []
    w = wp
    dist = 0.0
    while True:
        loc = w.transform.location
        r = w.transform.get_right_vector()
        half = w.lane_width / 2.0
        left.append((loc.x - r.x * half, loc.y - r.y * half, loc.z + 0.05))
        right.append((loc.x + r.x * half, loc.y + r.y * half, loc.z + 0.05))
        if dist >= ahead:
            break
        nxt = w.next(step)
        if not nxt:
            break
        w = min(nxt, key=lambda c: abs(wrap_deg(c.transform.rotation.yaw - eyaw)))
        if abs(wrap_deg(w.transform.rotation.yaw - eyaw)) > 60:
            break
        dist += step
    return left, right


def overlay_source(carla, world, world_map, ego, packet):
    tf = ego.get_transform()
    left, right = lane_world_edges(carla, world_map, tf)
    src = {"lane_left": left, "lane_right": right, "lead_gap": packet["lead"]["distance"],
           "lead_vertices": None, "others": []}
    lead_id = packet["lead"]["id"]
    for v in packet["vehicles"]:
        if v.get("kind") != "car" or v["x"] <= 0 or v["x"] > 70:
            continue
        a = world.get_actor(v["id"])
        if a is None:
            continue
        verts = [(p.x, p.y, p.z) for p in a.bounding_box.get_world_vertices(a.get_transform())]
        if v["id"] == lead_id:
            src["lead_vertices"] = verts
        else:
            src["others"].append(verts)
    gaps = [g for g in ((packet.get("rear") or {}).get("distance"), packet.get("rear_static")) if g is not None]
    src["rear_gap"] = min(gaps) if gaps else None
    return src


def build_packet(carla, world, world_map, ego, args, seq):
    tf = ego.get_transform()
    ex, ey, eyaw = tf.location.x, tf.location.y, tf.rotation.yaw
    ext = ego.bounding_box.extent
    ego_len, ego_wid = ext.x * 2, ext.y * 2
    vel = ego.get_velocity()
    speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2) * 3.6
    try:
        ctrl = ego.get_control()
        gear = "R" if ctrl.reverse else ("P" if ctrl.hand_brake and speed < 0.5 else "D")
        throttle, brake, steer = ctrl.throttle, ctrl.brake, ctrl.steer
    except Exception:
        gear, throttle, brake, steer = None, None, None, None

    inverse = tf.get_inverse_matrix()
    objects = []
    for a in world.get_actors():
        tid = a.type_id
        if a.id == ego.id:
            continue
        if tid.startswith("vehicle."):
            kind = "car"
        elif tid.startswith("walker.pedestrian"):
            kind = "walker"
        else:
            continue
        loc = a.get_location()
        x, y = to_ego(ex, ey, eyaw, loc.x, loc.y)
        if not (-args.behind <= x <= args.ahead and abs(y) <= args.side):
            continue
        b = a.bounding_box.extent
        v = a.get_velocity()
        # 자차 진행 방향 성분 속도 (TTC 계산용)
        vx_ego = (v.x * math.cos(math.radians(eyaw)) + v.y * math.sin(math.radians(eyaw)))
        actor_tf = a.get_transform()
        rotation = actor_tf.rotation
        local = to_ego_3d(tf, loc.x, loc.y, loc.z, inverse)
        local_yaw, local_pitch, local_roll = relative_rotation(inverse, actor_tf)
        objects.append({
            "id": a.id,
            "kind": kind,
            "type_id": tid,
            "x": round(x, 2),
            "y": round(y, 2),
            "yaw": round(wrap_deg(rotation.yaw - eyaw), 1),
            "z": round(local[2], 3),
            "local_x": round(local[0], 3),
            "local_y": round(local[1], 3),
            "local_z": round(local[2], 3),
            "local_yaw": round(local_yaw, 3),
            "local_pitch": round(local_pitch, 3),
            "local_roll": round(local_roll, 3),
            "relative_yaw": round(local_yaw, 3),
            "relative_pitch": round(local_pitch, 3),
            "relative_roll": round(local_roll, 3),
            "pitch": round(rotation.pitch, 2),
            "roll": round(rotation.roll, 2),
            "length": round(b.x * 2, 2),
            "width": round(b.y * 2, 2),
            "speed_kmh": round(math.sqrt(v.x ** 2 + v.y ** 2) * 3.6, 1),
            "v_fwd": round(vx_ego, 2),
        })

    lane = lane_info(carla, world_map, tf, args.ahead, 12.0, 2.0) or {
        "center": [[-10, 0], [args.ahead, 0]], "width": 3.5, "lanes_left": 1, "lanes_right": 1}
    packet_road_grade = lane.get("road_grade_deg") or 0.0
    if lane.get("on_road", True):
        lead_id, lead_gap = choose_lead(objects, lane["center"], lane["width"], ego_len,
                                        center3d=lane.get("center3d"))
    else:   # 차선 밖(광장·인도): 차선 대신 자차 정면 기준
        lead_id, lead_gap = choose_lead(objects, [[-10.0, 0.0], [args.ahead, 0.0]], 2.6, ego_len)

    # 후방 차량·보행자 (자차 뒤 직선, 폭 2.6 m): 앞/뒤를 뒤집어서 같은 함수로 → 후진 화면·후진 오조작 판정
    mirrored = [dict(o, x=-o["x"], y=-o["y"]) for o in objects if o["x"] < 0]
    rear_id, rear_gap = choose_lead(mirrored, [[-10.0, 0.0], [30.0, 0.0]], 2.6, ego_len)

    # 자차 진행 방향 속도 (후진이면 음수)
    ego_fwd = vel.x * math.cos(math.radians(eyaw)) + vel.y * math.sin(math.radians(eyaw))
    lead_vf = None
    for o in objects:
        if o["id"] == lead_id:
            lead_vf = o.get("v_fwd")
    return {
        "v": 1,
        "type": "carla_scene",
        "road_grade_deg": round(packet_road_grade, 2),
        "scene_frame": "ego_local_3d_v1",
        "seq": seq,
        "map": world_map.name.split("/")[-1],
        "ego": {
            "id": ego.id,
            "z": round(tf.location.z, 2),
            "pitch": round(tf.rotation.pitch, 2),
            "roll": round(tf.rotation.roll, 2),
            "type_id": ego.type_id,
            "length": round(ego_len, 2),
            "width": round(ego_wid, 2),
            "speed_kmh": round(speed, 2),
            "gear": gear,
            "throttle": throttle,
            "brake": brake,
            "steer": steer,
        },
        "lane": lane,
        "lead": {"id": lead_id, "distance": None if lead_gap is None else round(lead_gap, 2),
                 "closing": None if lead_vf is None else round(ego_fwd - lead_vf, 2)},
        "ego_v_fwd": round(ego_fwd, 2),
        "rear": {"id": rear_id, "distance": None if rear_gap is None else round(rear_gap, 2)},
        "vehicles": objects,
    }


def debug_scene_packet(packet):
    """Temporary 2 Hz sender trace; called only after a successful scene send."""
    center = (packet.get("lane") or {}).get("center3d") or []
    point = min(center, key=lambda p: abs(p[0] - 25)) if center else None
    vehicles = packet.get("vehicles") or []
    v = vehicles[0] if vehicles else {}
    local = tuple(v.get("local_" + k) for k in ("x", "y", "z")) if v else None
    print("[3DDBG] stage=bridge seq={} frame={} egoPitch={} roadGrade={} center25={} vehicleId={} vehicle0={} vehicles={}".format(
        packet.get("seq"), packet.get("scene_frame", "legacy"),
        (packet.get("ego") or {}).get("pitch"), packet["road_grade_deg"],
        point, v.get("id"), local, len(vehicles)), flush=True)


def main():
    p = argparse.ArgumentParser(description="CARLA → PACE HMI scene bridge")
    p.add_argument("--host", default="127.0.0.1", help="CARLA 서버")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--hmi-host", default="192.168.50.2", help="라즈베리파이 IP")
    p.add_argument("--hmi-port", type=int, default=5601)
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--role-name", default="hero,ego,ego_vehicle")
    p.add_argument("--actor-id", type=int, default=None)
    p.add_argument("--ahead", type=float, default=80.0)
    p.add_argument("--behind", type=float, default=20.0)
    p.add_argument("--side", type=float, default=20.0)
    p.add_argument("--list", action="store_true", help="차량 목록만 출력")
    p.add_argument("--no-map", action="store_true", help="실제 도로/건물 정보 보내지 않음 (자차 차선만)")
    p.add_argument("--camera", choices=["real", "clean", "off"], default="real",
                   help="HMI 가운데 카드에 CARLA 실제 카메라 영상 보냄 (기본 real, R 기어면 후방 카메라). off: 영상 없이 3D 화면만")
    p.add_argument("--cam-port", type=int, default=5602)
    p.add_argument("--cam-width", type=int, default=640)
    p.add_argument("--cam-height", type=int, default=360)
    p.add_argument("--cam-fps", type=float, default=12.0)
    p.add_argument("--cam-fov", type=float, default=75.0, help="카메라 화각 (도)")
    p.add_argument("--cam-back", type=float, default=4.0, help="자차 뒤 범퍼에서 카메라까지 거리 (m)")
    p.add_argument("--cam-up", type=float, default=2.6, help="자차 지붕 위 카메라 높이 (m)")
    p.add_argument("--cam-quality", type=int, default=72, help="JPEG 품질 (네트워크 느리면 낮춤)")
    args = p.parse_args()

    try:
        import carla
    except ImportError:
        print("carla 모듈 없음: pip install carla==0.9.16 (CARLA 서버 버전과 동일하게)")
        return 1

    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    try:
        world = client.get_world()
    except RuntimeError as error:
        print("[bridge] CARLA 서버 연결 실패 ({}:{}): {}".format(args.host, args.port, error))
        return 1

    if args.list:
        for v in world.get_actors().filter("vehicle.*"):
            print(v.id, v.type_id, "role_name=" + v.attributes.get("role_name", ""))
        return 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.hmi_host, args.hmi_port)
    period = 1.0 / args.rate

    print("[bridge] CARLA {}:{} → HMI {}:{}".format(args.host, args.port, *target))
    try:
        return _loop(carla, client, args, sock, target, period)
    except KeyboardInterrupt:
        print("[bridge] 종료")
        return 0


def _loop(carla, client, args, sock, target, period):
    sender = None
    camera = None
    if args.camera != "off":
        from camera_stream import FrameSender
        sender = FrameSender(args.hmi_host, args.cam_port)
        sender.start()
    state = {"camera": None}
    try:
        _run(carla, client, args, sock, target, period, sender, state)
    finally:
        # Ctrl+C 로 끝나도 CARLA 월드에 카메라가 남지 않게 정리
        if state["camera"] is not None:
            state["camera"].destroy()
            print("[camera] 카메라 제거")
        if state.get("collision_sensor") is not None:
            try:
                state["collision_sensor"].stop()
                state["collision_sensor"].destroy()
            except Exception:
                pass
        if sender is not None:
            sender.stop()


def _make_camera(carla, world, ego, sender, args):
    from camera_stream import ChaseCamera
    try:
        cam = ChaseCamera(carla, world, ego, sender, mode=args.camera, width=args.cam_width,
                          height=args.cam_height, rate=args.cam_fps, quality=args.cam_quality,
                          fov=args.cam_fov, back=args.cam_back, up=args.cam_up)
        print("[camera] 추격 카메라 부착 ({}x{}, {} 모드)".format(args.cam_width, args.cam_height, args.camera))
        return cam
    except Exception as error:      # numpy/pillow 없음 등
        print("[camera] 카메라 생성 실패 → 영상 없이 계속:", error)
        return None


def radar_info(carla, world, ego, reverse):
    """레이더 방식 장애물 1개 (Safety 와 같은 선택). 없으면 None"""
    try:
        r = find_obstacle_for_safety(carla, world, ego, max_distance=50.0, reverse=reverse)
    except RuntimeError:
        return None
    if r is None:
        return None
    return {"id": r["obstacle_id"], "distance": round(r["distance_m"], 2),
            "closing": round(r["closing_speed_mps"], 2),
            "ttc": None if r["ttc_s"] is None else round(r["ttc_s"], 2),
            "kind": r["kind"], "label": r["label"], "dir": r["direction"]}


HIT_DIRECTIONS = ("전방", "전방 우측", "우측", "후방 우측", "후방", "후방 좌측", "좌측", "전방 좌측")


def hit_direction(yaw_deg, vx, vy):
    """자차 yaw 기준 월드 벡터 (vx, vy) 의 8 방향 (CARLA 왼손 좌표: 오른쪽 = yaw + 90)"""
    if math.hypot(vx, vy) < 1e-6:
        return ""
    ang = math.degrees(math.atan2(vy, vx)) - yaw_deg      # 0 = 앞, +90 = 오른쪽
    return HIT_DIRECTIONS[int(((ang + 22.5) % 360) // 45)]


def _attach_collision(carla, world, ego, state):
    """충돌 센서: 벽·차·사람에 부딪히면 HMI 에 알림"""
    try:
        bp = world.get_blueprint_library().find("sensor.other.collision")
        sensor = world.spawn_actor(bp, carla.Transform(), attach_to=ego)
    except Exception as error:
        print("[bridge] 충돌 센서 생성 실패:", error)
        return None

    def on_hit(event):
        other = event.other_actor
        name = other.type_id if other is not None else "static"
        if name in ("", "static.static"):
            name = "static"
        direction = ""
        try:
            tf = ego.get_transform()
            vx = vy = 0.0
            if other is not None and other.type_id.startswith(("vehicle.", "walker.")):
                lo = other.get_location()
                vx, vy = lo.x - tf.location.x, lo.y - tf.location.y
            if math.hypot(vx, vy) < 1e-3:
                imp = event.normal_impulse            # 자차를 밀어낸 힘 → 반대쪽이 부딪힌 방향
                vx, vy = -imp.x, -imp.y
            direction = hit_direction(tf.rotation.yaw, vx, vy)
        except Exception:
            pass
        n = state.get("collision", {}).get("n", 0) + 1
        state["collision"] = {"n": n, "t": time.monotonic(), "with": name, "dir": direction}
        print("[bridge] 충돌 #{}: {} ({})".format(n, name, direction or "방향 모름"), flush=True)

    sensor.listen(on_hit)
    return sensor


def _send_map(sock, target, mapc, anchor, version):
    from map_stream import map_packets
    lanes, boxes = mapc.crop(anchor)
    for pkt in map_packets(version, lanes, boxes):
        try:
            sock.sendto(json.dumps(pkt, separators=(",", ":")).encode("utf-8"), target)
        except OSError as error:
            print("[map] 송신 실패:", error)
    return len(lanes), len(boxes)


def _run(carla, client, args, sock, target, period, sender, state):
    from map_stream import MapCache, front_static_gap, map_xf
    camera = None
    mapc = None
    anchor = None
    map_ver = 0
    map_sent = 0.0
    map_counts = (0, 0)
    seq = 0
    ego = None
    world_map = None
    last_map_name = None
    last_3d_debug = float("-inf")
    last_print = 0.0
    world = None
    last_world_check = 0.0
    last_hero_check = 0.0
    loops = 0
    while True:
        t0 = time.monotonic()
        try:
            # get_world() 는 서버 왕복이라 매번 부르면 느려짐 → 3초마다만 확인 (맵 변경 감지용)
            if world is None or t0 - last_world_check > 3.0:
                world = client.get_world()
                last_world_check = t0
            if world_map is None or world.id != last_map_name:     # 맵 변경(새 에피소드) 시에만 다시 받음
                world_map = world.get_map()
                last_map_name = world.id
                ego = None
                mapc = None
                anchor = None
            # 브리지를 manual_control 보다 먼저 켰으면 아무 차나 잡았을 수 있음 → 2초마다 hero 차 다시 찾기
            if ego is not None and ego.is_alive and t0 - last_hero_check > 2.0 and args.actor_id is None:
                last_hero_check = t0
                names = [n.strip() for n in args.role_name.split(",") if n.strip()]
                if ego.attributes.get("role_name", "") not in names:
                    hero = find_ego(world, args)
                    if hero is not None and hero.id != ego.id and hero.attributes.get("role_name", "") in names:
                        print("[bridge] 운전 차량(role_name={}) 발견 → 자차 변경".format(hero.attributes.get("role_name")))
                        ego = None
            if ego is None or not ego.is_alive:
                if camera is not None:
                    camera.destroy()
                    camera = None
                ego = find_ego(world, args)
                if ego is None:
                    if t0 - last_print > 2:
                        print("[bridge] 자차를 찾지 못함 (차량 없음). 대기 중...")
                        last_print = t0
                    time.sleep(0.5)
                    continue
                print("[bridge] 자차: id={} {} role_name={}".format(
                    ego.id, ego.type_id, ego.attributes.get("role_name", "")))
                if sender is not None:
                    camera = _make_camera(carla, world, ego, sender, args)
                    state["camera"] = camera
                if state.get("collision_sensor") is not None:
                    try:
                        state["collision_sensor"].stop()
                        state["collision_sensor"].destroy()
                    except Exception:
                        pass
                state["collision_sensor"] = _attach_collision(carla, world, ego, state)
                if mapc is None and not args.no_map:
                    t_map = time.monotonic()
                    mapc = MapCache.from_world(carla, world, world_map)
                    print("[map] 도로 차선 {}개, 건물/벽/물체 {}개 읽음 ({:.1f} s)".format(
                        len(mapc.lanes), len(mapc.boxes), time.monotonic() - t_map))

            packet = build_packet(carla, world, world_map, ego, args, seq)
            if find_obstacle_for_safety is not None:
                packet["radar"] = radar_info(carla, world, ego, packet["ego"].get("gear") == "R")

            # 실제 도로/건물: 자차가 40 m 이상 움직이면 새로 잘라 보내고, 2 초마다 재전송 (UDP 손실 대비)
            if mapc is not None:
                etf = ego.get_transform()
                pose = (etf.location.x, etf.location.y, etf.rotation.yaw)
                if anchor is None or math.hypot(pose[0] - anchor[0], pose[1] - anchor[1]) > 40.0:
                    anchor = pose
                    map_ver += 1
                    map_sent = 0.0
                if t0 - map_sent > 2.0:
                    map_counts = _send_map(sock, target, mapc, anchor, map_ver)
                    map_sent = t0
                packet["map_ver"] = map_ver
                packet["map_xf"] = map_xf(anchor, pose)
                # 전방 정적 장애물 (벽·건물·기둥): 차선 위면 차선을 따라, 아니면 정면으로
                if packet["ego"].get("gear") != "R":
                    lane = packet["lane"]
                    gap_s = front_static_gap(mapc.boxes, pose, packet["ego"]["length"],
                                             center=lane["center"] if lane.get("on_road", True) else None,
                                             ego_z=etf.location.z)
                    packet["front_static"] = None if gap_s is None else round(gap_s, 2)
                # 후방 정적 장애물 (후진 화면·후진 오조작 판정용): 자차 뒤쪽 직선 경로
                gap_r = front_static_gap(mapc.boxes, (pose[0], pose[1], pose[2] + 180.0),
                                         packet["ego"]["length"], ahead=20.0, ego_z=etf.location.z)
                packet["rear_static"] = None if gap_r is None else round(gap_r, 2)
            col = state.get("collision")
            if col:
                packet["collision"] = {"n": col["n"], "age": round(t0 - col["t"], 2), "with": col["with"],
                                       "dir": col.get("dir", "")}

            sock.sendto(json.dumps(packet, separators=(",", ":")).encode("utf-8"), target)
            if t0 - last_3d_debug >= 0.5:
                debug_scene_packet(packet)
                last_3d_debug = t0
            seq += 1
            if camera is not None:
                camera.set_overlay_source(overlay_source(carla, world, world_map, ego, packet))
                camera.set_reverse(packet["ego"].get("gear") == "R")

            loops += 1
            if t0 - last_print > 2:
                cam_txt = ""
                if sender is not None:
                    cam_txt = "  camera={} sent={}".format("연결" if sender.connected else "대기", sender.sent)
                hz = loops / max(1e-3, t0 - last_print) if last_print else 0.0
                print("[bridge] {:5.1f} km/h  lead={}  objects={}  도로 {} 물체 {}  송신 {:4.1f} Hz{}".format(
                    packet["ego"]["speed_kmh"], packet["lead"]["distance"], len(packet["vehicles"]),
                    map_counts[0], map_counts[1], hz, cam_txt))
                if last_print and hz < 8:
                    print("[bridge]   ※ 송신이 느림: CARLA 가 무거운 상태. --camera off 또는 CARLA 를 -quality-level=Low 로 실행")
                last_print = t0
                loops = 0

        except RuntimeError as error:      # CARLA 연결 끊김 / 타임아웃
            print("[bridge] CARLA 오류:", error)
            if camera is not None:
                camera.destroy()
                camera = None
            ego = None
            world = None
            time.sleep(1.0)
            continue

        time.sleep(max(0.0, period - (time.monotonic() - t0)))


if __name__ == "__main__":
    sys.exit(main())
