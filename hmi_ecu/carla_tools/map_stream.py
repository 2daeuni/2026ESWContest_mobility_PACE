"""
CARLA 맵(실제 도로·차선·건물/벽) → HMI 3D 화면

- 맵 전체 차선을 처음 1번만 읽어 둠 (world_map.generate_waypoints)
- 자차 주변 반경만 잘라서 "앵커 좌표계"(잘라낸 순간의 자차 기준 x 앞 / y 오른쪽, m)로 보냄
  → 자차가 40 m 이상 움직이거나 2 초가 지나면 다시 잘라서 보냄 (UDP 패킷 type "carla_map")
- 매 프레임 장면 패킷에는 "map_xf"(앵커 → 현재 자차 좌표 변환)만 넣음
  → HMI 는 도로 형상을 다시 계산하지 않고 3D 노드 위치/회전만 바꿈 (라즈베리파이 부담 적음)

변환: p_자차 = (tx, ty) + Rot(rot) · p_앵커,  Rot(θ)(x, y) = (x cosθ − y sinθ, x sinθ + y cosθ)
"""

import json
import math


def to_frame(ox, oy, oyaw_deg, px, py):
    """월드 → (ox, oy, oyaw) 기준 좌표 (x 앞, y 오른쪽). carla_hmi_bridge.to_ego 와 동일."""
    dx, dy = px - ox, py - oy
    yaw = math.radians(oyaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c + dy * s, -dx * s + dy * c


def map_xf(anchor, ego):
    """앵커 좌표 → 현재 자차 좌표 변환 파라미터. anchor / ego = (x, y, yaw_deg) 월드."""
    tx, ty = to_frame(ego[0], ego[1], ego[2], anchor[0], anchor[1])
    rot = (anchor[2] - ego[2] + 180.0) % 360.0 - 180.0
    return {"x": round(tx, 3), "y": round(ty, 3), "rot": round(rot, 3)}


def apply_xf(xf, x, y):
    th = math.radians(xf["rot"])
    c, s = math.cos(th), math.sin(th)
    return xf["x"] + x * c - y * s, xf["y"] + x * s + y * c


def marking_kind(carla, marking):
    """CARLA LaneMarking → (종류, 색) : solid / dash / double / none, w / y"""
    if marking is None:
        return "none", "w"
    T = carla.LaneMarkingType
    t = marking.type
    if t in (T.Broken, T.BrokenBroken):
        kind = "dash"
    elif t in (T.SolidSolid, T.SolidBroken, T.BrokenSolid):
        kind = "double"
    elif t == T.Solid:
        kind = "solid"
    else:
        kind = "none"
    color = "y" if getattr(marking, "color", None) == carla.LaneMarkingColor.Yellow else "w"
    return kind, color


class MapCache:
    """
    lanes: [{"L": [(x, y), ...], "R": [...], "lm": (kind, color), "rm": (kind, color), "j": junction}]  (월드 좌표)
    boxes: [{"x", "y", "yaw", "l", "w", "h"}]  (월드 좌표, 건물/벽/펜스/기둥/정적 물체)
    """

    OBSTACLE_LABELS = ("Buildings", "Walls", "Fences", "Poles", "Static", "GuardRail", "Bridge")

    def __init__(self):
        self.lanes = []
        self.boxes = []

    # ---------------- 맵 읽기 (CARLA 필요) ----------------
    @classmethod
    def from_world(cls, carla, world, world_map, step=2.0):
        mc = cls()
        groups = {}
        for wp in world_map.generate_waypoints(step):
            groups.setdefault((wp.road_id, wp.section_id, wp.lane_id), []).append(wp)
        for g in groups.values():
            g.sort(key=lambda w: w.s)
            L, R = [], []
            for w in g:
                loc = w.transform.location
                r = w.transform.get_right_vector()
                half = w.lane_width / 2.0
                L.append((loc.x - r.x * half, loc.y - r.y * half))
                R.append((loc.x + r.x * half, loc.y + r.y * half))
            mid = g[len(g) // 2]
            mc.lanes.append({
                "L": L, "R": R,
                "lm": marking_kind(carla, mid.left_lane_marking),
                "rm": marking_kind(carla, mid.right_lane_marking),
                "j": bool(mid.is_junction),
            })

        labels = []
        for name in cls.OBSTACLE_LABELS:
            lab = getattr(carla.CityObjectLabel, name, None)
            if lab is not None:
                labels.append(lab)
        for lab in labels:
            try:
                objs = world.get_environment_objects(lab)
            except Exception:
                continue
            for o in objs:
                bb = o.bounding_box
                e = bb.extent
                if e.x <= 0.05 or e.y <= 0.05 or max(e.x, e.y) > 150:
                    continue
                mc.boxes.append({
                    "x": bb.location.x, "y": bb.location.y, "yaw": bb.rotation.yaw,
                    "l": e.x * 2, "w": e.y * 2, "h": e.z * 2,
                })
        return mc

    # ---------------- 잘라내기 (carla 없이 테스트 가능) ----------------
    def crop(self, anchor, radius=90.0, step_pts=1):
        """anchor=(x, y, yaw) 주변 radius 안의 차선/물체를 앵커 좌표로"""
        ax, ay, ayaw = anchor
        r2 = radius * radius
        lanes = []
        for lane in self.lanes:
            run_l, run_r = [], []

            def flush():
                if len(run_l) >= 2:
                    lanes.append({"L": list(run_l), "R": list(run_r), "lm": list(lane["lm"]),
                                  "rm": list(lane["rm"]), "j": lane["j"]})
                run_l.clear()
                run_r.clear()

            for i, ((lx, ly), (rx, ry)) in enumerate(zip(lane["L"], lane["R"])):
                cx, cy = (lx + rx) / 2 - ax, (ly + ry) / 2 - ay
                if cx * cx + cy * cy > r2:
                    flush()
                    continue
                if step_pts > 1 and i % step_pts and i != len(lane["L"]) - 1:
                    continue
                x0, y0 = to_frame(ax, ay, ayaw, lx, ly)
                x1, y1 = to_frame(ax, ay, ayaw, rx, ry)
                run_l.append([round(x0, 2), round(y0, 2)])
                run_r.append([round(x1, 2), round(y1, 2)])
            flush()

        boxes = []
        for b in self.boxes:
            dx, dy = b["x"] - ax, b["y"] - ay
            reach = radius + max(b["l"], b["w"]) / 2
            if dx * dx + dy * dy > reach * reach:
                continue
            x, y = to_frame(ax, ay, ayaw, b["x"], b["y"])
            boxes.append({"x": round(x, 2), "y": round(y, 2),
                          "yaw": round((b["yaw"] - ayaw + 180) % 360 - 180, 1),
                          "l": round(b["l"], 2), "w": round(b["w"], 2), "h": round(b["h"], 2)})
        boxes.sort(key=lambda b: b["x"] ** 2 + b["y"] ** 2)
        return lanes, boxes[:300]


def map_packets(version, lanes, boxes, max_bytes=16000):
    """UDP 한 개에 안 들어가면 여러 조각으로 (HMI 가 version 기준으로 합침)"""
    parts = []
    cur_l, cur_b, size = [], [], 0
    items = [("l", x) for x in lanes] + [("b", x) for x in boxes]
    for kind, item in items:
        n = len(json.dumps(item, separators=(",", ":")))
        if cur_l or cur_b:
            if size + n > max_bytes:
                parts.append((cur_l, cur_b))
                cur_l, cur_b, size = [], [], 0
        (cur_l if kind == "l" else cur_b).append(item)
        size += n
    parts.append((cur_l, cur_b))
    out = []
    for i, (l, b) in enumerate(parts):
        out.append({"v": 1, "type": "carla_map", "ver": version, "part": i, "parts": len(parts),
                    "lanes": l, "boxes": b})
    return out


# =====================================================================
# 전방 장애물 (건물·벽·기둥 등 정적 물체) 거리 — 차는 choose_lead 가 따로 계산
# =====================================================================
def _interp(center, x):
    if not center:
        return 0.0
    if x <= center[0][0]:
        return center[0][1]
    for (x0, y0), (x1, y1) in zip(center, center[1:]):
        if x0 <= x <= x1:
            return y0 if x1 == x0 else y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return center[-1][1]


def front_static_gap(boxes, pose, ego_len, center=None, half_width=1.15, ahead=60.0, step=0.5):
    """
    자차 진행 경로(차선 위면 차선 중심선을 따라, 아니면 정면 직선) 폭 2.3 m 안에 들어오는
    가장 가까운 정적 물체까지의 범퍼 거리 (m). 없으면 None.
    boxes: MapCache.boxes (월드), pose: (x, y, yaw) 월드
    """
    ex, ey, eyaw = pose
    front = ego_len / 2.0
    best = None
    lim2 = (ahead + 30.0) ** 2
    for b in boxes:
        dx, dy = b["x"] - ex, b["y"] - ey
        if dx * dx + dy * dy > lim2 + (b["l"] + b["w"]) ** 2:
            continue
        cx, cy = to_frame(ex, ey, eyaw, b["x"], b["y"])
        if cx + max(b["l"], b["w"]) < front:
            continue
        # 물체 둘레를 step 간격으로 샘플링해서 경로 안에 들어오는 가장 앞쪽 점
        yaw = math.radians(b["yaw"] - eyaw)
        c, s = math.cos(yaw), math.sin(yaw)
        hl, hw = b["l"] / 2.0, b["w"] / 2.0
        corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
        pts = [(cx + px * c - py * s, cy + px * s + py * c) for px, py in corners]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
            n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
            for k in range(n + 1):
                x = x0 + (x1 - x0) * k / n
                if x < front - 0.3 or x > front + ahead:
                    continue
                y = y0 + (y1 - y0) * k / n
                mid = _interp(center, x) if center else 0.0
                if abs(y - mid) <= half_width:
                    gap = max(0.0, x - front)
                    if best is None or gap < best:
                        best = gap
    return best
