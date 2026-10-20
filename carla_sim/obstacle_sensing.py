"""
Safety ECU 로 보내는 장애물 탐지 (CARLA PC) — 근거리 레이더 방식

- 차선 정보는 쓰지 않음. 자차 둘레 반경(기본 50 m) 안의 물체를 모든 방향에서 봄
- 물체: 차량, 보행자, 정적 물체(건물·벽·펜스·기둥·가드레일·정적 소품 등, CARLA 환경 물체 박스)
- 높이: 자차 높이 범위(바닥~지붕)와 겹치는 물체만 → 다리 위에서 다리 아래 차량, 고가 아래 물체 제외
        다리 상판 자체(Bridge)는 장애물로 보지 않음
- 거리: 자차 박스 ↔ 물체 박스 사이 최단 거리 (범퍼 거리, 옆면·모서리 포함)
- 접근 속도: 두 박스를 잇는 방향으로의 상대 속도 (+ = 가까워짐)

Safety ECU 에 보낼 1개 선택 (receiver_v2 형식 그대로: distance / closing / ttc / obstacle_id)
  1. 어느 방향이든 다가오는 물체 중 TTC 가 가장 짧은 것 (TTC ≤ 10 s)
  2. 없으면 진행 방향(D: 앞, R: 뒤) 직선 통로 안에서 가장 가까운 것 (앞차 따라가기)
  3. 없으면 1 m 안에 붙어 있는 물체 (접촉 직전)
정보용 키: kind, label, direction(전방/후방/좌측/우측/전방 좌측 …), front_gap, rear_gap
"""

import math

STATIC_OBSTACLE_ID_BASE = 800000

STATIC_LABEL_NAMES = ("Buildings", "Walls", "Fences", "Poles", "GuardRail", "Static", "TrafficLight",
                      "TrafficSigns", "Dynamic", "Other")
NOT_OBSTACLE_LABEL_NAMES = ("Bridge", "Roads", "Sidewalks", "RoadLines", "Ground", "Terrain", "Water",
                            "Sky", "Vegetation", "RailTrack")

TTC_CONSIDER_S = 10.0
CONTACT_GAP_M = 1.0
PATH_HALF_WIDTH_M = 1.6

DIRECTIONS_8 = ("전방", "전방 우측", "우측", "후방 우측", "후방", "후방 좌측", "좌측", "전방 좌측")


# =====================================================================
# 2D 박스 기하 (carla 없이 테스트 가능)
# =====================================================================
def box_corners(cx, cy, yaw_deg, half_l, half_w):
    r = math.radians(yaw_deg)
    c, s = math.cos(r), math.sin(r)
    out = []
    for dx, dy in ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)):
        out.append((cx + dx * c - dy * s, cy + dx * s + dy * c))
    return out


def _seg_point(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    q = (ax + dx * t, ay + dy * t)
    return math.hypot(p[0] - q[0], p[1] - q[1]), q


def _inside(p, poly):
    sign = None
    for a, b in zip(poly, poly[1:] + poly[:1]):
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        s = cross >= 0
        if sign is None:
            sign = s
        elif s != sign:
            return False
    return True


def poly_gap(pa, pb):
    """두 볼록 사각형 사이 최단 거리와 (pa 위 점, pb 위 점). 겹치면 0"""
    if any(_inside(p, pb) for p in pa) or any(_inside(p, pa) for p in pb):
        ca = (sum(p[0] for p in pa) / 4, sum(p[1] for p in pa) / 4)
        cb = (sum(p[0] for p in pb) / 4, sum(p[1] for p in pb) / 4)
        return 0.0, ca, cb
    best = (float("inf"), None, None)
    for p in pa:
        for a, b in zip(pb, pb[1:] + pb[:1]):
            d, q = _seg_point(p, a, b)
            if d < best[0]:
                best = (d, p, q)
    for p in pb:
        for a, b in zip(pa, pa[1:] + pa[:1]):
            d, q = _seg_point(p, a, b)
            if d < best[0]:
                best = (d, q, p)
    return best


def direction_name(fx, fy, rx, ry, vx, vy):
    """자차 앞(fx,fy)·오른쪽(rx,ry) 단위벡터 기준 벡터 (vx,vy) 의 8 방향 이름"""
    ang = math.degrees(math.atan2(vx * rx + vy * ry, vx * fx + vy * fy))    # 0 = 앞, +90 = 오른쪽
    return DIRECTIONS_8[int(((ang + 22.5) % 360) // 45)]


def select_target(cands):
    """
    cands: [{"gap", "closing", "in_path", ...}]
    1) 다가오는 것 중 TTC 최소  2) 진행 경로 안 최근접  3) 1 m 안 최근접
    """
    closing = [c for c in cands if c["closing"] > 0.3 and c["gap"] / c["closing"] <= TTC_CONSIDER_S]
    if closing:
        return min(closing, key=lambda c: (c["gap"] / c["closing"], c["gap"]))
    path = [c for c in cands if c["in_path"]]
    if path:
        return min(path, key=lambda c: c["gap"])
    near = [c for c in cands if c["gap"] <= CONTACT_GAP_M]
    if near:
        return min(near, key=lambda c: c["gap"])
    return None


# =====================================================================
# CARLA
# =====================================================================
class _StaticCache:
    """환경 물체 박스를 맵마다 1번 읽어서 20 m 격자에 넣어 둠"""

    def __init__(self):
        self.world_id = None
        self.grid = {}
        self.cell = 20.0

    def get(self, carla, world):
        if self.world_id == world.id:
            return self
        self.world_id = world.id
        self.grid = {}
        for name in STATIC_LABEL_NAMES:
            lab = getattr(carla.CityObjectLabel, name, None)
            if lab is None:
                continue
            try:
                objs = world.get_environment_objects(lab)
            except Exception:
                continue
            for o in objs:
                bb = o.bounding_box
                e = bb.extent
                if e.x <= 0.03 or e.y <= 0.03 or max(e.x, e.y) > 120:
                    continue
                item = {"id": static_obstacle_id(o, _label_int(lab)) if getattr(o, "id", None) is not None else STATIC_OBSTACLE_ID_BASE + _label_int(lab), "label": name,
                        "x": bb.location.x, "y": bb.location.y, "yaw": bb.rotation.yaw,
                        "hl": e.x, "hw": e.y, "z0": bb.location.z - e.z, "z1": bb.location.z + e.z,
                        "r": math.hypot(e.x, e.y)}
                key = (int(item["x"] // self.cell), int(item["y"] // self.cell))
                self.grid.setdefault(key, []).append(item)
        return self

    def near(self, x, y, radius):
        n = int(radius // self.cell) + 2
        cx, cy = int(x // self.cell), int(y // self.cell)
        for i in range(cx - n, cx + n + 1):
            for j in range(cy - n, cy + n + 1):
                for it in self.grid.get((i, j), ()):
                    yield it


_STATIC = _StaticCache()


def _label_int(lab):
    try:
        return int(lab)
    except (TypeError, ValueError):
        return 0


def _bbox_world(actor):
    tf = actor.get_transform()
    bb = actor.bounding_box
    c = tf.transform(bb.location) if hasattr(tf, "transform") else tf.location
    return c, tf.rotation.yaw, bb.extent


def scan(carla, world, ego, radius=50.0, reverse=False, statics=None):
    """
    자차 주변 반경 안 물체 목록 (모든 방향).
    반환: (후보 목록, 자차 정보)
    """
    c, yaw, e = _bbox_world(ego)
    ground, top = c.z - e.z, c.z + e.z
    ego_poly = box_corners(c.x, c.y, yaw, e.x, e.y)
    r = math.radians(yaw)
    fx, fy = math.cos(r), math.sin(r)
    rx, ry = -fy, fx                                 # CARLA(왼손 좌표계) 오른쪽
    ev = ego.get_velocity()
    move = -1.0 if reverse else 1.0

    def height_ok(z0, z1):
        return z1 >= ground + 0.15 and z0 <= top + 0.3

    def make(cand_id, kind, label, poly, vx, vy, actor=None):
        gap, pe, po = poly_gap(ego_poly, poly)
        if gap > radius:
            return None
        nx, ny = po[0] - pe[0], po[1] - pe[1]
        L = math.hypot(nx, ny)
        if L < 1e-6:                                   # 겹침: 중심 방향
            cx = sum(p[0] for p in poly) / 4 - c.x
            cy = sum(p[1] for p in poly) / 4 - c.y
            L = math.hypot(cx, cy) or 1.0
            nx, ny = cx, cy
        nx, ny = nx / L, ny / L
        closing = (ev.x - vx) * nx + (ev.y - vy) * ny
        # 진행 방향 통로 (D: 앞, R: 뒤) — 물체 박스 어느 점이라도 통로 안이면
        in_path = False
        for (px, py) in poly + [((poly[0][0] + poly[2][0]) / 2, (poly[0][1] + poly[2][1]) / 2)]:
            lx = ((px - c.x) * fx + (py - c.y) * fy) * move
            ly = (px - c.x) * rx + (py - c.y) * ry
            if lx > e.x and abs(ly) <= PATH_HALF_WIDTH_M:
                in_path = True
                break
        return {"id": cand_id, "kind": kind, "label": label, "gap": gap, "closing": closing,
                "in_path": in_path, "dir": direction_name(fx, fy, rx, ry, nx, ny),
                "fwd": ((po[0] - c.x) * fx + (po[1] - c.y) * fy), "actor": actor}

    cands = []
    all_actors = world.get_actors()
    actors = list(all_actors.filter("vehicle.*")) + list(all_actors.filter("walker.pedestrian.*"))
    for a in actors:
        if a.id == ego.id:
            continue
        try:
            ac, ayaw, ae = _bbox_world(a)
            if math.hypot(ac.x - c.x, ac.y - c.y) > radius + 15:
                continue
            if not height_ok(ac.z - ae.z, ac.z + ae.z):
                continue
            v = a.get_velocity()
            cand = make(a.id, "walker" if a.type_id.startswith("walker") else "vehicle", a.type_id,
                        box_corners(ac.x, ac.y, ayaw, ae.x, ae.y), v.x, v.y, a)
            if cand:
                cands.append(cand)
        except RuntimeError:
            continue

    st = statics if statics is not None else _STATIC.get(carla, world)
    for it in st.near(c.x, c.y, radius + 10):
        if math.hypot(it["x"] - c.x, it["y"] - c.y) - it["r"] > radius:
            continue
        if not height_ok(it["z0"], it["z1"]):
            continue
        poly = box_corners(it["x"], it["y"], it["yaw"], it["hl"], it["hw"])
        if _inside((c.x, c.y), poly):
            continue        # 자차가 박스 안 = 곡선 가드레일 등 도로를 덮는 큰 박스 (실제 장애물 아님)
        cand = make(it["id"], "static", it["label"], poly, 0.0, 0.0)
        if cand:
            cands.append(cand)
    return cands


def find_obstacle_for_safety(carla, world, ego, world_map=None, max_distance=50.0, lateral_limit=None,
                             reverse=False, statics=None, **_ignored):
    """기존 호출 형태 유지 (world_map / lateral_limit 는 쓰지 않음). 없으면 None"""
    cands = scan(carla, world, ego, radius=max_distance, reverse=reverse, statics=statics)
    t = select_target(cands)
    if t is None:
        return None
    closing = float(t["closing"])
    gap = float(t["gap"])
    ttc = gap / closing if closing > 0.0 else None
    return {
        "actor": t["actor"],
        "obstacle_id": int(t["id"]),
        "distance_m": gap,
        "closing_speed_mps": closing,
        "ttc_s": None if ttc is None else float(ttc),
        "kind": t["kind"],
        "label": t["label"],
        "direction": t["dir"],
    }


def path_gap(cands):
    """진행 통로(scan 의 reverse 기준 앞/뒤) 안 최근접 거리, 없으면 None"""
    g = [c["gap"] for c in cands if c["in_path"]]
    return min(g) if g else None


def select_target_for_safety(cands):
    """Select the nearest obstacle in the current driving corridor."""
    path = [
        c for c in cands
        if c.get("in_path")
        and math.isfinite(float(c["gap"]))
        and math.isfinite(float(c["closing"]))
        and float(c["gap"]) >= 0.0
    ]
    if not path:
        return None

    return min(
        path,
        key=lambda c: (
            c["gap"],
            c["gap"] / c["closing"] if c["closing"] > 0.0 else math.inf,
        ),
    )


def static_obstacle_id(obj, label_id=0):
    """Return a stable ID for a CARLA environment object."""
    object_id = getattr(obj, "id", None)

    if type(object_id) is not int or object_id < 0:
        raise ValueError("Invalid environment object ID")

    return 1_000_000_000 + object_id


def find_path_obstacle_for_safety(
    carla, world, ego, max_distance=50.0, reverse=False, kinds=None
):
    """Find the selected obstacle in the driving corridor."""
    candidates = scan(
        carla,
        world,
        ego,
        radius=max_distance,
        reverse=reverse,
    )
    if kinds is not None:
        candidates = [candidate for candidate in candidates if candidate["kind"] in kinds]

    selected = select_target_for_safety(candidates)

    if selected is None:
        return None

    distance = float(selected["gap"])
    closing = float(selected["closing"])

    return {
        "obstacle_id": int(selected["id"]),
        "distance_m": distance,
        "closing_speed_mps": closing,
        "ttc_s": distance / closing if closing > 0.0 else None,
        "kind": selected["kind"],
    }
