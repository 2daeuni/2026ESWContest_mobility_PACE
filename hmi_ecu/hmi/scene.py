"""
도로 장면 데이터 생성 (QML RoadScene 이 원근 투영해서 그림)

좌표계: 자차 기준, 단위 m
  x : 앞쪽 +,  y : 오른쪽 +   (CARLA/UE 차량 좌표계와 동일)

장면 소스 우선순위
  1. CARLA  : carla_hmi_bridge 가 보낸 실제 주변 차량 / 차선 (곡선 포함)
  2. DEMO   : DEMO 주행 모드의 가상 전방 차량 (직선 도로)
  3. SENSOR : 거리 값만 있을 때 (ToF / Safety ECU) 같은 차선 정면에 앞차 1대
  4. NONE   : 자차만
"""

import math
import os

from hmi import config


ASSET_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "cars")

DEFAULT_LANE_WIDTH = 3.5
EGO_DIMS = (4.7, 1.85)          # 기본 자차 길이/폭
CAR_DIMS = (4.6, 1.85)


class SpriteLibrary:
    """assets/cars 의 PNG 를 찾아 file URL 로 돌려줌. 없으면 ""(QML 기본 도형)."""

    VIEWS = ("rear", "front", "hero")

    def __init__(self, directory=ASSET_DIR):
        self.directory = directory
        self.rescan()

    def rescan(self):
        self.files = {}
        try:
            for name in os.listdir(self.directory):
                if name.lower().endswith(".png"):
                    self.files[name[:-4]] = "file://" + os.path.join(self.directory, name)
        except OSError:
            pass

    def get(self, type_id, view):
        if type_id:
            url = self.files.get("{}_{}".format(type_id, view))
            if url:
                return url
        return self.files.get("default_{}".format(view), "")

    @property
    def count(self):
        return len(self.files)


# ---------------------------------------------------------------
# 차선 polyline
# ---------------------------------------------------------------
def straight_center(length=120.0, step=10.0):
    pts = []
    x = -8.0
    while x <= length:
        pts.append([x, 0.0])
        x += step
    return pts


def offset_polyline(center, offset):
    """center 폴리라인을 좌우로 offset(m, 오른쪽 +) 만큼 평행이동."""
    out = []
    n = len(center)
    for i in range(n):
        x0, y0 = center[max(0, i - 1)]
        x1, y1 = center[min(n - 1, i + 1)]
        dx, dy = x1 - x0, y1 - y0
        norm = math.hypot(dx, dy) or 1.0
        # 진행방향 (dx,dy) 의 오른쪽 법선 = (-dy, dx)  (x 앞, y 오른쪽 좌표계)
        nx, ny = -dy / norm, dx / norm
        out.append([round(center[i][0] + nx * offset, 3), round(center[i][1] + ny * offset, 3)])
    return out


def _height(value, default=0.0):
    """Optional rendering numbers never introduce NaN/Inf into scene transforms."""
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
        return float(value)
    return default


def interp_z(center, x):
    return interp_y([[p[0], p[2]] for p in center], x)


def interp_y(center, x):
    if x <= center[0][0]:
        return center[0][1]
    for (x0, y0), (x1, y1) in zip(center, center[1:]):
        if x0 <= x <= x1:
            return y0 if x1 == x0 else y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return center[-1][1]


def build_map_scene(lanes, boxes):
    """
    carla_tools/map_stream.py 가 보낸 실제 도로 (앵커 좌표) → 3D 화면용 묶음
      bands : 도로면 (차선마다 왼쪽/오른쪽 경계)
      white / white_dash / yellow / yellow_dash : 차선 표시 (strips: 점들 + 폭)
      boxes : 건물·벽·펜스·기둥 (높이는 화면용으로 제한)
    """
    out = {"bands": [], "white": [], "white_dash": [], "yellow": [], "yellow_dash": [], "boxes": []}
    for lane in lanes or []:
        L, R = lane.get("L") or [], lane.get("R") or []
        if len(L) < 2 or len(R) < 2:
            continue
        out["bands"].append({"L": L, "R": R})
        if lane.get("j"):
            continue                        # 교차로 안은 차선 표시 없음 (실제 도로와 동일)
        for edge, mark in ((L, lane.get("lm")), (R, lane.get("rm"))):
            kind, color = (mark or ["none", "w"])[:2]
            if kind == "none":
                continue
            key = ("yellow" if color == "y" else "white") + ("_dash" if kind == "dash" else "")
            if kind == "double":
                out[key].append({"p": offset_polyline(edge, -0.13), "w": 0.12})
                out[key].append({"p": offset_polyline(edge, 0.13), "w": 0.12})
            else:
                out[key].append({"p": edge, "w": 0.15})
    for b in boxes or []:
        h = min(float(b.get("h", 2.0)), 3.0)
        if h < 0.3:
            h = 0.6
        out["boxes"].append({"x": b["x"], "y": b["y"], "yaw": b.get("yaw", 0.0),
                             "l": max(0.2, b["l"]), "w": max(0.2, b["w"]), "h": h})
    return out


def lane_lines(center, width, lanes_left, lanes_right):
    """그릴 차선 목록. kind: ego(자차 차선 경계) | solid(도로 가장자리) | dash"""
    lines = []
    half = width / 2.0
    left_edge = -half - width * lanes_left
    right_edge = half + width * lanes_right

    lines.append({"kind": "ego", "points": offset_polyline(center, -half)})
    lines.append({"kind": "ego", "points": offset_polyline(center, half)})
    for k in range(1, lanes_left + 1):
        lines.append({"kind": "solid" if k == lanes_left else "dash",
                      "points": offset_polyline(center, -half - width * k)})
    for k in range(1, lanes_right + 1):
        lines.append({"kind": "solid" if k == lanes_right else "dash",
                      "points": offset_polyline(center, half + width * k)})
    road = {
        "left": offset_polyline(center, left_edge - 0.6),
        "right": offset_polyline(center, right_edge + 0.6),
    }
    return lines, road


# CARLA blueprint → 3D 모델 종류 (assets/models 의 car / truck / bus / bike / person)
_TRUCK_KEYS = ("carlacola", "firetruck", "ambulance", "european_hgv", "sprinter", "volkswagen.t2")
_BUS_KEYS = ("bus", "fusorosa")
_BIKE_KEYS = ("harley", "kawasaki", "yamaha", "vespa", "crossbike", "diamondback", "gazelle", "ninja", "low_rider", "century", "omafiets")


def model_category(type_id, kind="car", length=None):
    t = (type_id or "").lower()
    if kind == "walker" or t.startswith("walker"):
        return "person"
    if any(k in t for k in _BIKE_KEYS):
        return "bike"
    if any(k in t for k in _BUS_KEYS) or (length or 0) > 9.5:
        return "bus"
    if any(k in t for k in _TRUCK_KEYS) or (length or 0) > 6.2:
        return "truck"
    return "car"


def _vehicle(sprites, vid, type_id, x, y, yaw=0.0, length=None, width=None, lead=False, kind="car"):
    length = length or CAR_DIMS[0]
    width = width or CAR_DIMS[1]
    oncoming = abs(((yaw + 180) % 360) - 180) > 100
    return {
        "id": vid,
        "kind": kind,
        "type_id": type_id or "",
        "x": round(x, 3),
        "y": round(y, 3),
        "yaw": round(yaw, 1),
        "length": round(length, 2),
        "width": round(width, 2),
        "lead": lead,
        "model": model_category(type_id, kind, length),
        "sprite": "" if kind != "car" else sprites.get(type_id, "front" if oncoming else "rear"),
    }


# ---------------------------------------------------------------
# 장면 생성
# ---------------------------------------------------------------
def build_scene(sprites, carla=None, demo=None, distance_m=None):
    ego_type = config.EGO_BLUEPRINT
    ego_len, ego_wid = EGO_DIMS
    center = straight_center()
    width = DEFAULT_LANE_WIDTH
    lanes_left, lanes_right = 1, 1
    vehicles = []
    lead_gap = None
    source = "NONE"

    center3d = None
    has_map = False
    on_road = True
    map_xf = None
    if carla is not None:
        source = "CARLA"
        has_map = carla.get("map_ver") is not None and carla.get("map_xf") is not None
        map_xf = carla.get("map_xf")
        ego = carla.get("ego") or {}
        ego_type = ego.get("type_id") or ego_type
        ego_len = ego.get("length") or ego_len
        ego_wid = ego.get("width") or ego_wid
        lane = carla.get("lane") or {}
        if lane.get("center") and len(lane["center"]) >= 2:
            center = sorted(lane["center"], key=lambda c: c[0])
            # 차선 정보가 자차와 동떨어지면(도로 밖, 주차장, 스폰 직후) 직선 도로로 대체 → 화면에 도로가 항상 보이게
            if center[-1][0] < 10 or abs(interp_y(center, 0.0)) > (lane.get("width") or DEFAULT_LANE_WIDTH) * 1.5:
                center = straight_center()
        # Keep the 2D scene and all lead/distance calculations unchanged.
        # Validate the entire optional polyline, otherwise use the legacy plane.
        raw3d = lane.get("center3d")
        if isinstance(raw3d, (list, tuple)) and len(raw3d) >= 2:
            try:
                candidate = sorted([[float(c) for c in p[:3]] for p in raw3d], key=lambda p: p[0])
                if all(len(p) == 3 and all(math.isfinite(c) for c in p) for p in candidate):
                    center3d = candidate
            except (TypeError, ValueError, IndexError):
                pass
        width = lane.get("width") or width
        on_road = lane.get("on_road", True) is not False
        lanes_left = int(lane.get("lanes_left", 1))
        lanes_right = int(lane.get("lanes_right", 1))
        lead_id = (carla.get("lead") or {}).get("id")
        lead_gap = (carla.get("lead") or {}).get("distance")
        for v in carla.get("vehicles", []):
            vehicles.append(_vehicle(
                sprites, v.get("id"), v.get("type_id"), v.get("x", 0), v.get("y", 0),
                v.get("yaw", 0), v.get("length"), v.get("width"),
                lead=(lead_id is not None and v.get("id") == lead_id),
                kind=v.get("kind", "car"),
            ))
            vehicles[-1].update({field: _height(v.get(field)) for field in ("z", "pitch", "roll")})
            # Keep legacy XY/yaw for 2D drawing, visibility and risk calculation.
            # A complete optional triplet selects the full ego-local render frame.
            if all(field in v for field in ("local_x", "local_y", "local_z")):
                vehicles[-1].update({field: _height(v[field]) for field in
                                     ("local_x", "local_y", "local_z")})
            if all(field in v for field in ("local_yaw", "local_pitch", "local_roll")):
                vehicles[-1].update({field: _height(v[field]) for field in
                                     ("local_yaw", "local_pitch", "local_roll")})
            # Canonical packet names; local_* aliases remain backward compatible.
            for axis in ("yaw", "pitch", "roll"):
                key = "relative_" + axis
                alias = "local_" + axis
                if key in v or alias in v:
                    vehicles[-1][key] = _height(v.get(key, v.get(alias)))

    elif demo is not None:
        source = "DEMO"
        gap = demo["distance_m"]
        lead_gap = gap
        lx = ego_len / 2 + gap + CAR_DIMS[0] / 2
        if lx < 400:       # 400 m 보다 멀면 그리지 않음 (지평선 너머)
            vehicles.append(_vehicle(sprites, "demo-lead", config.DEMO_LEAD_BLUEPRINT, lx, 0.0, lead=True))

    elif distance_m is not None:
        source = "SENSOR"
        lead_gap = distance_m
        lx = ego_len / 2 + distance_m + CAR_DIMS[0] / 2
        vehicles.append(_vehicle(sprites, "sensor-lead", config.DEMO_LEAD_BLUEPRINT, lx, 0.0, lead=True))

    lines, road = lane_lines(center, width, lanes_left, lanes_right)
    if has_map:
        # 실제 도로/차선은 맵 데이터로 그림 → 여기서는 자차 차선 강조선만 (차선 밖이면 그것도 없음)
        lines = [l for l in lines if l["kind"] == "ego"] if on_road else []
        road = {"left": [], "right": []}
    vehicles.sort(key=lambda v: -v["x"])      # 먼 차부터 그림

    # 자차 앞범퍼 ~ 앞차 뒷범퍼 사이 위험 영역 (차선 중심선 따라)
    risk_zone = []
    lead = next((v for v in vehicles if v["lead"]), None)
    if lead is not None and on_road:
        x0 = ego_len / 2.0
        x1 = lead["x"] - lead["length"] / 2.0
        if x1 > x0 + 0.5:
            n = 12
            xs = [x0 + (x1 - x0) * i / n for i in range(n + 1)]
            half = width * 0.42
            left = [[x, interp_y(center, x) - half] for x in xs]
            right = [[x, interp_y(center, x) + half] for x in reversed(xs)]
            risk_zone = left + right

    render_center = center3d or [p[:2] + [0.0] for p in center]
    # The map protocol contains XY only. On slopes it cannot supply a surface
    # consistent with actor heights; use the sampled local lane corridor instead.
    # Even a constant grade has local_z=0. Its yaw-only map is still in a
    # different frame (XY foreshortening and missing bank), so keep it disabled.
    ego_tilt = bool(carla and any(_height((carla.get("ego") or {}).get(k)) != 0 for k in ("pitch", "roll")))
    slope_scene = (ego_tilt or bool(center3d and any(p[2] != 0 for p in center3d))
                   or any(v.get("local_z", v.get("z", 0)) != 0 for v in vehicles))
    render_lines, render_road = lane_lines([p[:2] for p in render_center], width, lanes_left, lanes_right)
    for line in render_lines:
        line["points"] = [p + [render_center[i][2]] for i, p in enumerate(line["points"])]
    for side in ("left", "right"):
        render_road[side] = [p + [render_center[i][2]] for i, p in enumerate(render_road[side])]
    local_road = center3d is not None
    if has_map and not local_road and not slope_scene:
        render_lines = [line for line in render_lines if line["kind"] == "ego"] if on_road else []
        render_road = {"left": [], "right": []}
    risk3d = [p + [interp_z(render_center, p[0])] for p in risk_zone]
    if risk_zone and lead is not None and "local_x" in lead:
        # Only the displayed polygon uses local longitudinal distance. The 2D
        # risk_zone and every lead/distance/TTC/risk decision above are unchanged.
        start, end = ego_len / 2.0, lead["local_x"] - lead["length"] / 2.0
        risk3d = []
        if end > start + 0.5:
            xs = [start + (end - start) * i / 12 for i in range(13)]
            for offset, samples in ((-width * 0.42, xs), (width * 0.42, reversed(xs))):
                risk3d += [[x, interp_y([p[:2] for p in render_center], x) + offset, interp_z(render_center, x)]
                           for x in samples]

    raw_vehicle = next((v for v in (carla.get("vehicles") or [])
                        if vehicles and v.get("id") == vehicles[0]["id"]), {}) if carla else {}

    return {
        "source": source,
        "road_grade_deg": _height(carla.get("road_grade_deg", (carla.get("lane") or {}).get("road_grade_deg")), None) if carla else None,
        "scene_frame": carla.get("scene_frame", "legacy") if carla else "none",
        "scene_seq": carla.get("seq") if carla else None,
        "local_road": local_road,
        "debug3d": {
            "center_mode": "center3d" if local_road else "legacy",
            "raw_center25": min((carla.get("lane") or {}).get("center3d") or [],
                                key=lambda p: abs(p[0] - 25), default=None) if local_road else None,
            "raw_vehicle_id": raw_vehicle.get("id"),
            "raw_vehicle_local": [raw_vehicle.get("local_" + k) for k in ("x", "y", "z")]
                                 if all("local_" + k in raw_vehicle for k in ("x", "y", "z")) else None,
            "received_vehicles": len(carla.get("vehicles") or []) if carla else 0,
            "received_local": sum(all(k in v for k in ("local_x", "local_y", "local_z"))
                                  for v in carla.get("vehicles", [])) if carla else 0,
        },
        "center3d": render_center,
        "lines3d": render_lines,
        "road_left3d": render_road["left"],
        "road_right3d": render_road["right"],
        "risk_zone3d": risk3d,
        "slope_scene": slope_scene,
        "lane_width": width,
        "center": center,
        "lines": lines,
        "road_left": road["left"],
        "road_right": road["right"],
        "vehicles": vehicles,
        "lead_gap": lead_gap,
        "risk_zone": risk_zone,
        "has_map": has_map,
        "map_ver": carla.get("map_ver") if has_map else None,
        "map_xf": map_xf,
        "on_road": on_road,
        "collided": bool(carla and ((carla.get("collision") or {}).get("age") if (carla.get("collision") or {}).get("age") is not None else 99) < 1.5),
        "ego": {
            "z": _height((carla.get("ego") or {}).get("z"), None) if carla else None,
            "pitch": _height((carla.get("ego") or {}).get("pitch")) if carla else 0.0,
            "roll": _height((carla.get("ego") or {}).get("roll")) if carla else 0.0,
            "type_id": ego_type,
            "length": ego_len,
            "width": ego_wid,
            "sprite": sprites.get(ego_type, "rear"),
            "hero": sprites.get(ego_type, "hero"),
        },
    }
