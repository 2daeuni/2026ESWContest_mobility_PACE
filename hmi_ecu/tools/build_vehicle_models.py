#!/usr/bin/env python3
"""
3D 차량 모델(.glb) → HMI 용 Qt Quick 3D 메시(.mesh) 변환 (개발 PC 에서 1회 실행, 결과물은 assets/models 에 커밋)

- 실내/번호판/엔진 제거, 재질별로 묶어서 HMI 가 직접 색을 입힐 수 있게 함 (테슬라 화면처럼 단색 + 반사)
- 자차(ego): 외장 전체 (도장 / 투톤 / 유리 / 트림 / 타이어 / 휠 / 헤드램프 / 브레이크등 / 방향지시등)
- 주변 차(traffic): 가볍게 줄여서 3개 묶음 (차체 / 유리+트림 / 바퀴) → 라즈베리파이에서 여러 대 그려도 가벼움
- 트럭 / 버스 / 이륜차 / 보행자: 단순한 매끈한 형상 (테슬라 화면의 회색 형상과 같은 방식)

필요: pip install trimesh fast-simplification numpy, PySide6 (balsam 사용)

  python3 tools/build_vehicle_models.py --car path/to/CarConcept.glb
  python3 tools/build_vehicle_models.py --car my_ego.glb --traffic my_sedan.glb

기본 모델: Khronos glTF Sample Assets "CarConcept" (© 2024 Darmstadt Graphics Group GmbH, CC BY 4.0)
  https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/CarConcept
다른 .glb 를 쓰려면 아래 GROUP_RULES 의 재질 이름 규칙만 맞추면 됨 (못 맞추면 전부 'paint' 로 들어감)
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "assets", "models")

# (재질/노드 이름에 포함된 문자열, 그룹) - 위에서부터 먼저 맞는 것
GROUP_RULES = [
    ("license", None), ("interior", None), ("engine", None), ("floormat", None), ("dashboard", None),
    ("panel sides", None), ("hardware", None), ("seat", None), ("steering", None), ("pedal", None),
    ("brakelight", "brakelight"), ("taillight", "brakelight"),
    ("signallight", "signal"), ("turnsignal", "signal"),
    ("headlight", "headlight"),
    ("gasket", "trim"), ("wiper", "trim"),
    ("glass", "glass"), ("window", "glass"), ("windshield", "glass"),
    ("tiretread", "tire"), ("tireside", "tire"), ("tire", "tire"),
    ("rim", "rim"), ("disc", "rim"), ("brake", "rim"),
    ("paint 2", "accent"), ("color2", "accent"),
    ("paint", "paint"), ("color1", "paint"), ("hood", "paint"), ("pillar", "paint"),
    ("mirror", "trim"), ("mechanical", "trim"), ("underside", "trim"), ("gasket", "trim"), ("wiper", "trim"),
]

EGO_BUDGET = {   # 그룹별 최대 삼각형 수 (넘는 그룹만 줄임, 나머지는 원본 그대로)
    "paint": 60000, "accent": 30000, "glass": 12000, "trim": 6000, "tire": 9000, "rim": 12000,
    "headlight": 1500, "brakelight": 1500, "signal": 600,
}
TRAFFIC_GROUPS = {"body": ("paint", "accent"), "dark": ("glass", "trim", "headlight", "brakelight", "signal"),
                  "wheel": ("tire", "rim")}
TRAFFIC_BUDGET = {"body": 26000, "dark": 7000, "wheel": 2400}


def classify(node_name, mat_name):
    key = "{} {}".format(mat_name or "", node_name or "").lower()
    # 노드 이름이 실내면 재질과 상관없이 제외
    if (node_name or "").lower().startswith("interior"):
        return None
    for needle, group in GROUP_RULES:
        if needle in key:
            return group
    return "trim"


def load_groups(path):
    sc = trimesh.load(path, force="scene")
    groups = {}
    for node in sc.graph.nodes_geometry:
        T, gname = sc.graph[node]
        geom = sc.geometry[gname]
        if not isinstance(geom, trimesh.Trimesh) or len(geom.faces) == 0:
            continue
        mat = getattr(getattr(geom, "visual", None), "material", None)
        group = classify(node, getattr(mat, "name", ""))
        if group is None:
            continue
        m = trimesh.Trimesh(vertices=geom.vertices.copy(), faces=geom.faces.copy(),
                            vertex_normals=geom.vertex_normals.copy(), process=False)   # 원본 법선 유지 (매끈한 차체)
        m.apply_transform(T)
        groups.setdefault(group, []).append(m)
    return {k: trimesh.util.concatenate(v) for k, v in groups.items()}


def normalize(groups):
    """앞 = -Z, 지면 y=0, 중심 x=z=0. 길이/폭 반환 (미러 제외 폭은 바퀴 기준)."""
    allm = trimesh.util.concatenate(list(groups.values()))
    lo, hi = allm.bounds
    center = (lo + hi) / 2
    # 전면 판단: 헤드램프가 있는 쪽이 앞
    front_sign = 1.0
    if "headlight" in groups:
        front_sign = 1.0 if groups["headlight"].bounds.mean(0)[2] > center[2] else -1.0
    ext = hi - lo
    if ext[0] > ext[2]:      # 길이 방향이 X 인 모델 → Z 로 돌림
        R = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
        for m in groups.values():
            m.apply_transform(R)
        return normalize(groups)
    M = trimesh.transformations.translation_matrix([-center[0], -lo[1], -center[2]])
    if front_sign > 0:       # 앞이 +Z → 180° 돌려서 -Z
        M = trimesh.transformations.rotation_matrix(np.pi, [0, 1, 0]) @ M
    for m in groups.values():
        m.apply_transform(M)
    allm = trimesh.util.concatenate(list(groups.values()))
    lo, hi = allm.bounds
    ref = groups["tire"] if "tire" in groups else allm      # 미러 제외 폭: 바퀴 바깥 기준
    width = ref.bounds[1][0] - ref.bounds[0][0]
    return {"length": float(hi[2] - lo[2]), "width": float(width), "height": float(hi[1] - lo[1])}


def decimate(mesh, max_faces):
    if len(mesh.faces) <= max_faces:
        return mesh                       # 원본 (법선 포함) 그대로
    mesh = mesh.copy()
    mesh.merge_vertices()
    try:
        import fast_simplification
        ratio = 1.0 - max_faces / len(mesh.faces)
        v, f = fast_simplification.simplify(mesh.vertices.astype(np.float32), mesh.faces.astype(np.int32), target_reduction=ratio)
        out = trimesh.Trimesh(vertices=v, faces=f, process=True)
        trimesh.repair.fix_normals(out)
        return smooth_normals(out)
    except ImportError:
        print("  fast-simplification 없음 → 원본 유지")
        return mesh


def smooth_normals(mesh, angle_deg=40):
    # 날카로운 모서리는 유지하면서 부드러운 음영 (Tesla 화면의 매끈한 차체)
    return trimesh.graph.smooth_shade(mesh, angle=np.radians(angle_deg)) if hasattr(trimesh.graph, "smooth_shade") else mesh


# ----------------------------------------------------------------------
# 단순 형상 (트럭, 버스, 이륜차, 보행자)
# ----------------------------------------------------------------------
def rounded_box(size, center, radius=0.12):
    """모서리가 둥근 상자: 세분화한 상자 꼭짓점을 안쪽 상자에서 radius 만큼 밀어냄"""
    size = np.array(size, dtype=float)
    radius = min(radius, size.min() / 2 - 1e-3)
    box = trimesh.creation.box(extents=size)
    for _ in range(4):
        box = box.subdivide()
    v = box.vertices
    inner = size / 2 - radius
    clamped = np.clip(v, -inner, inner)
    d = v - clamped
    n = np.linalg.norm(d, axis=1, keepdims=True)
    n[n < 1e-9] = 1.0
    v = clamped + d / n * radius
    return trimesh.Trimesh(vertices=v + np.array(center), faces=box.faces, process=True)


def wheels(positions, radius, width):
    out = []
    for x, z in positions:
        c = trimesh.creation.cylinder(radius=radius, height=width, sections=24)
        c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1]))
        c.apply_translation([x, radius, z])
        out.append(c)
    return trimesh.util.concatenate(out)


def make_truck():
    L, W = 7.5, 2.4
    cab = rounded_box([W, 2.5, 2.0], [0, 0.45 + 1.25, -L / 2 + 1.0], 0.22)
    cargo = rounded_box([W + 0.05, 2.9, L - 2.3], [0, 0.55 + 1.45, -L / 2 + 2.15 + (L - 2.3) / 2], 0.08)
    glass = rounded_box([W - 0.2, 0.8, 0.12], [0, 2.25, -L / 2 + 0.02], 0.04)
    wh = wheels([(sx * (W / 2 - 0.25), z) for sx in (-1, 1) for z in (-L / 2 + 1.1, L / 2 - 1.5, L / 2 - 0.5)], 0.5, 0.36)
    return {"body": trimesh.util.concatenate([cab, cargo]), "dark": glass, "wheel": wh}, {"length": L, "width": W}


def make_bus():
    L, W = 11.5, 2.5
    body = rounded_box([W, 2.9, L], [0, 0.35 + 1.45, 0], 0.3)
    side = []
    for sx in (-1, 1):
        side.append(rounded_box([0.06, 0.9, L - 1.6], [sx * (W / 2 + 0.005), 2.2, 0.3], 0.03))
    front = rounded_box([W - 0.3, 1.3, 0.08], [0, 2.0, -L / 2 - 0.005], 0.05)
    wh = wheels([(sx * (W / 2 - 0.3), z) for sx in (-1, 1) for z in (-L / 2 + 2.4, L / 2 - 2.6)], 0.5, 0.36)
    return {"body": body, "dark": trimesh.util.concatenate(side + [front]), "wheel": wh}, {"length": L, "width": W}


def make_bike():
    L, W = 2.1, 0.8
    frame = rounded_box([0.32, 0.45, 1.3], [0, 0.75, 0], 0.12)
    rider_body = trimesh.creation.capsule(height=0.45, radius=0.2)
    rider_body.apply_translation([0, 1.35, 0.15])
    head = trimesh.creation.icosphere(subdivisions=2, radius=0.15)
    head.apply_translation([0, 1.82, 0.05])
    wh = wheels([(0, -0.72), (0, 0.72)], 0.33, 0.14)
    return {"body": trimesh.util.concatenate([frame, rider_body, head]), "wheel": wh}, {"length": L, "width": W}


def make_person():
    # 서 있는 사람 (몸통 + 머리 + 다리 2개 + 팔 2개), 키 약 1.75 m
    def cap(r, length, center):
        c = trimesh.creation.capsule(height=length, radius=r, count=[12, 12])   # Z 방향
        c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
        c.apply_translation(center)
        return c
    parts = [cap(0.17, 0.42, [0, 1.17, 0]),                     # 몸통
             cap(0.075, 0.7, [-0.09, 0.47, 0]), cap(0.075, 0.7, [0.09, 0.47, 0]),   # 다리
             cap(0.055, 0.5, [-0.25, 1.12, 0]), cap(0.055, 0.5, [0.25, 1.12, 0])]   # 팔
    head = trimesh.creation.icosphere(subdivisions=3, radius=0.115)
    head.apply_translation([0, 1.63, 0])
    return {"body": trimesh.util.concatenate(parts + [head])}, {"length": 0.5, "width": 0.5}


# ----------------------------------------------------------------------
def export_meshes(groups, prefix, workdir):
    """trimesh → glb → balsam → .mesh 파일 (그룹 이름별)"""
    scene = trimesh.Scene()
    for name, mesh in groups.items():
        if mesh is None or len(mesh.faces) == 0:
            continue
        scene.add_geometry(mesh, node_name="{}_{}".format(prefix, name), geom_name="{}_{}".format(prefix, name))
    glb = os.path.join(workdir, prefix + ".glb")
    scene.export(glb)
    out = os.path.join(workdir, prefix + "_out")
    import PySide6
    balsam = os.path.join(os.path.dirname(PySide6.__file__), "balsam")
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen", LC_ALL="C.UTF-8")
    subprocess.run([balsam, "-o", out, glb], check=True, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    made = []
    for f in os.listdir(os.path.join(out, "meshes")):
        if f.endswith(".mesh"):
            # balsam 이 소문자로 바꿈: prefix_group.mesh
            shutil.copy(os.path.join(out, "meshes", f), os.path.join(OUT_DIR, f.lower()))
            made.append(f.lower())
    return sorted(made)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--car", required=True, help="자차 모델 .glb")
    ap.add_argument("--traffic", default=None, help="주변 차 모델 .glb (기본: 자차 모델을 단순화)")
    ap.add_argument("--traffic-height", type=float, default=1.12, help="주변 차 높이 배율 (스포츠카 모델을 세단 비율로)")
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    info = {}
    with tempfile.TemporaryDirectory() as work:
        print("[ego]", args.car)
        g = load_groups(args.car)
        info["ego"] = normalize(g)
        ego = {k: decimate(m, EGO_BUDGET.get(k, 4000)) for k, m in g.items()}
        for k, m in ego.items():
            print("  {:<11} {:>6} tris".format(k, len(m.faces)))
        print("  →", export_meshes(ego, "ego", work))

        src = load_groups(args.traffic) if args.traffic else load_groups(args.car)
        info["car"] = normalize(src)
        merged = {}
        for name, members in TRAFFIC_GROUPS.items():
            parts = [src[m] for m in members if m in src]
            if parts:
                total = sum(len(pp.faces) for pp in parts)
                # 부품별로 따로 줄여야 경계(도장 투톤 이음새)에 깨짐이 안 생김
                parts = [decimate(pp, max(200, int(TRAFFIC_BUDGET[name] * len(pp.faces) / total))) for pp in parts]
                mm = trimesh.util.concatenate(parts)
                mm.apply_scale([1.0, args.traffic_height, 1.0])
                merged[name] = mm
        info["car"]["height"] *= args.traffic_height
        print("[car]", {k: len(v.faces) for k, v in merged.items()}, "→", export_meshes(merged, "car", work))

        for name, fn in (("truck", make_truck), ("bus", make_bus), ("bike", make_bike), ("person", make_person)):
            parts, dims = fn()
            parts = {k: smooth_normals(v, 50) for k, v in parts.items() if v is not None and len(v.faces)}
            info[name] = dims
            print("[{}]".format(name), export_meshes(parts, name, work))

    import json
    with open(os.path.join(OUT_DIR, "models.json"), "w") as f:
        json.dump(info, f, indent=1)
    print("크기 정보:", json.dumps(info))
    print("완료:", os.path.abspath(OUT_DIR))


if __name__ == "__main__":
    sys.exit(main())
