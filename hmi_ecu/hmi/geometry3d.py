"""
Qt Quick 3D 용 도로 형상 (차선, 도로면, 위험 영역).

QML:
    import PaceHmi 1.0
    Model { geometry: RibbonGeometry { points: [[x, y], ...]; width: 0.15; dash: 3; gap: 6 } }   # UV v = 거리/(dash+gap)
    Model { geometry: RibbonGeometry { left: [[x, y]...]; right: [[x, y]...] } }      # 두 선 사이 채우기

좌표: 장면 dict 와 같은 자차 기준 (x 앞, y 오른쪽, m) → 3D 좌표 X = y, Z = -x, 높이 Y = ego-local z + lift (추가 경사 보정 없음)
"""

import math
import struct

from PySide6.QtCore import Property, QByteArray, Signal
from PySide6.QtGui import QVector3D
from PySide6.QtQuick3D import QQuick3DGeometry


def _pts(v):
    out = []
    for p in v or []:
        try:
            point = (float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0)
            if all(math.isfinite(c) for c in point):
                out.append(point)
        except (TypeError, ValueError, IndexError):
            continue
    return out


def _resample(pts, step):
    """폴리라인을 일정 간격(step m)으로 다시 샘플 → 누적거리 같이 반환"""
    if len(pts) < 2:
        return [], []
    out, dist = [pts[0]], [0.0]
    acc = 0.0
    for (x0, y0, z0), (x1, y1, z1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-6:
            continue
        t = step - (acc % step) if acc % step else step
        while t < seg:
            out.append((x0 + (x1 - x0) * t / seg, y0 + (y1 - y0) * t / seg, z0 + (z1 - z0) * t / seg))
            dist.append(acc + t)
            t += step
        acc += seg
        out.append((x1, y1, z1))
        dist.append(acc)
    return out, dist


class RibbonGeometry(QQuick3DGeometry):
    changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._points = []
        self._left = []
        self._right = []
        self._width = 0.15
        self._dash = 0.0
        self._gap = 0.0
        self._phase = 0.0
        self._lift = 0.01
        self._bands = []          # [{"L": [[x,y]..], "R": [..]}, ...]  여러 도로면을 한 형상으로
        self._strips = []         # [{"p": [[x,y]..], "w": 폭}, ...]      여러 차선을 한 형상으로
        self._rebuild()

    # ---------------- properties ----------------
    def _mk(name, conv):
        def getter(self):
            return getattr(self, "_" + name)

        def setter(self, value):
            value = conv(value)
            if value != getattr(self, "_" + name):
                setattr(self, "_" + name, value)
                self._rebuild()
                self.changed.emit()
        return getter, setter

    points = Property("QVariantList", *_mk("points", lambda v: list(v or [])), notify=changed)
    left = Property("QVariantList", *_mk("left", lambda v: list(v or [])), notify=changed)
    right = Property("QVariantList", *_mk("right", lambda v: list(v or [])), notify=changed)
    width = Property(float, *_mk("width", float), notify=changed)
    dash = Property(float, *_mk("dash", float), notify=changed)
    gap = Property(float, *_mk("gap", float), notify=changed)
    phase = Property(float, *_mk("phase", float), notify=changed)
    lift = Property(float, *_mk("lift", float), notify=changed)
    bands = Property("QVariantList", *_mk("bands", lambda v: list(v or [])), notify=changed)
    strips = Property("QVariantList", *_mk("strips", lambda v: list(v or [])), notify=changed)
    del _mk

    # ---------------- build ----------------
    def _rebuild(self):
        tris = []
        if self._bands or self._strips:
            for b in self._bands:
                L, R = _pts(b.get("L")), _pts(b.get("R"))
                if len(L) >= 2 and len(R) >= 2:
                    tris += self._band(L, R)
            for st in self._strips:
                tris += self._strip(_pts(st.get("p")), float(st.get("w", self._width)))
            self._upload(tris)
            return
        L, R = _pts(self._left), _pts(self._right)
        if len(L) >= 2 and len(R) >= 2:
            tris = self._band(L, R)
        else:
            tris = self._strip(_pts(self._points))
        self._upload(tris)

    def _band(self, L, R):
        # 두 폴리라인을 같은 개수로 맞춰서 사다리꼴로 채움
        n = max(len(L), len(R))

        def at(P, i):
            f = i * (len(P) - 1) / (n - 1)
            k = min(int(f), len(P) - 2)
            t = f - k
            return tuple(P[k][axis] + (P[k + 1][axis] - P[k][axis]) * t for axis in range(3))
        tris = []
        for i in range(n - 1):
            a, b = at(L, i), at(L, i + 1)
            c, d = at(R, i), at(R, i + 1)
            tris += [(a, 0.0, 0.0), (c, 1.0, 0.0), (b, 0.0, 1.0), (b, 0.0, 1.0), (c, 1.0, 0.0), (d, 1.0, 1.0)]
        return tris

    def _strip(self, pts, width=None):
        if len(pts) < 2:
            return []
        pts, dist = _resample(pts, 2.0)
        if len(pts) < 2:
            return []
        hw = (self._width if width is None else width) / 2.0
        period = self._dash + self._gap
        tris = []

        def normal(i):
            j0, j1 = max(0, i - 1), min(len(pts) - 1, i + 1)
            dx, dy = pts[j1][0] - pts[j0][0], pts[j1][1] - pts[j0][1]
            n = math.hypot(dx, dy) or 1.0
            return (-dy / n, dx / n)

        per = period if period > 0 else 1.0
        for i in range(len(pts) - 1):
            # 점선은 텍스처(lane_dash.png)를 v 방향으로 반복 → 흐르는 효과는 GPU 에서 (Texture.positionV)
            v0, v1 = dist[i] / per, dist[i + 1] / per
            (x0, y0, z0), (x1, y1, z1) = pts[i], pts[i + 1]
            n0, n1 = normal(i), normal(i + 1)
            a = (x0 + n0[0] * hw, y0 + n0[1] * hw, z0)
            b = (x0 - n0[0] * hw, y0 - n0[1] * hw, z0)
            c = (x1 + n1[0] * hw, y1 + n1[1] * hw, z1)
            d = (x1 - n1[0] * hw, y1 - n1[1] * hw, z1)
            tris += [(a, 0.0, v0), (b, 1.0, v0), (c, 0.0, v1), (c, 0.0, v1), (b, 1.0, v0), (d, 1.0, v1)]
        return tris

    def _upload(self, tris):
        self.clear()
        y = self._lift
        buf = bytearray()
        lo = [1e9, y, 1e9]
        hi = [-1e9, y, -1e9]
        # 3D: X = y_ego, Z = -x_ego. 위에서 볼 때 반시계(앞면 위쪽)가 되게 순서 확인
        for i in range(0, len(tris), 3):
            p = [(t[0][1], t[0][2] + y, -t[0][0], t[1], t[2]) for t in tris[i:i + 3]]
            ux, uz = p[1][0] - p[0][0], p[1][2] - p[0][2]
            vx, vz = p[2][0] - p[0][0], p[2][2] - p[0][2]
            if ux * vz - uz * vx > 0:        # 아래를 보고 있으면 뒤집기
                p[1], p[2] = p[2], p[1]
            u = [p[1][j] - p[0][j] for j in range(3)]
            v = [p[2][j] - p[0][j] for j in range(3)]
            normal = [u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0]]
            norm = math.sqrt(sum(c*c for c in normal))
            normal = [c / norm for c in normal] if norm else [0.0, 1.0, 0.0]
            for X, Y, Z, U, V in p:
                buf += struct.pack("<8f", X, Y, Z, *normal, U, V)
                lo[1], hi[1] = min(lo[1], Y), max(hi[1], Y)
                lo[0], lo[2] = min(lo[0], X), min(lo[2], Z)
                hi[0], hi[2] = max(hi[0], X), max(hi[2], Z)
        if not buf:
            # 빈 형상도 유효하게 (삼각형 1개 크기 0)
            buf = bytearray(struct.pack("<8f", 0, y, 0, 0, 1, 0, 0, 0) * 3)
            lo, hi = [0, y, 0], [0, y, 0]
        self.setVertexData(QByteArray(bytes(buf)))
        self.setStride(32)
        self.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0,
                          QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.NormalSemantic, 12,
                          QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.addAttribute(QQuick3DGeometry.Attribute.Semantic.TexCoord0Semantic, 24,
                          QQuick3DGeometry.Attribute.ComponentType.F32Type)
        self.setBounds(QVector3D(lo[0], lo[1] - 0.01, lo[2]), QVector3D(hi[0], hi[1] + 0.01, hi[2]))
        self.update()


def register():
    from PySide6.QtQml import qmlRegisterType
    qmlRegisterType(RibbonGeometry, "PaceHmi", 1, 0, "RibbonGeometry")
