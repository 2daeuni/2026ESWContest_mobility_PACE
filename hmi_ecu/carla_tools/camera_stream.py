"""
CARLA 전방 운전자 카메라 영상 → HMI (TCP 5602)

- 자차 전방 운전자 위치에 RGB 카메라
  (+clean 모드면 Semantic Segmentation 카메라)를 붙여 실제 렌더링 영상을 받음
- 차선 경로 / 앞차 박스를 카메라 내부 파라미터로 투영해
  이미지 픽셀 좌표로 같이 보냄
  → HMI가 영상 위에 정확히 겹쳐 그림
- 프레임 형식:
  [4B header_len][4B jpeg_len][header JSON][JPEG]
  (big endian)

모드
  real  : CARLA 렌더링 그대로
  clean : 테슬라식 정리 화면.
          실제 3D 차·보행자는 그대로 두고,
          도로/인도/차선만 단색으로,
          나머지(건물·나무·하늘)는 밝은 배경으로
"""

import io
import json
import math
import queue
import socket
import struct
import threading
import time


# CARLA 0.9.14+ CityObjectLabel
TAG_ROAD = 1
TAG_SIDEWALK = 2
TAG_ROADLINE = 24
TAG_GROUND = 25

TAG_VEHICLES = (
    13,
    14,
    15,
    16,
    17,
    18,
    19,
)

TAG_PEDESTRIAN = 12


# =====================================================================
# 투영
# carla 없이 테스트 가능하게 행렬/좌표만 사용
# =====================================================================

def intrinsics(width, height, fov_deg):
    f = width / (
        2.0
        * math.tan(
            math.radians(fov_deg) / 2.0
        )
    )

    return (
        f,
        width / 2.0,
        height / 2.0,
    )


def world_to_pixel(inv_matrix, K, point):
    """
    inv_matrix :
        camera.get_transform().get_inverse_matrix()
        (4x4 리스트)

    K :
        (f, cx, cy)

    point :
        (x, y, z) 월드 좌표

    반환:
        (u, v, depth)
        또는 None (카메라 뒤쪽)

    UE4 카메라 좌표:
        x 앞
        y 오른쪽
        z 위
    """

    x, y, z = point
    m = inv_matrix

    cxw = (
        m[0][0] * x
        + m[0][1] * y
        + m[0][2] * z
        + m[0][3]
    )

    cyw = (
        m[1][0] * x
        + m[1][1] * y
        + m[1][2] * z
        + m[1][3]
    )

    czw = (
        m[2][0] * x
        + m[2][1] * y
        + m[2][2] * z
        + m[2][3]
    )

    if cxw <= 0.3:
        return None

    f, cx, cy = K

    return (
        cx + f * cyw / cxw,
        cy - f * czw / cxw,
        cxw,
    )


def project_polyline(
    inv_matrix,
    K,
    points,
    width,
    height,
):
    out = []

    for p in points:
        q = world_to_pixel(
            inv_matrix,
            K,
            p,
        )

        if q is None:
            continue

        u, v, _depth = q

        if (
            -width <= u <= 2 * width
            and -height <= v <= 2 * height
        ):
            out.append(
                [
                    round(u, 1),
                    round(v, 1),
                ]
            )

    return out


def project_box(
    inv_matrix,
    K,
    vertices,
    width,
    height,
):
    pts = [
        world_to_pixel(
            inv_matrix,
            K,
            v,
        )
        for v in vertices
    ]

    pts = [
        p
        for p in pts
        if p is not None
    ]

    if len(pts) < 4:
        return None

    us = [p[0] for p in pts]
    vs = [p[1] for p in pts]

    x0 = max(
        0.0,
        min(us),
    )

    x1 = min(
        float(width),
        max(us),
    )

    y0 = max(
        0.0,
        min(vs),
    )

    y1 = min(
        float(height),
        max(vs),
    )

    if (
        x1 - x0 < 2
        or y1 - y0 < 2
    ):
        return None

    return [
        round(x0, 1),
        round(y0, 1),
        round(x1, 1),
        round(y1, 1),
    ]


# =====================================================================
# clean (테슬라식) 스타일
# =====================================================================

def stylize_clean(
    np,
    rgb,
    tags,
):
    """
    rgb:
        HxWx3 uint8 (RGB)

    tags:
        HxW uint8 (semantic tag)

    실제 차/보행자 픽셀은
    원본 렌더링을 살짝 탈색해서 유지

    → 진짜 3D 형상·반사·그림자가 그대로 보임
    """

    h, w = tags.shape

    out = np.empty(
        (h, w, 3),
        dtype=np.uint8,
    )

    # -------------------------------------------------------------
    # 배경
    # 위는 밝게,
    # 아래로 갈수록 살짝 어둡게
    # -------------------------------------------------------------

    grad = np.linspace(
        246,
        232,
        h,
        dtype=np.float32,
    )[:, None]

    out[:] = np.repeat(
        grad,
        w,
        axis=1,
    )[:, :, None].astype(
        np.uint8
    )

    lum = (
        0.299 * rgb[:, :, 0]
        + 0.587 * rgb[:, :, 1]
        + 0.114 * rgb[:, :, 2]
    ).astype(
        np.float32
    )

    # -------------------------------------------------------------
    # 도로
    # -------------------------------------------------------------

    road = (
        (tags == TAG_ROAD)
        | (tags == TAG_GROUND)
    )

    # 단색 + 원본 밝기 약간
    # 그림자 남김

    road_val = (
        196
        + (lum - 110) * 0.12
    ).clip(
        175,
        215,
    )

    out[road] = np.stack(
        [
            road_val,
            road_val + 3,
            road_val + 7,
        ],
        axis=-1,
    )[road].astype(
        np.uint8
    )

    # -------------------------------------------------------------
    # 인도
    # -------------------------------------------------------------

    side = (
        tags
        == TAG_SIDEWALK
    )

    out[side] = (
        226,
        229,
        233,
    )

    # -------------------------------------------------------------
    # 차선
    # -------------------------------------------------------------

    lines = (
        tags
        == TAG_ROADLINE
    )

    out[lines] = (
        252,
        252,
        252,
    )

    # -------------------------------------------------------------
    # 차량 / 보행자
    # -------------------------------------------------------------

    veh = (
        np.isin(
            tags,
            TAG_VEHICLES,
        )
        | (
            tags
            == TAG_PEDESTRIAN
        )
    )

    if veh.any():
        src = rgb.astype(
            np.float32
        )

        gray = lum[
            :,
            :,
            None,
        ]

        # 탈색 70% + 원본 30%
        # 대비 강화
        # → 테슬라 회색 3D 차 느낌

        mixed = (
            gray * 0.7
            + src * 0.3
        )

        mixed = (
            (mixed - 128)
            * 1.15
            + 140
        )

        out[veh] = (
            mixed
            .clip(0, 255)
            .astype(np.uint8)[veh]
        )

    return out


# =====================================================================
# TCP FRAME SENDER
# =====================================================================

class FrameSender(
    threading.Thread
):
    """
    HMI(TCP 서버)에 접속해서
    최신 프레임만 보냄.

    느리면 오래된 프레임은 버리고,
    연결이 끊기면 자동 재접속.
    """

    def __init__(
        self,
        host,
        port,
    ):
        super().__init__(
            daemon=True,
            name="frame-sender",
        )

        self.host = host
        self.port = port

        self.q = queue.Queue(
            maxsize=1
        )

        self.stop_event = (
            threading.Event()
        )

        self.connected = False
        self.sent = 0

    def offer(
        self,
        header,
        jpeg,
    ):
        # 기존 오래된 프레임 제거
        try:
            self.q.get_nowait()
        except queue.Empty:
            pass

        # 최신 프레임 삽입
        try:
            self.q.put_nowait(
                (
                    header,
                    jpeg,
                )
            )
        except queue.Full:
            pass

    def run(self):
        sock = None

        while not self.stop_event.is_set():

            # -----------------------------------------------------
            # HMI 연결
            # -----------------------------------------------------

            if sock is None:
                try:
                    sock = (
                        socket.create_connection(
                            (
                                self.host,
                                self.port,
                            ),
                            timeout=2.0,
                        )
                    )

                    sock.setsockopt(
                        socket.IPPROTO_TCP,
                        socket.TCP_NODELAY,
                        1,
                    )

                    sock.settimeout(
                        2.0
                    )

                    self.connected = True

                    print(
                        "[camera] "
                        f"HMI {self.host}:{self.port} "
                        "연결됨"
                    )

                except OSError:
                    self.connected = False

                    self.stop_event.wait(
                        1.0
                    )

                    continue

            # -----------------------------------------------------
            # 최신 프레임 대기
            # -----------------------------------------------------

            try:
                header, jpeg = (
                    self.q.get(
                        timeout=0.5
                    )
                )

            except queue.Empty:
                continue

            hb = json.dumps(
                header,
                separators=(
                    ",",
                    ":",
                ),
            ).encode(
                "utf-8"
            )

            # -----------------------------------------------------
            # TCP 송신
            #
            # [4B header length]
            # [4B jpeg length]
            # [header JSON]
            # [jpeg]
            # -----------------------------------------------------

            try:
                sock.sendall(
                    struct.pack(
                        ">II",
                        len(hb),
                        len(jpeg),
                    )
                    + hb
                    + jpeg
                )

                self.sent += 1

            except OSError:
                print(
                    "[camera] "
                    "HMI 연결 끊김, "
                    "재접속 시도"
                )

                try:
                    sock.close()
                except OSError:
                    pass

                sock = None
                self.connected = False

        if sock is not None:
            sock.close()

    def stop(self):
        self.stop_event.set()


# =====================================================================
# CARLA FRONT DRIVER CAMERA
# =====================================================================

REAR_FOV = 110.0
REAR_PITCH = 30.0                      # 아래로 (도)

# 후방 가이드선 구간: (시작 m, 끝 m, 색)  — 뒤 범퍼 기준 거리
REAR_GUIDE_BANDS = ((0.0, 0.5, "red"), (0.5, 1.0, "yellow"), (1.0, 3.0, "green"))


def rear_cam_pixel(K, w, cam_h, pitch_deg, back, lateral):
    """
    후방 카메라 영상(좌우 반전 후) 픽셀.
    back: 뒤 범퍼에서 뒤쪽 거리 (m), lateral: 운전자 기준 오른쪽 + (m), 지면 높이 0
    """
    a = math.radians(pitch_deg)
    X, Z = back, -cam_h                         # 카메라 수평 기준: 앞(=차 뒤쪽), 위
    xc = X * math.cos(a) - Z * math.sin(a)
    zc = X * math.sin(a) + Z * math.cos(a)
    if xc <= 0.05:
        return None
    f, cx, cy = K
    yc = -lateral                               # 뒤를 보는 카메라의 오른쪽 = 차의 왼쪽
    u = cx + f * yc / xc
    v = cy - f * zc / xc
    return [round(w - u, 1), round(v, 1)]       # 좌우 반전 (룸미러처럼)


def rear_guides(K, w, h, cam_h, pitch_deg, half_w):
    """후진 가이드선: 양옆 선(구간별 색) + 0.5/1/2/3 m 가로선. [{"c", "pts"}]"""
    out = []
    for d0, d1, color in REAR_GUIDE_BANDS:
        n = 6
        for side in (-1, 1):
            pts = []
            for i in range(n + 1):
                d = d0 + (d1 - d0) * i / n
                p = rear_cam_pixel(K, w, cam_h, pitch_deg, max(0.02, d), side * half_w)
                if p is not None:
                    pts.append(p)
            if len(pts) >= 2:
                out.append({"c": color, "pts": pts})
    for d, color in ((0.5, "red"), (1.0, "yellow"), (2.0, "green"), (3.0, "green")):
        a = rear_cam_pixel(K, w, cam_h, pitch_deg, d, -half_w)
        b = rear_cam_pixel(K, w, cam_h, pitch_deg, d, half_w)
        if a and b:
            q = 0.22                                       # 가로선은 양 끝만 짧게
            out.append({"c": color, "pts": [a, [a[0] + (b[0] - a[0]) * q, a[1] + (b[1] - a[1]) * q]]})
            out.append({"c": color, "pts": [b, [b[0] + (a[0] - b[0]) * q, b[1] + (a[1] - b[1]) * q]]})
    return out


class ChaseCamera:
    """
    기존 클래스 이름 ChaseCamera는
    다른 코드와의 호환성을 위해 유지한다.

    실제 동작은:
        자차에 전방 운전자 카메라를 부착하고
        매 프레임 JPEG + Overlay를
        FrameSender로 넘긴다.

    Overlay:
        메인 루프가 set_overlay_source()로
        최신 차선 / 차량 정보를 넣어준다.
    """

    def __init__(
        self,
        carla,
        world,
        ego,
        sender,
        mode="real",
        width=960,
        height=540,
        fov=75.0,
        rate=20.0,
        quality=82,
        back=4.0,
        up=2.6,
        pitch=-1.0,
    ):
        import numpy as np
        from PIL import Image

        self.carla = carla
        self.world = world
        self.ego = ego

        self.np = np
        self.Image = Image

        self.sender = sender

        self.mode = mode

        self.w = width
        self.h = height
        self.fov = fov

        self.K = intrinsics(
            width,
            height,
            fov,
        )

        self.quality = quality

        self.lock = (
            threading.Lock()
        )

        self.overlay_src = None
        self.last_tags = None

        self.frames = 0

        self.actors = []

        bp_lib = (
            world
            .get_blueprint_library()
        )

        ext = (
            ego
            .bounding_box
            .extent
        )

        # ==========================================================
        # FRONT DRIVER CAMERA POSITION
        #
        # 기존:
        #   x = 0.8 * ext.x
        #   z = 1.3 * ext.z
        #   pitch = -4°
        #
        # 문제:
        #   카메라가 너무 낮아서
        #   보닛/차체가 화면 아래쪽을 많이 가림
        #
        # 변경:
        #   x = 0.8 * ext.x
        #   z = 1.8 * ext.z
        #   pitch = -1°
        #
        # 목적:
        #   - 전방 도로가 화면 대부분 차지
        #   - 보닛은 거의 안 보이거나 아래쪽에만 조금
        #   - 너무 높은 3인칭 시점은 피함
        # ==========================================================

        rel = carla.Transform(
            carla.Location(
                x=0.8 * ext.x,
                y=0.0,
                z=1.8 * ext.z,
            ),
            carla.Rotation(
                pitch=pitch,
                yaw=0.0,
                roll=0.0,
            ),
        )

        print(
            "[camera] FRONT DRIVER VIEW"
        )

        print(
            "[camera] "
            f"x={0.8 * ext.x:.2f}, "
            f"y=0.00, "
            f"z={1.8 * ext.z:.2f}, "
            f"pitch={pitch:.1f}"
        )

        # back/up은 기존 실행 옵션 호환을 위해
        # argument로 계속 받지만
        # 현재 전방 카메라 위치 계산에는 사용하지 않는다.

        _ = back
        _ = up

        def make(kind, rel=rel, fov=fov):
            bp = bp_lib.find(
                "sensor.camera."
                + kind
            )

            bp.set_attribute(
                "image_size_x",
                str(width),
            )

            bp.set_attribute(
                "image_size_y",
                str(height),
            )

            bp.set_attribute(
                "fov",
                str(fov),
            )

            bp.set_attribute(
                "sensor_tick",
                str(
                    1.0 / rate
                ),
            )

            if kind == "rgb":
                for attr, val in (
                    (
                        "motion_blur_intensity",
                        "0.0",
                    ),
                    (
                        "enable_postprocess_effects",
                        "true",
                    ),
                ):
                    if bp.has_attribute(
                        attr
                    ):
                        bp.set_attribute(
                            attr,
                            val,
                        )

            # 운전자 시점은 차량 움직임에
            # 완전히 고정되어야 하므로 Rigid 사용

            attach = (
                carla
                .AttachmentType
                .Rigid
            )

            cam = world.spawn_actor(
                bp,
                rel,
                attach_to=ego,
                attachment_type=attach,
            )

            self.actors.append(
                cam
            )

            return cam

        # ----------------------------------------------------------
        # CLEAN 모드:
        # RGB + Semantic Segmentation
        # ----------------------------------------------------------

        if mode == "clean":
            self.seg = make(
                "semantic_segmentation"
            )

            self.seg.listen(
                self._on_seg
            )

        # ----------------------------------------------------------
        # RGB CAMERA
        # ----------------------------------------------------------

        self.cam = make(
            "rgb"
        )

        self.cam.listen(
            self._on_rgb
        )

        # ----------------------------------------------------------
        # REAR CAMERA (R 기어일 때만 생성 → 평소 CARLA 부담 없음)
        # 뒤 범퍼 가운데, 뒤를 보고 약간 아래로. 영상은 좌우 반전(룸미러처럼)
        # ----------------------------------------------------------
        self._make = make
        self.rear = None
        self.reverse = False
        self.rear_fov = REAR_FOV
        self.rear_K = intrinsics(width, height, REAR_FOV)
        self.rear_h = max(0.5, 1.15 * ext.z)
        self.rear_pitch = REAR_PITCH
        self.half_w = ext.y
        self.rear_rel = carla.Transform(
            carla.Location(x=-ext.x - 0.05, y=0.0, z=self.rear_h),
            carla.Rotation(pitch=-REAR_PITCH, yaw=180.0, roll=0.0),
        )

    # =================================================================
    # 후방 카메라 켜기/끄기 (브리지가 매 루프 기어로 호출)
    # =================================================================

    def set_reverse(self, on):
        on = bool(on)
        if on == self.reverse:
            return
        self.reverse = on
        if on and self.rear is None:
            try:
                self.rear = self._make("rgb", rel=self.rear_rel, fov=self.rear_fov)
                self.rear.listen(self._on_rear)
                print("[camera] R 기어 → 후방 카메라")
            except Exception as error:
                print("[camera] 후방 카메라 생성 실패:", error)
                self.rear = None
        elif not on and self.rear is not None:
            try:
                self.rear.stop()
                self.rear.destroy()
            except Exception:
                pass
            if self.rear in self.actors:
                self.actors.remove(self.rear)
            self.rear = None
            print("[camera] D/P 기어 → 전방 카메라")

    def _on_rear(self, image):
        np = self.np
        bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        rgb = bgra[:, ::-1, 2::-1]                     # BGRA → RGB + 좌우 반전
        buf = io.BytesIO()
        self.Image.fromarray(np.ascontiguousarray(rgb)).save(buf, format="JPEG", quality=self.quality)
        with self.lock:
            src = self.overlay_src or {}
        header = {
            "v": 1, "type": "camera_frame", "frame": image.frame, "t": image.timestamp,
            "w": self.w, "h": self.h, "mode": self.mode, "view": "rear",
            "overlay": {"guides": rear_guides(self.rear_K, self.w, self.h, self.rear_h, self.rear_pitch,
                                              self.half_w + 0.15),
                        "rear_gap": src.get("rear_gap")},
        }
        self.sender.offer(header, buf.getvalue())
        self.frames += 1

    # =================================================================
    # OVERLAY SOURCE
    # =================================================================

    def set_overlay_source(
        self,
        src,
    ):
        with self.lock:
            self.overlay_src = src

    # =================================================================
    # SEGMENTATION CALLBACK
    # =================================================================

    def _on_seg(
        self,
        image,
    ):
        arr = (
            self.np
            .frombuffer(
                image.raw_data,
                dtype=self.np.uint8,
            )
            .reshape(
                image.height,
                image.width,
                4,
            )
        )

        self.last_tags = (
            image.frame,
            arr[:, :, 2].copy(),
        )

    # =================================================================
    # RGB CALLBACK
    # =================================================================

    def _on_rgb(
        self,
        image,
    ):
        if self.reverse and self.rear is not None:
            return                       # 후진 중에는 후방 영상만 보냄
        np = self.np

        bgra = (
            np.frombuffer(
                image.raw_data,
                dtype=np.uint8,
            )
            .reshape(
                image.height,
                image.width,
                4,
            )
        )

        # BGRA → RGB
        rgb = bgra[
            :,
            :,
            2::-1,
        ]

        # ----------------------------------------------------------
        # CLEAN MODE
        # ----------------------------------------------------------

        if (
            self.mode == "clean"
            and self.last_tags
            is not None
        ):
            rgb = stylize_clean(
                np,
                rgb,
                self.last_tags[1],
            )

        # ----------------------------------------------------------
        # JPEG ENCODING
        # ----------------------------------------------------------

        buf = io.BytesIO()

        self.Image.fromarray(
            np.ascontiguousarray(
                rgb
            )
        ).save(
            buf,
            format="JPEG",
            quality=self.quality,
        )

        # ----------------------------------------------------------
        # OVERLAY
        #
        # 이 프레임을 찍은
        # 실제 카메라 transform 기준으로 투영
        # ----------------------------------------------------------

        inv = (
            image
            .transform
            .get_inverse_matrix()
        )

        with self.lock:
            src = (
                self.overlay_src
            )

        if src:
            overlay = build_overlay(
                inv,
                self.K,
                self.w,
                self.h,
                src,
            )

        else:
            overlay = {}

        # ----------------------------------------------------------
        # HEADER
        # ----------------------------------------------------------

        header = {
            "v": 1,
            "type": "camera_frame",
            "frame": image.frame,
            "t": image.timestamp,
            "w": self.w,
            "h": self.h,
            "mode": self.mode,
            "view": "front",
            "overlay": overlay,
        }

        # ----------------------------------------------------------
        # TCP SENDER QUEUE
        # ----------------------------------------------------------

        self.sender.offer(
            header,
            buf.getvalue(),
        )

        self.frames += 1

    # =================================================================
    # DESTROY
    # =================================================================

    def destroy(self):
        for a in self.actors:
            try:
                a.stop()
                a.destroy()

            except Exception:
                pass

        self.actors = []


# =====================================================================
# CAMERA OVERLAY
# =====================================================================

def build_overlay(
    inv,
    K,
    w,
    h,
    src,
):
    """
    src = {
        "lane_left":
            [(x,y,z)...],

        "lane_right":
            [(x,y,z)...],

        "lead_vertices":
            [(x,y,z)*8] or None,

        "lead_gap":
            m,

        "others":
            [
                [(x,y,z)*8],
                ...
            ]
    }
    """

    out = {
        "lane_left": project_polyline(
            inv,
            K,
            src.get(
                "lane_left",
                [],
            ),
            w,
            h,
        ),

        "lane_right": project_polyline(
            inv,
            K,
            src.get(
                "lane_right",
                [],
            ),
            w,
            h,
        ),

        "lead_box": None,

        "lead_gap": src.get(
            "lead_gap"
        ),

        "boxes": [],
    }

    # -------------------------------------------------------------
    # LEAD VEHICLE BOX
    # -------------------------------------------------------------

    if src.get(
        "lead_vertices"
    ):
        out["lead_box"] = (
            project_box(
                inv,
                K,
                src[
                    "lead_vertices"
                ],
                w,
                h,
            )
        )

    # -------------------------------------------------------------
    # OTHER VEHICLE BOXES
    # -------------------------------------------------------------

    for verts in src.get(
        "others",
        [],
    )[:24]:

        b = project_box(
            inv,
            K,
            verts,
            w,
            h,
        )

        if b:
            out[
                "boxes"
            ].append(
                b
            )

    return out