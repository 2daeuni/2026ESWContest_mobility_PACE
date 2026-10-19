#!/usr/bin/env python3
"""
CARLA 차량 이미지 생성기  (CARLA 가 돌아가는 PC 에서 실행)

CARLA 의 실제 차량 3D 모델을 렌더링해서 배경이 투명한 PNG 로 저장.
→ HMI 도로 장면/왼쪽 클러스터에 시뮬레이션과 똑같은 차가 나옴.

원리
  - 차량을 하늘 위(지면 +120 m)에 물리 끄고 띄움 → 나무/전봇대에 가려지지 않음
  - 같은 위치에 RGB 카메라 + Semantic Segmentation 카메라를 붙여 동시에 촬영
  - Segmentation 에서 차량 픽셀(Car/Truck/Bus/Motorcycle/Bicycle/Rider)만 alpha 로 사용
  - 2배 해상도로 찍고 축소해서 가장자리 계단현상 제거

뷰 3종 (HMI 와 맞춤)
  rear  : 뒤쪽 위에서 (도로 장면의 앞차/주변 차량, 자차)
  front : 앞쪽 위에서 (마주 오는 차)
  hero  : 앞-왼쪽 3/4 (왼쪽 클러스터의 큰 자차 이미지)

실행 (다른 주행 스크립트는 끄고 실행 권장: 잠깐 synchronous 모드로 바꿨다가 원래대로 복구함)
  python capture_vehicle_sprites.py --out ../assets/cars --default vehicle.tesla.model3
  python capture_vehicle_sprites.py --filter "vehicle.tesla.*,vehicle.lincoln.*"
  → 만들어진 assets/cars 폴더를 라즈베리파이 hmi_ecu/assets/cars 에 복사

필요 패키지: carla(서버와 같은 버전), numpy, pillow
"""

import argparse
import fnmatch
import math
import os
import queue
import shutil
import sys

VEHICLE_TAGS = {13, 14, 15, 16, 17, 18, 19}   # Rider, Car, Truck, Bus, Train, Motorcycle, Bicycle (CARLA 0.9.14+)

IMG_W, IMG_H = 1600, 1200      # 2배 해상도로 촬영
OUT_WIDTH = {"rear": 420, "front": 420, "hero": 760}


def view_transforms(carla, ext):
    """차량 bounding box 크기에 맞춘 카메라 상대 위치 (차량 좌표계: x 앞, y 오른쪽, z 위)."""
    L = max(ext.x, 1.0)
    H = max(ext.z, 0.6)

    def aim(x, y, z):
        # 차량 중심(0,0,H*0.6)을 바라보는 회전
        dx, dy, dz = -x, -y, H * 0.6 - z
        yaw = math.degrees(math.atan2(dy, dx))
        pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        return carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw))

    d = L * 2.0 + 6.0
    views = {
        # HMI 추격 카메라와 비슷한 내려다보는 각도 (~12°)
        "rear": aim(-d, 0.0, H + d * 0.21),
        "front": aim(d, 0.0, H + d * 0.21),
    }
    r = L * 2.3 + 4.5
    ang = math.radians(38)
    views["hero"] = aim(r * math.cos(ang), -r * math.sin(ang), H + 1.6)
    return views


def make_sprite(np, Image, rgb_bgra, sem_bgra, target_w):
    """RGB(BGRA) + Semantic(BGRA) → 차량만 남긴 투명 PNG(Image) . 차량 픽셀이 없으면 None."""
    h, w = rgb_bgra.shape[:2]
    tags = sem_bgra[:, :, 2]                       # BGRA → R 채널 = 태그
    mask = np.isin(tags, list(VEHICLE_TAGS))
    if mask.sum() < 500:
        return None

    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    rgba[:, :, 0] = rgb_bgra[:, :, 2]
    rgba[:, :, 1] = rgb_bgra[:, :, 1]
    rgba[:, :, 2] = rgb_bgra[:, :, 0]
    rgba[:, :, 3] = np.where(mask, 255, 0)

    # 차 가장자리까지 꽉 차게 자름 (HMI 가 이미지 폭 = 차 폭으로 크기를 계산)
    ys, xs = np.where(mask)
    crop = Image.fromarray(rgba[ys.min():ys.max() + 1, xs.min():xs.max() + 1], "RGBA")
    scale = target_w / float(crop.width)
    size = (target_w, max(1, int(round(crop.height * scale))))
    # premultiplied 로 축소해야 가장자리에 배경색이 번지지 않음
    return crop.convert("RGBa").resize(size, Image.LANCZOS).convert("RGBA")


def capture(world, carla, np, Image, bp_lib, bp_id, anchor, out_dir, views_wanted):
    bp = bp_lib.find(bp_id)
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "sprite_capture")
    if bp.has_attribute("color"):
        # 기본 색 하나로 고정 (추천값 첫 번째)
        rec = bp.get_attribute("color").recommended_values
        if rec:
            bp.set_attribute("color", rec[0])

    vehicle = world.try_spawn_actor(bp, anchor)
    if vehicle is None:
        print("  spawn 실패:", bp_id)
        return []
    actors = [vehicle]
    saved = []
    try:
        vehicle.set_simulate_physics(False)
        vehicle.set_transform(anchor)
        world.tick()
        ext = vehicle.bounding_box.extent

        for view, rel in view_transforms(carla, ext).items():
            if view not in views_wanted:
                continue
            cams = {}
            qs = {}
            for kind in ("rgb", "semantic_segmentation"):
                cbp = bp_lib.find("sensor.camera." + kind)
                cbp.set_attribute("image_size_x", str(IMG_W))
                cbp.set_attribute("image_size_y", str(IMG_H))
                cbp.set_attribute("fov", "40")
                if kind == "rgb":
                    for attr, val in (("motion_blur_intensity", "0"), ("bloom_intensity", "0.2")):
                        if cbp.has_attribute(attr):
                            cbp.set_attribute(attr, val)
                cam = world.spawn_actor(cbp, rel, attach_to=vehicle)
                q = queue.Queue()
                cam.listen(q.put)
                cams[kind], qs[kind] = cam, q
                actors.append(cam)

            # 텍스처/LOD 로딩 대기
            frame = None
            for _ in range(8):
                frame = world.tick()

            imgs = {}
            for kind, q in qs.items():
                img = None
                while True:
                    try:
                        img = q.get(timeout=5.0)
                    except queue.Empty:
                        break
                    if img.frame >= frame:
                        break
                imgs[kind] = img
            for cam in cams.values():
                cam.stop()
                cam.destroy()
                actors.remove(cam)

            if any(v is None for v in imgs.values()):
                print("  {} {}: 이미지 수신 실패".format(bp_id, view))
                continue

            rgb = np.frombuffer(imgs["rgb"].raw_data, dtype=np.uint8).reshape(IMG_H, IMG_W, 4)
            sem = np.frombuffer(imgs["semantic_segmentation"].raw_data, dtype=np.uint8).reshape(IMG_H, IMG_W, 4)
            resized = make_sprite(np, Image, rgb, sem, OUT_WIDTH[view])
            if resized is None:
                print("  {} {}: 차량 픽셀 없음 (태그 확인 필요)".format(bp_id, view))
                continue

            path = os.path.join(out_dir, "{}_{}.png".format(bp_id, view))
            resized.save(path, optimize=True)
            saved.append(path)
            print("  saved", os.path.basename(path), resized.size)
    finally:
        for a in actors:
            try:
                a.destroy()
            except Exception:
                pass
    return saved


def main():
    p = argparse.ArgumentParser(description="CARLA vehicle sprite capture for PACE HMI")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    here = os.path.dirname(os.path.abspath(__file__))
    p.add_argument("--out", default=os.path.join(os.path.dirname(here), "assets", "cars"))
    p.add_argument("--filter", default="vehicle.*", help="쉼표로 여러 패턴 (예: vehicle.tesla.*,vehicle.audi.*)")
    p.add_argument("--views", default="rear,front,hero")
    p.add_argument("--default", default="vehicle.tesla.model3",
                   help="모르는 차종에 쓸 기본 이미지(default_*.png)로 복사할 차종")
    p.add_argument("--cars-only", action="store_true", help="오토바이/자전거 제외")
    args = p.parse_args()

    try:
        import carla
        import numpy as np
        from PIL import Image
    except ImportError as error:
        print("필요 패키지 없음:", error, "\n  pip install carla==0.9.16 numpy pillow")
        return 1

    os.makedirs(args.out, exist_ok=True)
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    patterns = [s.strip() for s in args.filter.split(",") if s.strip()]
    bps = sorted({bp.id for bp in bp_lib.filter("vehicle.*") if any(fnmatch.fnmatch(bp.id, pt) for pt in patterns)})
    if args.cars_only:
        bps = [b for b in bps if bp_lib.find(b).get_attribute("number_of_wheels").as_int() >= 4]
    if args.default and args.default not in bps and bp_lib.filter(args.default):
        bps.append(args.default)
    views = [v.strip() for v in args.views.split(",") if v.strip()]
    print("대상 차종 {}개, 뷰 {}".format(len(bps), views))

    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    try:
        world.set_weather(carla.WeatherParameters.ClearNoon)
        sp = world.get_map().get_spawn_points()[0]
        anchor = carla.Transform(
            carla.Location(x=sp.location.x, y=sp.location.y, z=sp.location.z + 120.0),
            carla.Rotation(yaw=0.0),
        )
        all_saved = []
        for i, bp_id in enumerate(bps):
            print("[{}/{}] {}".format(i + 1, len(bps), bp_id))
            all_saved += capture(world, carla, np, Image, bp_lib, bp_id, anchor, args.out, views)

        if args.default:
            for view in views:
                src = os.path.join(args.out, "{}_{}.png".format(args.default, view))
                if os.path.exists(src):
                    shutil.copyfile(src, os.path.join(args.out, "default_{}.png".format(view)))
                    print("default_{}.png ← {}".format(view, args.default))
        print("완료: {}장 → {}".format(len(all_saved), args.out))
    finally:
        world.apply_settings(original)
    return 0


if __name__ == "__main__":
    sys.exit(main())
