#!/usr/bin/env python3
"""HMI 3D 화면용 스튜디오 조명 환경맵(.hdr) 생성 → assets/env/studio.hdr (차 도장 반사용)"""
import os
import numpy as np

W, H = 512, 256
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "env", "studio.hdr")


def build(dark=False):
    v = (np.arange(H) + 0.5) / H              # 0 위 → 1 아래
    u = (np.arange(W) + 0.5) / W
    U, V = np.meshgrid(u, v)
    elev = (0.5 - V) * np.pi                  # +위
    sky = 1.25 + 0.55 * np.clip(np.sin(elev), 0, 1)
    ground = 0.42 + 0.15 * np.clip(-np.sin(elev), 0, 1)
    t = np.clip((elev + 0.08) / 0.16, 0, 1)
    val = ground * (1 - t) + sky * t
    img = np.stack([val, val * 1.005, val * 1.02], axis=-1)
    # 천장 소프트박스 2개 + 측면 스트립 (도장에 길게 맺히는 반사)
    for cu, w, e0, e1, k in ((0.25, 0.10, 0.9, 1.4, 5.0), (0.75, 0.10, 0.9, 1.4, 4.0),
                             (0.0, 0.035, 0.12, 0.55, 3.0), (0.5, 0.035, 0.12, 0.55, 3.0)):
        du = np.minimum(np.abs(U - cu), 1 - np.abs(U - cu))
        m = (du < w) & (elev > e0 - 0.9 if e0 > 0.8 else (elev > e0)) & (elev < e1)
        img[m] += k
    if dark:
        img *= 0.35
    return img.astype(np.float32)


def write_hdr(path, img):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    h, w, _ = img.shape
    m = img.max(axis=2)
    e = np.zeros_like(m, dtype=np.int32)
    mant = np.zeros_like(m)
    nz = m > 1e-32
    mant[nz], e[nz] = np.frexp(m[nz])
    scale = np.zeros_like(m)
    scale[nz] = mant[nz] * 256.0 / m[nz]
    rgbe = np.zeros((h, w, 4), dtype=np.uint8)
    rgbe[..., :3] = np.clip(img * scale[..., None], 0, 255).astype(np.uint8)
    rgbe[..., 3] = np.where(nz, e + 128, 0).astype(np.uint8)
    with open(path, "wb") as f:
        f.write(b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n")
        f.write("-Y {} +X {}\n".format(h, w).encode())
        f.write(rgbe.tobytes())


if __name__ == "__main__":
    write_hdr(OUT, build())
    write_hdr(OUT.replace("studio.hdr", "studio_dark.hdr"), build(dark=True))
    print("saved", os.path.abspath(OUT))
