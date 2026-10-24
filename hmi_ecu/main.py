#!/usr/bin/env python3
"""
PACE HMI (UI Display ECU) - Raspberry Pi 5 / PySide6 + QML

실행 예
    python3 main.py                 # 실차/벤치: can0 수신 + Safety ECU UDP 수신, 전체화면
    python3 main.py --scripted      # (개발용) 정해진 가상 시나리오 재생
    → 실제 페달로 주행해 보려면 화면 하단 DEMO 버튼
    python3 main.py --windowed      # 창 모드 (개발용)
    python3 main.py --can vcan0     # 가상 CAN 으로 통합 테스트
"""

import argparse
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from PySide6.QtCore import QTimer, QUrl  # noqa: E402
from PySide6.QtGui import QFont, QGuiApplication  # noqa: E402
from PySide6.QtQml import QQmlApplicationEngine  # noqa: E402
from PySide6.QtQuick import QQuickWindow  # noqa: E402,F401  (rootObjects 를 QQuickWindow 로 받기 위해)

from hmi import config  # noqa: E402
from hmi import geometry3d  # noqa: E402
from hmi.backend import HmiBackend  # noqa: E402
from hmi.sources import CameraReceiver, CanSniffer, DemoSource, UdpReceiver  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description="PACE HMI")
    p.add_argument("--scripted", action="store_true", help="(개발용) 정해진 가상 시나리오 재생")
    p.add_argument("--carla-port", type=int, default=config.CARLA_SCENE_PORT, help="CARLA 장면 UDP 포트")
    p.add_argument("--camera-port", type=int, default=config.CAMERA_PORT, help="CARLA 카메라 영상 TCP 포트")
    p.add_argument("--no-camera", action="store_true", help="CARLA 카메라 영상 수신 끄기 (그래픽 도로 화면만)")
    p.add_argument("--can", default=config.CAN_CHANNEL, help="CAN 채널 (기본 can0)")
    p.add_argument("--can-interface", default=config.CAN_INTERFACE, help="python-can interface (기본 socketcan)")
    p.add_argument("--no-can", action="store_true", help="CAN 수신 끄기 (UDP 만 사용)")
    p.add_argument("--udp-port", type=int, default=config.UDP_PORT, help="Safety ECU UDP 포트")
    p.add_argument("--windowed", action="store_true", help="창 모드")
    p.add_argument("--size", default="1600x900", help="창 모드 크기 WxH")
    p.add_argument("--mute", action="store_true", help="경고음 끄고 시작")
    p.add_argument("--demo-drive", action="store_true", help="DEMO 주행 모드로 시작")
    p.add_argument("--demo-distance", type=float, default=None, help="DEMO 전방 차량 시작 거리 (m)")
    p.add_argument("--tab", type=int, default=0, help="시작 탭 (0=PACE, 1=진단, 2=기록)")
    p.add_argument("--driver-view", action="store_true", help="운전자 화면(단순)으로 시작")
    p.add_argument("--night", action="store_true", help="야간 모드로 시작")
    p.add_argument("--screenshot", default=None, help="(테스트용) N초 후 화면 저장: path.png[:sec]")
    return p.parse_args()


def setup_graphics():
    """
    라즈베리파이 5 GPU 는 데스크톱 OpenGL 3.1 (GLSL 140) 로 잡히면 Qt Quick 3D 셰이더 일부(환경 조명)가 안 돌아감
    → OpenGL ES 3 로 강제. PACE_GL=desktop 으로 끌 수 있음.
    """
    if os.environ.get("PACE_GL", "es") != "es" or not sys.platform.startswith("linux"):
        return
    from PySide6.QtGui import QSurfaceFormat
    os.environ.setdefault("QSG_RHI_BACKEND", "opengl")
    os.environ.setdefault("QT_XCB_GL_INTEGRATION", "xcb_egl")      # X11 에서도 EGL(ES) 사용
    fmt = QSurfaceFormat.defaultFormat()
    fmt.setRenderableType(QSurfaceFormat.RenderableType.OpenGLES)
    fmt.setVersion(3, 1)
    fmt.setDepthBufferSize(24)
    fmt.setStencilBufferSize(8)
    QSurfaceFormat.setDefaultFormat(fmt)


def main():
    args = parse_args()
    setup_graphics()

    app = QGuiApplication(sys.argv)
    app.setApplicationName("PACE HMI")

    font = QFont()
    font.setFamilies(["Pretendard", "Noto Sans CJK KR", "NanumGothic", "Noto Sans KR", "sans-serif"])
    app.setFont(font)

    backend = HmiBackend(mode="SCRIPTED" if args.scripted else "LIVE")
    if args.mute:
        backend.toggleMute()

    workers = []
    if args.scripted:
        workers.append(DemoSource(backend.sink))
    else:
        workers.append(UdpReceiver(backend.sink, port=args.udp_port))
        workers.append(UdpReceiver(backend.sink, port=args.carla_port, kind="carla"))
        if not args.no_camera:
            workers.append(CameraReceiver(backend.sink, port=args.camera_port))
        if not args.no_can:
            workers.append(CanSniffer(backend.sink, channel=args.can, interface=args.can_interface))

    for w in workers:
        w.start()

    if args.demo_distance is not None:
        backend.setDemoStartDistance(args.demo_distance)
    if args.demo_drive:
        backend.toggleDemo()

    geometry3d.register()          # QML: import PaceHmi 1.0 → RibbonGeometry (3D 차선/도로)
    engine = QQmlApplicationEngine()
    engine.addImageProvider("carla", backend.camera_provider)
    engine.rootContext().setContextProperty("hmi", backend)
    engine.rootContext().setContextProperty("startWindowed", args.windowed)
    w, h = (int(x) for x in args.size.lower().split("x"))
    engine.rootContext().setContextProperty("startTab", args.tab)
    engine.rootContext().setContextProperty("startDriverView", args.driver_view)
    engine.rootContext().setContextProperty("startDark", args.night)
    engine.rootContext().setContextProperty("startWidth", w)
    engine.rootContext().setContextProperty("startHeight", h)
    engine.load(QUrl.fromLocalFile(os.path.join(HERE, "qml", "Main.qml")))

    if not engine.rootObjects():
        print("[HMI] QML 로드 실패")
        return 1

    if os.environ.get("PACE_HMI_FPS_LOG") == "1":
        window = engine.rootObjects()[0]
        frame_count = [0]
        frame_started = [time.monotonic()]

        def frame_swapped():
            frame_count[0] += 1

        def report_hmi_fps():
            now = time.monotonic()
            elapsed = now - frame_started[0]
            print(f"[PACE HMI PERF] render_fps={frame_count[0] / elapsed:.1f}", flush=True)
            frame_count[0] = 0
            frame_started[0] = now

        window.frameSwapped.connect(frame_swapped)
        fps_timer = QTimer()
        fps_timer.timeout.connect(report_hmi_fps)
        fps_timer.start(5000)

    # Ctrl+C 로 종료 가능하게
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    keepalive = QTimer()
    keepalive.start(200)
    keepalive.timeout.connect(lambda: None)

    if args.screenshot:
        path, _, sec = args.screenshot.partition(":")
        delay = float(sec) if sec else 2.0

        def grab():
            try:
                win = engine.rootObjects()[0]
                win.grabWindow().save(path)
                print("[HMI] screenshot saved:", path)
            finally:
                app.quit()

        QTimer.singleShot(int(delay * 1000), grab)

    code = app.exec()

    # QML 이 먼저 정리되도록 엔진을 backend 보다 먼저 삭제
    del engine

    for w in workers:
        w.stop()
    for w in workers:
        w.join(timeout=1.0)
    return code


if __name__ == "__main__":
    sys.exit(main())
