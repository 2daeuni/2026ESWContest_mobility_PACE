#!/usr/bin/env bash
# HMI 실행 (데스크톱 세션에서). 인자는 main.py 로 그대로 전달.
cd "$(dirname "$0")/.."

PY="/home/pi/PACE/.venv/bin/python3"
if [ ! -x "$PY" ]; then
    echo "[run_hmi] 오류: 가상환경 Python을 실행할 수 없습니다: $PY" >&2
    exit 1
fi

# Pi OS Bookworm 기본은 Wayland(labwc). 문제 있으면 xcb(Xwayland) 로 재시도.
if [ -z "$QT_QPA_PLATFORM" ] && [ -n "$WAYLAND_DISPLAY" ]; then
    export QT_QPA_PLATFORM=wayland
fi
export DISPLAY="${DISPLAY:-:0}"

"$PY" main.py "$@"
code=$?
if [ $code -ne 0 ] && [ "$QT_QPA_PLATFORM" = "wayland" ]; then
    echo "[run_hmi] wayland 실패 → xcb 로 재시도"
    QT_QPA_PLATFORM=xcb "$PY" main.py "$@"
fi
