#!/usr/bin/env bash
# CAN 인터페이스 설정 (Safety ECU 와 동일 설정, 500 kbps)
#   ./scripts/setup_can.sh          → can0 실제 하드웨어
#   ./scripts/setup_can.sh vcan     → vcan0 가상 CAN (하드웨어 없이 테스트)
set -e
if [ "$1" = "vcan" ]; then
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan 2>/dev/null || true
    sudo ip link set up vcan0
    ip -details link show vcan0
else
    sudo ip link set can0 down 2>/dev/null || true
    sudo ip link set can0 up type can bitrate 500000
    ip -details link show can0
fi
