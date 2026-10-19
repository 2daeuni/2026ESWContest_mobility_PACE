#!/usr/bin/env bash
# Raspberry Pi 5 (Raspberry Pi OS Bookworm 64-bit) HMI 설치
set -e
cd "$(dirname "$0")/.."

sudo apt-get update
# 한글 폰트 + Qt xcb 플랫폼 플러그인 의존성
sudo apt-get install -y fonts-noto-cjk libxcb-cursor0 python3-venv

# Safety ECU 와 같은 venv 를 써도 되고, 따로 만들어도 됨
if [ ! -d "$HOME/pace-venv" ]; then
    python3 -m venv "$HOME/pace-venv"
fi
"$HOME/pace-venv/bin/pip" install --upgrade pip
"$HOME/pace-venv/bin/pip" install -r requirements.txt

echo
echo "설치 완료. 실행:  ./scripts/run_hmi.sh          (실차)"
echo "              ./scripts/run_hmi.sh --demo-drive   (DEMO 주행으로 시작)"
