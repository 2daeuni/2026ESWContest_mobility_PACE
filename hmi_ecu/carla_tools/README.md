# carla_tools (CARLA PC 에서 실행)

라즈베리파이 HMI 의 도로 장면을 실제 CARLA 주행과 똑같이 보이게 하는 도구 2개.
둘 다 기존 CARLA 주행 스크립트(G29 → CARLA, CARLA ↔ Safety ECU UDP)는 건드리지 않음.

## 준비 (PC)

```bash
pip install carla==0.9.16 numpy pillow     # CARLA 서버 버전과 같은 carla 패키지
```

## 1. 차량 이미지 만들기 (최초 1회)

CARLA 차량 3D 모델을 직접 렌더링해서 배경 투명 PNG 로 저장. 시뮬레이션에 나오는 차와 HMI 화면의 차가 똑같아짐.

```bash
# CARLA 서버만 켜 둔 상태에서 (주행 스크립트는 끄고)
python capture_vehicle_sprites.py --default vehicle.tesla.model3
# 일부 차종만:
python capture_vehicle_sprites.py --filter "vehicle.tesla.*,vehicle.lincoln.*,vehicle.audi.*" --default vehicle.tesla.model3
```

- 결과: `hmi_ecu/assets/cars/<차종>_rear.png`, `_front.png`, `_hero.png`, `default_*.png`
- 이 폴더를 라즈베리파이 `hmi_ecu/assets/cars/` 로 복사 → HMI 재시작
- `--default` 는 자차/모르는 차종에 쓸 기본 이미지. 자차 차종으로 지정 권장
- 실행 중 잠깐 synchronous 모드로 바꿨다가 끝나면 원래 설정으로 복구
- 차량을 지면 위 120 m 에 띄워서 촬영하므로 맵 오브젝트에 가려지지 않음

## 2. 주행 장면 브리지 (주행할 때마다)

```bash
python carla_hmi_bridge.py --hmi-host <라즈베리파이 IP>
python carla_hmi_bridge.py --list                          # 자차 못 찾으면 차량 목록 확인
python carla_hmi_bridge.py --hmi-host <IP> --actor-id 123  # 자차 직접 지정
```

- 같은 CARLA 서버에 읽기 전용 클라이언트로 붙음 (차량 제어 / world.tick 안 함)
- 20 Hz 로 자차 기준 주변 차량·보행자 위치, 차선 중심선(곡선), 앞차 거리, 자차 속도/기어 송신 → Pi UDP 5601
- 자차 찾는 순서: `--actor-id` → role_name 이 `hero/ego/ego_vehicle` → 관전자 카메라에 가장 가까운 차량
  - 주행 스크립트에서 자차 blueprint 에 `bp.set_attribute("role_name", "hero")` 해두면 확실함

### 실제 도로 / 건물 (기본 켜짐)

- 브리지 시작 시 CARLA 맵 전체 차선(실제 차선 표시 종류·색 포함)과 건물·벽·펜스·기둥 위치를 한 번 읽음
- 자차 주변 90 m 를 잘라서 HMI 로 보냄 (40 m 움직일 때마다 갱신, 2 초마다 재전송) → HMI 3D 화면에 실제 도로 모양, 교차로, 노란 중앙선, 건물/벽이 그대로 나옴
- 자차가 인도·광장처럼 차선 밖에 있으면 자차 차선 강조를 끄고 실제 주변만 표시
- 충돌 센서: 벽·차에 부딪히면 HMI 이벤트 로그에 "CARLA 충돌 감지" + 자차가 잠깐 빨갛게
- 끄기: `--no-map`
- CARLA 없이 확인: Pi 에서 `python3 tools/fake_carla_town.py` (교차로 + 건물 가짜 도시)

### 카메라 영상 (기본 꺼짐, `--camera real` 로 켬)

브리지가 자차 뒤·위에 카메라를 붙여 **CARLA 가 실제로 렌더링한 화면**을 Pi 로 보냄 (TCP 5602, JPEG 960×540, 20 fps).
HMI 는 그 영상 위에 자차 주행 경로(위험 단계 색)와 앞차 표시·거리를 정확히 겹쳐 그림.

```bash
python carla_hmi_bridge.py --hmi-host <IP> --camera real      # 실제 CARLA 화면
python carla_hmi_bridge.py --hmi-host <IP> --camera clean     # 테슬라식: 실제 3D 차 + 단색 도로, 건물/하늘 제거
python carla_hmi_bridge.py --hmi-host <IP>                    # 영상 없이 장면 데이터만 (기본)
python carla_hmi_bridge.py --hmi-host <IP> --cam-quality 65   # 와이파이가 느리면 JPEG 품질/해상도 낮춤 (--cam-width 640 --cam-height 360)
```

- 카메라 위치: `--cam-back 4` (뒤 범퍼에서 m) `--cam-up 2.6` (지붕 위 m) `--cam-fov 75`
- 브리지를 끄면(Ctrl+C) 붙였던 카메라는 CARLA 월드에서 자동 삭제
- Windows 방화벽: 브리지 → Pi 로 나가는 연결이라 보통 문제 없음. Pi 쪽에 방화벽을 켰다면 TCP 5602 허용
- 영상이 0.6 초 이상 끊기면 HMI 는 자동으로 그래픽 도로 화면으로 돌아감 (DEMO 주행 중에도 그래픽 화면)

## HMI 쪽 확인

- 장면 오른쪽 아래 태그가 `CARLA` 로 바뀜, SYSTEM STATUS 의 CARLA (Scene) Connected
- 진단 탭 `CARLA_SCENE` 행: 수신 주기(≈20 Hz), 차량 수, 이미지 개수
- 영상이 들어오면 장면 오른쪽 아래 `● CARLA CAM 20 fps`, 진단 탭 `CARLA_CAMERA` 행 (TCP 5602, 해상도·KB)
- CARLA 없이 화면만 확인: Pi 에서 `python3 tools/fake_carla_scene.py`
- CARLA 없이 영상 경로 확인: Pi 에서 `python3 tools/fake_camera_sender.py` (간단한 그림 + 오버레이 정렬 확인용)
