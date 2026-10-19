# PACE HMI (UI Display ECU)

Raspberry Pi 5 에서 동작하는 PACE 운전자 HMI. PySide6 + QML, 1600×900 기준 디자인을 화면 크기에 맞춰 비율 스케일.

## 구조

```
Pedal ECU ──0x100──┐
Haptic ECU ─0x200──┤        (수신 전용, 송신 없음)
Safety ECU ─0x110──┼── can0 ───────────────► HMI CanSniffer ─┐
           └0x120──┘                                         ├─► state_merge ─► QML
Safety ECU ── UDP 127.0.0.1:5600 (JSON, 30 Hz) ► UdpReceiver ─┤
CARLA PC (carla_hmi_bridge) ── UDP :5601 (장면, 20 Hz) ────────┤
                            └─ TCP :5602 (카메라 영상 JPEG) ──┘
```

- HMI 는 독립 프로세스. Safety ECU 루프와 분리되어 HMI 가 죽거나 느려져도 안전 제어에 영향 없음
- CAN: SocketCAN 은 여러 프로세스가 동시에 열 수 있음 → Safety ECU 와 충돌 없음. HMI 는 버스에 아무것도 송신하지 않음 (Bus Load 영향 0)
- UDP: BNS / PMS / Total / 거리 / TTC / 차속 / Fail-Safe 등 CAN 에 없는 Safety ECU 내부 상태 수신
- 데이터 우선순위
  - 페달, 햅틱/모터 명령 → 실제 버스 값(CAN) 우선, 없으면 UDP
  - 위험도 → Safety ECU UDP 우선, UDP 없으면 0x110 의 risk_level(1~3 만) 사용
  - 데이터 없음/타임아웃 → "정상"으로 표시하지 않고 OFFLINE / 판단 대기로 표시

## Safety ECU 연동 (필수 변경 2가지)

1. `safety_ecu/hmi_publisher.py` 추가
2. `safety_ecu/main.py` 에 3군데 추가 (import / 생성, 명령 송신 블록의 `system_logger.log(...)` 바로 아래 `hmi_publisher.publish(...)`, finally 에서 `close()`)
   - 변경 내용은 `docs/safety_ecu_main.patch` 참고
   - non-blocking UDP, 30 Hz 제한, 실패 시 무시 → Safety 10 ms 주기에 영향 없음
   - `SystemState` 에 `vehicle_speed`, `ttc`, `gear`, `carla_connected`, `relative_speed` 속성을 추가하면 자동으로 HMI 에 표시됨 (getattr 사용)

## 설치 / 실행 (Pi 5, Raspberry Pi OS Bookworm 64-bit)

```bash
cd hmi_ecu
./scripts/install_pi.sh            # 한글 폰트, PySide6, python-can
./scripts/setup_can.sh             # can0 500 kbps (Safety ECU 쪽에서 이미 했다면 생략)

./scripts/run_hmi.sh               # 실차/벤치 (전체화면)
./scripts/run_hmi.sh --windowed    # 창 모드
```

- 종료: `Esc`, 전체화면 전환: `F11`, 탭 전환: `Tab`
- 부팅 시 자동 실행: `scripts/pace-hmi.desktop` 의 경로 수정 후 `~/.config/autostart/` 에 복사
- 실행 순서 무관 (HMI 먼저 켜도 됨, Safety ECU 재시작해도 자동 재연결)

## 도로 장면

### 3D 주행 화면 (왼쪽 클러스터 · PACE 장면 카드 · 운전자 화면)

- 테슬라 주행 화면 방식: 실제 3D 자동차 모델을 Qt Quick 3D 로 라즈베리파이 5 GPU 에서 렌더링
  - 정차: 자차를 위에서 내려다보는 화면 / 주행(CARLA·DEMO·센서 데이터 있음): 자차 뒤·위 추격 시점으로 자동 전환
  - 자차: 파란색 실제 차 모델 (브레이크 밟으면 브레이크등 점등)
  - 주변 차: CARLA 차량을 실제 위치·방향에 회색 3D 차로, 트럭/버스/이륜차/보행자는 형상별로
  - 전방 차량: 위험 단계 색으로 강조 + 거리 라벨, 자차 앞~전방 차량 사이 위험 영역 색 표시, L2 이상이면 자차 차선 색 변경
  - 점선 차선은 속도에 맞춰 흐름
- 파일: `qml/World3D.qml`(장면, 한 번만 만들어 공유) / `qml/DriveVis3D.qml`(시점) / `qml/EgoCar3D.qml` / `qml/Vehicle3D.qml` / `hmi/geometry3d.py`(차선 형상)
- 차 모델: `assets/models/*.mesh` (출처·라이선스 `assets/models/LICENSE.md`, CC BY 4.0)
  - 다른 차 모델(.glb)로 바꾸기: `python3 tools/build_vehicle_models.py --car 자차.glb --traffic 주변차.glb`
- GPU 가 없는 환경(소프트웨어 렌더링)에서는 자동으로 기존 2D 도로 화면으로 대체

### CARLA 실제 영상 (CARLA 연결 시)

- 브리지가 보내는 CARLA 추격 카메라 영상을 그대로 표시 + AR 오버레이
  - 자차 주행 경로: 정상 파랑, L1 노랑, L2 주황, L3 빨강 (앞차 뒤에서 끊김)
  - 앞차: 모서리 브래킷 + 거리 (Safety ECU 거리값 우선), L2 이상이면 강조, L3 화면 테두리 빨강
- 화면: `qml/CameraView.qml`, 수신: `hmi/sources.py` `CameraReceiver`, 설정 `config.CAMERA_PORT/CAMERA_TIMEOUT`
- 영상이 없거나 DEMO 주행 중이면 아래 그래픽 화면으로 자동 전환
- `--no-camera` 로 영상 수신 끄기

### 그래픽 도로 화면 (영상 없을 때)

- 자차 뒤·위 추격 카메라 시점으로 원근 투영, 자차 차선은 위험 단계 색으로 표시
- 장면 소스 (오른쪽 아래 태그)
  - `CARLA`  : `carla_tools/carla_hmi_bridge.py` 가 보낸 실제 주변 차량/보행자/곡선 차선 (`carla_tools/README.md`)
  - `DEMO`   : DEMO 주행의 가상 전방 차량
  - `SENSOR` : 거리값만 있을 때 정면 앞차
- 차량 이미지: `assets/cars/` 의 PNG 사용 (CARLA 차량을 직접 렌더링해서 생성: `carla_tools/capture_vehicle_sprites.py`)
  - 없으면 기본 도형으로 그림
- 카메라 높이/거리: `qml/RoadScene.qml` 의 `camH`, `camBack`

## DEMO 주행 (하단 DEMO 버튼, 또는 F2)

- **CARLA 가 연결돼 있으면 (DEMO · CARLA)**: 가상 차량 대신 CARLA 에서 실제로 운전하는 값으로 HMI 가 위험 판정
  - 거리 = 앞차 / 벽·건물 중 가까운 것, TTC = 거리 / 접근 속도, 페달 = Pedal ECU(있으면) 또는 CARLA throttle·brake
  - L1~L3 배너, 경고음, BRAKE NOW 전체 화면, 충돌 알림까지 실제와 똑같이 동작 (`hmi/local_risk.py`)
  - DEMO 를 끄면(LIVE) 위험 판정은 Safety ECU 값만 사용 → 실제 운행 구조 그대로
- **CARLA 가 없으면**: 아래 가상 DEMO

- 전방 차량을 1 km (선택: 300 m / 100 m) 앞에 두고 실제 페달로 운전
  - Pedal ECU(0x100) 가 들어오면 실제 페달 사용, 없으면 키보드 `↑` 가속 / `↓`·`Space` 브레이크 / `R` 리셋
- 차량 모델: 1.6 t, 최대 120 kW, 풀브레이크 약 0.9 g, 공기저항/구름저항 포함 → 페달 밟는 만큼 실제 차처럼 가감속
- 전방 차량 속도: 정지 / 30 / 60 km/h
- 위험 판정: Safety ECU 와 같은 구조 (거리·TTC·페달 3조건 중 2개, 상승 3회/하강 5회 확인, L3 시 출력 30% 제한)
  - 임계값은 아직 확정 전이라 `hmi/demo_drive.py` 의 `THRESHOLDS` 에 임시값 (팀 확정값으로 교체)
- 충돌 시 충돌 화면 + RESET
- DEMO 판정은 화면 안에서만 사용. CAN 으로 햅틱/모터 명령을 보내지 않음 (Safety ECU 명령과 충돌 방지)

## 청각 경고 · 운전자 화면 · 반응 시간 기록

- 경고음: L1 차임 1회 / L2 2연속 비프 1.2 s 반복 / L3·BRAKE NOW 고음 연속 / Fail-Safe 저음 / 충돌 저음
  - `assets/sounds/*.wav` 는 시작 시 자동 생성, 헤더 `소리 ON/OFF` 또는 M 키
  - Pi 5 는 3.5 mm 단자 없음 → HDMI 모니터 스피커 / USB / 블루투스
- 판정 근거 표 (SAFETY 카드): 단계별 거리·TTC·페달 조건 충족 표시. Safety ECU 는 `state.risk_conditions` 로 전송
- 운전자 화면 (`V` 키, `--driver-view`): 큰 경고 + 큰 도로 장면 + 단계만
- 야간 모드 (`N` 키, `--night`)
- 기록 탭: L2 이상 경고마다 가속 해제·브레이크 반응 시간 자동 측정, 경고음 ON/OFF 별 평균 비교, `logs/reaction_log.csv` 저장
- Diagnostics 성능 줄: FPS, CPU, 상태 계산 시간, 페달 입력 → 화면 지연

## 옵션

| 옵션 | 설명 |
|---|---|
| `--driver-view` / `--night` / `--mute` | 운전자 화면 / 야간 / 경고음 끔으로 시작 |
| `--demo-drive` | DEMO 주행 모드로 시작 (`--demo-distance 300` 으로 시작 거리 지정) |
| `--carla-port 5601` | CARLA 장면 UDP 포트 |
| `--camera-port 5602` / `--no-camera` | CARLA 카메라 영상 TCP 포트 / 영상 끄기 |
| `--scripted` | (개발용) 정해진 40초 시나리오 재생 |
| `--can vcan0` | CAN 채널 변경 (기본 `can0`) |
| `--no-can` | UDP 만 사용 |
| `--udp-port 5600` | Safety ECU UDP 포트 |
| `--windowed --size 1280x720` | 창 모드 / 크기 |
| `--tab 1` | 진단 탭으로 시작 |

## 하드웨어 없이 통합 테스트

```bash
./scripts/setup_can.sh vcan                          # vcan0 생성

# 0) CARLA 장면만 가짜로: python3 tools/fake_carla_scene.py

# 1) HMI 단독: 가상 ECU 전체 + 가상 Safety UDP
python3 tools/sim_bus.py --channel vcan0 --mode full
python3 main.py --can vcan0 --windowed

# 2) 실제 Safety ECU 코드 포함: 가상 Pedal/Haptic ECU 만 송신
#    (safety_ecu/main.py 의 "can0" 을 잠시 "vcan0" 으로 바꾸거나 vcan 이름을 can0 으로 생성)
python3 tools/sim_bus.py --channel vcan0 --mode pedal

# 3) Safety ECU 다운 시 fallback 확인: CAN 만 송신
python3 tools/sim_bus.py --channel vcan0 --mode can
```

단위 테스트: `python3 -m unittest discover -s tests`

## 화면 구성

- 왼쪽 클러스터: 차속, 기어(P/R/N/D), READY / FAIL-SAFE / OFFLINE, 전방 감지 영역, 가속·브레이크 페달 게이지, 변화율, 데이터 출처
- PACE 탭
  - 경고 배너 (단계별 문구 + TTC) + 원근 도로 장면 (거리 기반 전방 차량 위치)
  - PACE STATUS: Risk Level 링, Total Risk, BNS / PMS / 거리, L0~L3 단계
  - PEDAL (0x100) / MOTOR COMMAND (0x120) / HAPTIC (0x110, 0x200)
  - SAFETY / SYSTEM STATUS (ECU 별 연결) / EVENT LOG
- 진단 탭: CAN ID 별 수신 주기(Hz)·Age·Alive·Raw, UDP 수신 상태, 전체 이벤트, 병합 상태값 → 통합 디버깅용
- BRAKE NOW 오버레이: Risk L3 + 가속 페달 ≥ 5% + 브레이크 ≤ 5% 일 때 자동 표시, 조건 해제 시 자동 해제

## 설정 변경 위치

- CAN ID, 타임아웃, 점수 스케일(BNS/PMS 최대값, 기본 100), BRAKE NOW 조건 → `hmi/config.py`
- CAN 바이트 레이아웃 → `hmi/can_decoder.py`
- 문구 / Fail-Safe 사유 한글 → `hmi/state_merge.py`
- 색상 / 레이아웃 → `qml/Theme.qml`, `qml/*.qml`

## 미확정 사항 (팀 확인 필요)

- 0x210 MOTOR_STATUS: motor_ecu main.c / safety_ecu decoder.py 규격 반영 (PWM, 적용 제한, 차속 0.01 m/s, 상태, 고장 비트)
- Safety ECU `state.vehicle_speed` 단위는 노션 규격대로 m/s 로 가정 (`HMIPublisher(speed_unit="km/h")` 로 변경 가능)
- 차속 Source: Safety ECU 값 → 없으면 0x210 (Motor ECU) → CARLA 장면이 있으면 CARLA (노션 CARLA Mode)
- BNS / PMS / Total 스케일: 0~100 가정 (`score_max` 로 UDP 에 같이 전송, 바뀌면 publisher 인자만 수정)
- 차속 / TTC / 기어: 현재 Safety ECU `SystemState` 에 값이 없음 (None) → CARLA / G29 연동 코드에서 state 에 넣어주면 자동 표시
