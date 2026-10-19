"""
PACE HMI 설정값.

CAN ID / 바이트 레이아웃은 safety_ecu/canbus/decoder.py, sender.py 및
pedal_ecu / haptic_ecu main.c 와 동일하게 맞춰져 있음.
CAN 규격이 바뀌면 이 파일과 hmi/can_decoder.py 만 수정하면 됨.
"""

# -------------------------------------------------
# Safety ECU -> HMI (UDP, JSON)
# -------------------------------------------------
UDP_BIND_HOST = "0.0.0.0"
UDP_PORT = 5600

# Safety ECU 패킷이 이 시간 이상 안 오면 "Safety ECU 끊김" 처리 (s)
SAFETY_UDP_TIMEOUT = 0.5

# BNS / PMS / Total Risk 의 최대값 (Safety ECU 기준 스케일)
# UI 에서는 0.00 ~ 1.00 으로 정규화해서 표시
SCORE_MAX = 100.0

# -------------------------------------------------
# CAN (수신 전용, HMI 는 버스에 절대 송신하지 않음)
# -------------------------------------------------
CAN_CHANNEL = "can0"
CAN_INTERFACE = "socketcan"

PEDAL_STATUS_ID = 0x100     # Pedal ECU  -> Safety / Motor, 10 ms
HAPTIC_COMMAND_ID = 0x110   # Safety ECU -> Haptic ECU, 10 ms
MOTOR_COMMAND_ID = 0x120    # Safety ECU -> Motor ECU, 10 ms
HAPTIC_STATUS_ID = 0x200    # Haptic ECU -> Safety ECU, 10 ms
MOTOR_STATUS_ID = 0x210     # Motor ECU  -> Safety ECU, 10 ms (PWM, 적용 제한, 차속 0.01 m/s, 상태, 고장)

# 프레임별 연결 판정 타임아웃 (s). 10 ms 주기 기준 넉넉히 잡음 (깜빡임 방지)
CAN_FRESH_TIMEOUT = 0.3

# -------------------------------------------------
# UI
# -------------------------------------------------
UI_REFRESH_HZ = 30           # QML 로 상태를 밀어주는 주기
EVENT_LOG_MAX = 100

# BRAKE NOW 전체 경고 조건: Risk L3 + 가속 페달 입력 중
BRAKE_NOW_ACCEL_MIN = 5      # %
BRAKE_NOW_BRAKE_MAX = 5      # %

# -------------------------------------------------
# CARLA 장면 (PC 의 carla_tools/carla_hmi_bridge.py → Pi HMI, UDP JSON)
# -------------------------------------------------
CARLA_SCENE_PORT = 5601
CARLA_SCENE_TIMEOUT = 1.5
CARLA_UI_STATE_TIMEOUT = 1.5  # UI telemetry expires independently of scene traffic.

# CARLA 추격 카메라 영상 (TCP, 브리지가 접속)
CAMERA_PORT = 5602
CAMERA_TIMEOUT = 2.0

# -------------------------------------------------
# 차량 이미지 (carla_tools/capture_vehicle_sprites.py 로 생성한 PNG)
#   assets/cars/<CARLA blueprint id>_<rear|front|hero>.png
#   assets/cars/default_<rear|front|hero>.png  ← 모르는 차종일 때
# -------------------------------------------------
EGO_BLUEPRINT = "vehicle.tesla.model3"      # 자차 이미지로 쓸 차종 (CARLA 에서는 실제 자차 차종 자동 사용)
DEMO_LEAD_BLUEPRINT = "vehicle.lincoln.mkz_2020"

# -------------------------------------------------
# DEMO 주행
# -------------------------------------------------
DEMO_START_DISTANCE = 1000.0   # m
DEMO_LEAD_SPEED = 0.0          # km/h (0 = 정지 장애물)

# -------------------------------------------------
# 차속 (노션 "CARLA Mode" 설계)
#   ACTUAL 모드: vehicle_speed = 0x210 MOTOR_STATUS 차속
#   CARLA  모드: vehicle_speed = CARLA 속도
#   Safety ECU state.vehicle_speed 단위: m/s (0x210 규격과 동일). km/h 로 계산하면 "km/h" 로 변경
# -------------------------------------------------
SAFETY_SPEED_UNIT = "m/s"
