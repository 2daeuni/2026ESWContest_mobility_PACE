import carla
import pygame
import numpy as np

import socket
import json
import math
import random
import time
import weakref


# ============================================================
# CARLA / WINDOW
# ============================================================

CARLA_HOST = "127.0.0.1"
CARLA_PORT = 2000
MAP_NAME = "Town05"

# 주행용 Pygame 화면은 모니터 기본 해상도로 FULLSCREEN 실행
# CARLA RGB 센서는 1920x1080으로 유지한 뒤 전체화면 크기로 확대한다.
# 4K 센서를 직접 생성하는 것보다 GPU/메모리 부하가 훨씬 적다.
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CLIENT_FPS = 60

CAMERA_GAMMA = 2.2


# ============================================================
# HERO VEHICLE FIXED SPAWN - Town05 parking lot
#
# 원하는 위치로 바꾸려면 아래 4개 값만 수정
# ============================================================

HERO_SPAWN_X = 4.10
HERO_SPAWN_Y = -41.13
HERO_SPAWN_Z = 0.90
HERO_SPAWN_YAW = -180.0


# ============================================================
# AI ROAD TRAFFIC
# ============================================================

TRAFFIC_VEHICLE_COUNT = 40
TRAFFIC_MANAGER_PORT = 8000

# 내 Mustang 바로 주변에는 AI 차량을 만들지 않음
TRAFFIC_MIN_DISTANCE_FROM_HERO = 12.0

# Traffic Manager 설정
TRAFFIC_GLOBAL_DISTANCE = 2.5

# 멀리 있는 AI 차량 physics 부하를 줄여 FPS 방어
TRAFFIC_HYBRID_PHYSICS = True
TRAFFIC_HYBRID_RADIUS = 70.0


# ============================================================
# FRONT OBSTACLE DISTANCE
#
# 정면 50 m 안에 들어온 장애물만 거리값을 사용한다.
# 50 m 밖이거나 감지 물체가 없으면 NO SIGN 로 표시.
# ============================================================

FRONT_DISTANCE_RANGE = 50.0
FRONT_RADAR_HORIZONTAL_FOV = 14.0
FRONT_RADAR_VERTICAL_FOV = 10.0
FRONT_RADAR_POINTS_PER_SECOND = 1800
FRONT_RADAR_SENSOR_TICK = 0.05
# Safety corridor filtering (independent from display distance)
SAFETY_RADAR_AZIMUTH_DEG = 4.0
SAFETY_RADAR_ALTITUDE_DEG = 3.0
SAFETY_RADAR_MIN_POINTS = 3
SAFETY_RADAR_FRESHNESS_S = 0.2


# ============================================================
# DISTANCE STABILIZATION
#
# Radar는 한 프레임마다 다른 점을 맞출 수 있어서
# 정지 상태에서도 몇 cm씩 값이 흔들릴 수 있다.
#
# 아래 설정으로:
# 1) 한 프레임의 가까운 여러 점을 평균
# 2) 최근 여러 프레임의 중앙값 사용
# 3) 5 cm 단위로 표시
# 4) 아주 작은 변화는 무시
# ============================================================

DISTANCE_CLOSEST_POINT_COUNT = 5
DISTANCE_HISTORY_SIZE = 7
DISTANCE_DISPLAY_STEP = 0.05
DISTANCE_DEADBAND = 0.08

# 한두 프레임 Radar return이 끊겨도 바로 NO SIGN으로 바뀌지 않도록 유지
DISTANCE_NO_SIGN_DELAY = 0.25


# ============================================================
# NIGHT LOW-VISIBILITY SCENARIO
#
# N : 밤 / 낮 환경 토글
# O : 100 m 가상 장애물 시나리오 시작 / Reset
# L : 자차 헤드라이트 ON/OFF
#
# 기존 주행/UDP/CAN/카메라/물리 설정은 변경하지 않는다.
# ============================================================

NIGHT_SCENARIO_DISTANCE_M = 100.0
NIGHT_REAL_SPAWN_DISTANCE_M = 50.0
NIGHT_HEADLIGHT_ON_DISTANCE_M = 20.0
NIGHT_SUN_ALTITUDE_ANGLE = -90.0
NIGHT_OBSTACLE_COLOR = "5,5,5"


# ============================================================
# DYNAMIC JUNCTION SCENARIO
#
# I : arm/cancel the next-junction sudden-vehicle scenario
#
# 버튼을 누른 뒤 "다음 사용 가능한 교차로"를 자동 탐색한다.
# EGO 기준 오른쪽 측면 도로에서 차량을 대기시킨 뒤,
# 교차로 접근 시 오른쪽 차량이 직진으로 돌발 진입한다.
# ============================================================

JUNCTION_SEARCH_DISTANCE = 180.0
JUNCTION_PREPARE_DISTANCE = 60.0
JUNCTION_TRIGGER_DISTANCE = 18.0
JUNCTION_SPAWN_BACK_DISTANCE = 4.0

# 40대 교통량 때문에 한 spawn point가 막힐 수 있으므로
# 오른쪽 진입도로 뒤쪽 여러 지점을 순서대로 시도한다.
JUNCTION_SPAWN_BACK_CANDIDATES_M = (
    4.0,
    7.0,
    10.0,
    13.0,
    16.0,
    20.0,
    24.0,
)

JUNCTION_NPC_INITIAL_SPEED = 8.0       # m/s = 28.8 km/h
JUNCTION_NPC_THROTTLE = 1.0
JUNCTION_NPC_LIFETIME = 10.0


# ============================================================
# UDP : Raspberry Pi -> PC
# format = accel,brake,steer,gear
# example = 70,0,-0.3520,D
#
# gear:
#   P = Parking
#   R = Reverse
#   D = Drive
# ============================================================

UDP_IP = "0.0.0.0"
UDP_PORT = 5005
FAILSAFE_TIMEOUT = 0.5


# ============================================================
# UDP : PC(CARLA) -> Raspberry Pi Safety ECU
#
# JSON / 10 Hz
#
# {
#   "source": "CARLA",
#   "sequence": 0,
#   "vehicle_speed_kmh": 30.0,
#   "obstacle_id": 42,
#   "distance_m": 15.0,
#   "closing_speed_mps": 3.0,
#   "ttc_s": 5.0
# }
#
# 장애물이 없으면 obstacle_id / distance_m /
# closing_speed_mps / ttc_s 를 모두 null 로 송신한다.
# ============================================================

RPI_IP = "192.168.0.37"
RPI_TELEMETRY_PORT = 5006

TELEMETRY_SEND_HZ = 10.0
TELEMETRY_SEND_INTERVAL = 1.0 / TELEMETRY_SEND_HZ

# Independent HMI UI state destination (send only; no bind).
RPI_UI_IP = "192.168.50.2"
RPI_UI_PORT = 5601
UI_SEND_HZ = 10.0
UI_SEND_INTERVAL = 1.0 / UI_SEND_HZ

# O 키 가상 야간 장애물은 실제 actor 생성 전에도
# Safety ECU에서 동일한 장애물로 추적할 수 있도록 고정 양수 ID를 사용.
NIGHT_SCENARIO_OBSTACLE_ID = 900001
RADAR_OBSTACLE_ID = 900002

# Safety ECU용 실제 전방 차량 탐색 범위.
SAFETY_FRONT_MAX_DISTANCE_M = 50.0
SAFETY_FRONT_LATERAL_LIMIT_M = 2.5


# ============================================================
# ENGINE / ACCELERATION
# ============================================================

# CARLA 기본 Mustang 물리 특성은 유지하고,
# 엔진 토크만 약 18% 올려 초반/중간 가속을 조금 더 강하게 한다.
# 1.00 = CARLA 순정
# 1.10 = 약간 강하게
# 1.18 = 추천
# 1.25 = 꽤 강하게
ENGINE_TORQUE_MULTIPLIER = 1.30

# D-only automatic-transmission creep; Safety remains the final authority.
CREEP_ENABLED = True
CREEP_TARGET_KMH = 10.0
CREEP_MAX_THROTTLE = 0.18
CREEP_ACCEL_DEADZONE = 0.02
CREEP_BRAKE_DEADZONE = 0.02


# ============================================================
# HARDWARE INPUT
# ============================================================

ACCEL_DEADZONE = 2
BRAKE_DEADZONE = 2

# CARLA 기본 차량 느낌을 살리기 위해 선형 매핑
ACCEL_EXPONENT = 1.0

# 기본 저속 조향 게인은 아래 SPEED-SENSITIVE STEERING 설정에서 사용
STEERING_DEADZONE = 0.015

# 실제 입력의 작은 노이즈는 줄이되,
# 가속/브레이크 반응은 이전보다 빠르게 설정
THROTTLE_ALPHA = 0.60
BRAKE_ALPHA = 0.90
STEER_ALPHA = 0.45

# 브레이크 입력 증폭
# 1.00 = 기존과 동일
# 1.15 = 같은 입력에서 약 15% 더 강하게 제동
BRAKE_INPUT_GAIN = 1.20


# ============================================================
# SPEED-SENSITIVE STEERING
#
# 저속에서는 주차/회전을 위해 기존 조향량을 유지하고,
# 속도가 올라갈수록 조향 민감도를 자동으로 낮춘다.
# ============================================================

STEERING_GAIN_LOW_SPEED = 0.82
STEERING_GAIN_MID_SPEED = 0.60
STEERING_GAIN_HIGH_SPEED = 0.50

STEERING_LOW_SPEED_KMH = 20.0
STEERING_MID_SPEED_KMH = 60.0
STEERING_HIGH_SPEED_KMH = 100.0


# ============================================================
# REALISTIC LANE FOLLOWING ASSIST (LFA)
#
# J : LFA ON / OFF
#
# 실제 양산차 느낌을 목표로:
# - 항상 중앙으로 세게 끌지 않음
# - 운전자 조향 우선
# - 고속일수록 더 멀리 보고 더 약하게 보정
# - 작은 오차에는 거의 개입하지 않음
# - 교차로 / 차선변경 의도에서는 개입 억제
# - 개입/해제를 부드럽게 ramp 처리
# ============================================================

LANE_ASSIST_ENABLED_DEFAULT = False

LFA_MIN_SPEED_KMH = 8.0
LFA_MAX_SPEED_KMH = 130.0

# speed-adaptive lookahead
LFA_LOOKAHEAD_LOW_M = 8.0
LFA_LOOKAHEAD_MID_M = 18.0
LFA_LOOKAHEAD_HIGH_M = 32.0

LFA_LOW_SPEED_KMH = 35.0
LFA_MID_SPEED_KMH = 60.0
LFA_HIGH_SPEED_KMH = 100.0

# speed-adaptive max steering correction
LFA_MAX_CORR_LOW = 0.16
LFA_MAX_CORR_MID = 0.13
LFA_MAX_CORR_HIGH = 0.10

# speed-adaptive gains
LFA_LAT_GAIN_LOW = 0.080
LFA_LAT_GAIN_MID = 0.050
LFA_LAT_GAIN_HIGH = 0.032

LFA_HEAD_GAIN_LOW = 0.55
LFA_HEAD_GAIN_MID = 0.46
LFA_HEAD_GAIN_HIGH = 0.34

# near center: don't hunt
LFA_LATERAL_DEADBAND_M = 0.10
LFA_HEADING_DEADBAND_DEG = 1.2

# smoothing
LFA_FILTER_ALPHA_LOW = 0.10
LFA_FILTER_ALPHA_MID = 0.075
LFA_FILTER_ALPHA_HIGH = 0.055

LFA_RATE_LOW = 0.55
LFA_RATE_MID = 0.34
LFA_RATE_HIGH = 0.20

# ------------------------------------------------------------
# High-speed stabilizer (> 35 km/h)
#
# v10 used the angle to one far waypoint plus lateral error.
# At higher speeds those two terms can overlap and cause hunting.
#
# v11 separates:
#   1) lateral position error
#   2) lane-heading error
#   3) road-curvature feed-forward
#   4) lateral-error derivative damping
# ------------------------------------------------------------

LFA_HIGH_SPEED_CONTROLLER_START_KMH = 35.0

# derivative damping: larger at high speed
LFA_D_GAIN_LOW = 0.000
LFA_D_GAIN_MID = 0.018
LFA_D_GAIN_HIGH = 0.030

LFA_LATERAL_RATE_FILTER_ALPHA = 0.12
LFA_MAX_LATERAL_RATE_MPS = 3.0

# preview heading uses a near and a far waypoint
LFA_PREVIEW_HEADING_FAR_WEIGHT = 0.65

# curvature feed-forward helps hold curves without late corrections
LFA_CURVATURE_FF_LOW = 0.00
LFA_CURVATURE_FF_MID = 0.32
LFA_CURVATURE_FF_HIGH = 0.45

LFA_APPROX_WHEELBASE_M = 2.70
LFA_APPROX_MAX_TIRE_STEER_RAD = 0.60

# driver override
LFA_OVERRIDE_START = 0.18
LFA_OVERRIDE_FULL = 0.38
LFA_OVERRIDE_HOLD_SEC = 1.0

# junction release / reacquire
LFA_JUNCTION_REACQUIRE_SEC = 0.8

# soft engagement
LFA_ENGAGE_RAMP_SEC = 0.7

# if the vehicle is already far from the lane center,
# treat it as a lane-change / driver-intent situation
LFA_MAX_LATERAL_ERROR_M = 1.15

# lane-width sanity range
LFA_MIN_LANE_WIDTH_M = 2.4
LFA_MAX_LANE_WIDTH_M = 5.5


# ============================================================
# GEAR
# ============================================================

GEAR_CHANGE_SPEED_KMH = 1.0


# ============================================================
# CONTROL MODE
# ============================================================

control_mode = "HARDWARE"


# ============================================================
# UTIL
# ============================================================


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def calculate_creep_throttle(gear, accelerator, brake, speed_kmh):
    """Use current driver inputs, before smoothing, to release creep immediately."""
    if (not CREEP_ENABLED or gear != "D"
            or accelerator > CREEP_ACCEL_DEADZONE
            or brake > CREEP_BRAKE_DEADZONE):
        return 0.0
    ratio = clamp(
        (CREEP_TARGET_KMH - speed_kmh) / CREEP_TARGET_KMH,
        0.0, 1.0,
    )
    return CREEP_MAX_THROTTLE * ratio


def median(values):
    if not values:
        return None

    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2

    if n % 2 == 1:
        return ordered[mid]

    return (ordered[mid - 1] + ordered[mid]) * 0.5


def get_speed_kmh(vehicle):
    velocity = vehicle.get_velocity()
    return 3.6 * math.sqrt(
        velocity.x * velocity.x
        + velocity.y * velocity.y
        + velocity.z * velocity.z
    )


def get_forward_speed_mps(actor, reference_forward=None):
    """Actor velocity projected onto a forward direction."""
    velocity = actor.get_velocity()

    if reference_forward is None:
        reference_forward = (
            actor.get_transform().get_forward_vector()
        )

    return (
        velocity.x * reference_forward.x
        + velocity.y * reference_forward.y
        + velocity.z * reference_forward.z
    )


def find_front_vehicle_for_safety(
    world,
    ego_vehicle,
    max_distance=SAFETY_FRONT_MAX_DISTANCE_M,
    lateral_limit=SAFETY_FRONT_LATERAL_LIMIT_M,
):
    """
    Find the closest real vehicle in the ego-forward corridor.

    Returns None or:
    {
        "actor": actor,
        "distance_m": bumper_gap,
        "closing_speed_mps": signed closing speed,
        "ttc_s": float or None,
    }

    This is intentionally actor-based instead of radar-point-based so the
    Safety ECU receives a stable obstacle_id.
    """
    ego_transform = ego_vehicle.get_transform()
    ego_location = ego_transform.location
    ego_forward = ego_transform.get_forward_vector()
    ego_right = ego_transform.get_right_vector()
    ego_velocity_forward = get_forward_speed_mps(
        ego_vehicle,
        ego_forward,
    )

    ego_front_extent = max(
        0.0,
        float(ego_vehicle.bounding_box.extent.x),
    )

    closest = None

    for actor in world.get_actors().filter("vehicle.*"):
        if actor.id == ego_vehicle.id:
            continue

        try:
            location = actor.get_location()

            dx = location.x - ego_location.x
            dy = location.y - ego_location.y
            dz = location.z - ego_location.z

            longitudinal = (
                dx * ego_forward.x
                + dy * ego_forward.y
                + dz * ego_forward.z
            )

            lateral = (
                dx * ego_right.x
                + dy * ego_right.y
                + dz * ego_right.z
            )

            # Behind ego or clearly outside the ego path.
            if longitudinal <= 0.0:
                continue

            if abs(lateral) > lateral_limit:
                continue

            target_half_length = max(
                0.0,
                float(actor.bounding_box.extent.x),
            )

            bumper_gap = max(
                0.0,
                longitudinal
                - ego_front_extent
                - target_half_length,
            )

            if bumper_gap > max_distance:
                continue

            target_velocity_forward = get_forward_speed_mps(
                actor,
                ego_forward,
            )

            closing_speed = (
                ego_velocity_forward
                - target_velocity_forward
            )

            if closing_speed > 0.0:
                ttc = bumper_gap / closing_speed
            else:
                ttc = None

            candidate = {
                "actor": actor,
                "distance_m": float(bumper_gap),
                "closing_speed_mps": float(closing_speed),
                "ttc_s": (
                    float(ttc)
                    if ttc is not None
                    else None
                ),
            }

            if (
                closest is None
                or candidate["distance_m"]
                < closest["distance_m"]
            ):
                closest = candidate

        except RuntimeError:
            # Actor may disappear between world snapshot and property access.
            continue

    return closest


def find_rear_vehicle_for_safety(
    world,
    ego_vehicle,
    max_distance=SAFETY_FRONT_MAX_DISTANCE_M,
    lateral_limit=SAFETY_FRONT_LATERAL_LIMIT_M,
):
    """Find the closest vehicle in the ego-rear corridor."""
    ego_transform = ego_vehicle.get_transform()
    ego_location = ego_transform.location
    ego_forward = ego_transform.get_forward_vector()
    ego_right = ego_transform.get_right_vector()

    ego_velocity_forward = get_forward_speed_mps(
        ego_vehicle, ego_forward
    )
    ego_rear_extent = max(
        0.0, float(ego_vehicle.bounding_box.extent.x)
    )

    closest = None

    for actor in world.get_actors().filter("vehicle.*"):
        if actor.id == ego_vehicle.id:
            continue

        try:
            location = actor.get_location()

            dx = location.x - ego_location.x
            dy = location.y - ego_location.y
            dz = location.z - ego_location.z

            longitudinal = (
                dx * ego_forward.x
                + dy * ego_forward.y
                + dz * ego_forward.z
            )
            lateral = (
                dx * ego_right.x
                + dy * ego_right.y
                + dz * ego_right.z
            )

            # Ignore vehicles in front or outside the rear corridor.
            if longitudinal >= 0.0:
                continue

            if abs(lateral) > lateral_limit:
                continue

            target_half_length = max(
                0.0, float(actor.bounding_box.extent.x)
            )

            bumper_gap = max(
                0.0,
                -longitudinal
                - ego_rear_extent
                - target_half_length,
            )

            if bumper_gap > max_distance:
                continue

            target_velocity_forward = get_forward_speed_mps(
                actor, ego_forward
            )

            # Positive when the rear gap is closing.
            closing_speed = (
                target_velocity_forward
                - ego_velocity_forward
            )

            ttc = (
                bumper_gap / closing_speed
                if closing_speed > 0.0
                else None
            )

            candidate = {
                "actor": actor,
                "distance_m": float(bumper_gap),
                "closing_speed_mps": float(closing_speed),
                "ttc_s": float(ttc) if ttc is not None else None,
            }

            if (
                closest is None
                or candidate["distance_m"] < closest["distance_m"]
            ):
                closest = candidate

        except RuntimeError:
            continue

    return closest


def build_safety_telemetry(
    world,
    ego_vehicle,
    sequence,
    virtual_obstacle_active,
    virtual_obstacle_distance,
    virtual_obstacle_materialized=False,
    gear="D",
    front_distance_sensor=None,
):
    """Create the exact JSON object expected by Pi Safety ECU."""
    vehicle_speed_kmh = max(
        0.0,
        float(get_speed_kmh(ego_vehicle)),
    )

    obstacle_id = None
    distance_m = None
    closing_speed_mps = None
    ttc_s = None

    # Read front Radar distance.
    # This value is not used for Safety decisions yet.
    radar_distance_m = None

    if gear == "D" and front_distance_sensor is not None:
        radar_distance_m = front_distance_sensor.get_safety_corridor_distance()

        if radar_distance_m is not None:
            radar_distance_m = float(radar_distance_m)

            if not math.isfinite(radar_distance_m):
                radar_distance_m = None

            elif not 0.0 <= radar_distance_m <= FRONT_DISTANCE_RANGE:
                radar_distance_m = None

    # Virtual preview is valid only before the real obstacle is spawned.
    # After handoff, use current actor detection, including an absent obstacle.
    # A consumed zero-distance preview must never remain latched after collision.
    if (
        gear == "D"
        and virtual_obstacle_active
        and virtual_obstacle_distance is not None
        and virtual_obstacle_distance > 0.0
        and not virtual_obstacle_materialized
    ):
        distance_m = max(
            0.0,
            float(virtual_obstacle_distance),
        )

        ego_forward = (
            ego_vehicle
            .get_transform()
            .get_forward_vector()
        )

        closing_speed_mps = max(
            0.0,
            float(
                get_forward_speed_mps(
                    ego_vehicle,
                    ego_forward,
                )
            ),
        )

        obstacle_id = NIGHT_SCENARIO_OBSTACLE_ID

        if closing_speed_mps > 0.0:
            ttc_s = (
                distance_m
                / closing_speed_mps
            )


    else:
        detected_vehicle = None

        if gear == "D":
            detected_vehicle = find_front_vehicle_for_safety(
                world,
                ego_vehicle,
            )
        elif gear == "R":
            detected_vehicle = find_rear_vehicle_for_safety(
                world,
                ego_vehicle,
            )

        if detected_vehicle is not None:
            # Preserve existing vehicle Actor tracking.
            obstacle_id = int(detected_vehicle["actor"].id)
            distance_m = float(detected_vehicle["distance_m"])
            closing_speed_mps = float(
                detected_vehicle["closing_speed_mps"]
            )
            ttc_s = detected_vehicle["ttc_s"]

        elif gear == "D" and radar_distance_m is not None:
            # Radar fallback when no vehicle Actor is selected.
            obstacle_id = RADAR_OBSTACLE_ID
            distance_m = float(radar_distance_m)

            ego_forward = (
                ego_vehicle
                .get_transform()
                .get_forward_vector()
            )

            closing_speed_mps = max(
                0.0,
                float(
                    get_forward_speed_mps(
                        ego_vehicle,
                        ego_forward,
                    )
                ),
            )

            if closing_speed_mps > 0.0:
                ttc_s = distance_m / closing_speed_mps
            else:
                ttc_s = None


    return {
        "source": "CARLA",
        "sequence": int(sequence) & 0xFFFF,
        "vehicle_speed_kmh": vehicle_speed_kmh,
        "obstacle_id": obstacle_id,
        "distance_m": distance_m,
        "closing_speed_mps": closing_speed_mps,
        "ttc_s": ttc_s,
    }


def normalize_angle_deg(angle):
    while angle >= 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


def lerp(a, b, t):
    t = clamp(t, 0.0, 1.0)
    return a + (b - a) * t


def move_toward(current, target, max_delta):
    if target > current:
        return min(current + max_delta, target)

    if target < current:
        return max(current - max_delta, target)

    return current


def get_lfa_speed_parameters(speed_kmh):
    if speed_kmh <= LFA_LOW_SPEED_KMH:
        return {
            "lookahead": LFA_LOOKAHEAD_LOW_M,
            "max_corr": LFA_MAX_CORR_LOW,
            "lat_gain": LFA_LAT_GAIN_LOW,
            "head_gain": LFA_HEAD_GAIN_LOW,
            "d_gain": LFA_D_GAIN_LOW,
            "ff_gain": LFA_CURVATURE_FF_LOW,
            "alpha": LFA_FILTER_ALPHA_LOW,
            "rate": LFA_RATE_LOW,
        }

    if speed_kmh <= LFA_MID_SPEED_KMH:
        t = (
            (speed_kmh - LFA_LOW_SPEED_KMH)
            / (LFA_MID_SPEED_KMH - LFA_LOW_SPEED_KMH)
        )

        return {
            "lookahead": lerp(
                LFA_LOOKAHEAD_LOW_M,
                LFA_LOOKAHEAD_MID_M,
                t,
            ),
            "max_corr": lerp(
                LFA_MAX_CORR_LOW,
                LFA_MAX_CORR_MID,
                t,
            ),
            "lat_gain": lerp(
                LFA_LAT_GAIN_LOW,
                LFA_LAT_GAIN_MID,
                t,
            ),
            "head_gain": lerp(
                LFA_HEAD_GAIN_LOW,
                LFA_HEAD_GAIN_MID,
                t,
            ),
            "d_gain": lerp(
                LFA_D_GAIN_LOW,
                LFA_D_GAIN_MID,
                t,
            ),
            "ff_gain": lerp(
                LFA_CURVATURE_FF_LOW,
                LFA_CURVATURE_FF_MID,
                t,
            ),
            "alpha": lerp(
                LFA_FILTER_ALPHA_LOW,
                LFA_FILTER_ALPHA_MID,
                t,
            ),
            "rate": lerp(
                LFA_RATE_LOW,
                LFA_RATE_MID,
                t,
            ),
        }

    if speed_kmh <= LFA_HIGH_SPEED_KMH:
        t = (
            (speed_kmh - LFA_MID_SPEED_KMH)
            / (LFA_HIGH_SPEED_KMH - LFA_MID_SPEED_KMH)
        )

        return {
            "lookahead": lerp(
                LFA_LOOKAHEAD_MID_M,
                LFA_LOOKAHEAD_HIGH_M,
                t,
            ),
            "max_corr": lerp(
                LFA_MAX_CORR_MID,
                LFA_MAX_CORR_HIGH,
                t,
            ),
            "lat_gain": lerp(
                LFA_LAT_GAIN_MID,
                LFA_LAT_GAIN_HIGH,
                t,
            ),
            "head_gain": lerp(
                LFA_HEAD_GAIN_MID,
                LFA_HEAD_GAIN_HIGH,
                t,
            ),
            "d_gain": lerp(
                LFA_D_GAIN_MID,
                LFA_D_GAIN_HIGH,
                t,
            ),
            "ff_gain": lerp(
                LFA_CURVATURE_FF_MID,
                LFA_CURVATURE_FF_HIGH,
                t,
            ),
            "alpha": lerp(
                LFA_FILTER_ALPHA_MID,
                LFA_FILTER_ALPHA_HIGH,
                t,
            ),
            "rate": lerp(
                LFA_RATE_MID,
                LFA_RATE_HIGH,
                t,
            ),
        }

    return {
        "lookahead": LFA_LOOKAHEAD_HIGH_M,
        "max_corr": LFA_MAX_CORR_HIGH,
        "lat_gain": LFA_LAT_GAIN_HIGH,
        "head_gain": LFA_HEAD_GAIN_HIGH,
        "d_gain": LFA_D_GAIN_HIGH,
        "ff_gain": LFA_CURVATURE_FF_HIGH,
        "alpha": LFA_FILTER_ALPHA_HIGH,
        "rate": LFA_RATE_HIGH,
    }


def get_lfa_override_scale(driver_steer):
    amount = abs(driver_steer)

    if amount <= LFA_OVERRIDE_START:
        return 1.0

    if amount >= LFA_OVERRIDE_FULL:
        return 0.0

    return 1.0 - (
        (
            amount - LFA_OVERRIDE_START
        )
        / (
            LFA_OVERRIDE_FULL
            - LFA_OVERRIDE_START
        )
    )


def calculate_realistic_lfa(
    world,
    vehicle,
    speed_kmh,
    applied_gear,
):
    """
    Production-like lane-following assist using CARLA map ground truth.

    Returns:
        raw_correction
        lateral_error_m
        heading_error_deg
        state
        speed_params
    """
    params = get_lfa_speed_parameters(speed_kmh)

    if applied_gear != "D":
        return 0.0, 0.0, 0.0, "NOT_DRIVE", params

    if speed_kmh < LFA_MIN_SPEED_KMH:
        return 0.0, 0.0, 0.0, "LOW_SPEED", params

    if speed_kmh > LFA_MAX_SPEED_KMH:
        return 0.0, 0.0, 0.0, "HIGH_SPEED", params

    world_map = world.get_map()
    vehicle_transform = vehicle.get_transform()
    vehicle_location = vehicle_transform.location

    current_wp = world_map.get_waypoint(
        vehicle_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )

    if current_wp is None:
        return 0.0, 0.0, 0.0, "NO_LANE", params

    if (
        current_wp.lane_width < LFA_MIN_LANE_WIDTH_M
        or current_wp.lane_width > LFA_MAX_LANE_WIDTH_M
    ):
        return 0.0, 0.0, 0.0, "BAD_LANE", params

    if current_wp.is_junction:
        return 0.0, 0.0, 0.0, "JUNCTION", params

    next_wps = current_wp.next(params["lookahead"])

    if len(next_wps) != 1:
        return 0.0, 0.0, 0.0, "JUNCTION", params

    target_wp = next_wps[0]

    if target_wp.is_junction:
        return 0.0, 0.0, 0.0, "JUNCTION", params

    lane_center = current_wp.transform.location

    center_dx = lane_center.x - vehicle_location.x
    center_dy = lane_center.y - vehicle_location.y
    center_dz = lane_center.z - vehicle_location.z

    right = vehicle_transform.get_right_vector()

    lateral_error = (
        center_dx * right.x
        + center_dy * right.y
        + center_dz * right.z
    )

    # Do not fight a clear lane-change / strong driver maneuver.
    if abs(lateral_error) > LFA_MAX_LATERAL_ERROR_M:
        return (
            0.0,
            lateral_error,
            0.0,
            "LANE_CHANGE",
            params,
        )

    target_location = target_wp.transform.location

    target_dx = target_location.x - vehicle_location.x
    target_dy = target_location.y - vehicle_location.y
    target_dz = target_location.z - vehicle_location.z

    forward = vehicle_transform.get_forward_vector()

    if speed_kmh <= LFA_HIGH_SPEED_CONTROLLER_START_KMH:
        # ----------------------------------------------------
        # LOW SPEED:
        # Keep the v10 behavior that already worked well.
        # ----------------------------------------------------
        local_forward = (
            target_dx * forward.x
            + target_dy * forward.y
            + target_dz * forward.z
        )

        local_right = (
            target_dx * right.x
            + target_dy * right.y
            + target_dz * right.z
        )

        target_angle_rad = math.atan2(
            local_right,
            max(local_forward, 0.05),
        )

        heading_error_deg = math.degrees(
            target_angle_rad
        )

        correction = (
            params["head_gain"] * target_angle_rad
            + params["lat_gain"] * lateral_error
        )

    else:
        # ----------------------------------------------------
        # MID / HIGH SPEED:
        #
        # Do NOT use the angle from the car to the far target
        # as the heading error. That angle already contains
        # cross-track error, so adding lateral_error again can
        # cause left-right hunting.
        #
        # Instead, separately control:
        #   - lane heading
        #   - lateral position
        # and add curvature feed-forward.
        # ----------------------------------------------------

        near_distance = max(
            5.0,
            params["lookahead"] * 0.45,
        )

        near_wps = current_wp.next(
            near_distance
        )

        if len(near_wps) != 1:
            return (
                0.0,
                lateral_error,
                0.0,
                "JUNCTION",
                params,
            )

        near_wp = near_wps[0]

        if near_wp.is_junction:
            return (
                0.0,
                lateral_error,
                0.0,
                "JUNCTION",
                params,
            )

        near_yaw = near_wp.transform.rotation.yaw
        far_yaw = target_wp.transform.rotation.yaw
        current_lane_yaw = (
            current_wp.transform.rotation.yaw
        )
        vehicle_yaw = (
            vehicle_transform.rotation.yaw
        )

        near_to_far_delta = normalize_angle_deg(
            far_yaw - near_yaw
        )

        preview_lane_yaw = (
            near_yaw
            + LFA_PREVIEW_HEADING_FAR_WEIGHT
            * near_to_far_delta
        )

        heading_error_deg = normalize_angle_deg(
            preview_lane_yaw - vehicle_yaw
        )

        heading_error_rad = math.radians(
            heading_error_deg
        )

        # Road curvature estimate from lane yaw change.
        curve_yaw_change_deg = normalize_angle_deg(
            far_yaw - current_lane_yaw
        )

        curvature_rad_per_m = (
            math.radians(curve_yaw_change_deg)
            / max(params["lookahead"], 1.0)
        )

        # Bicycle-model-like feed-forward converted to a
        # normalized CARLA steering contribution.
        curve_steer_rad = math.atan(
            LFA_APPROX_WHEELBASE_M
            * curvature_rad_per_m
        )

        curvature_ff = (
            params["ff_gain"]
            * curve_steer_rad
            / max(
                LFA_APPROX_MAX_TIRE_STEER_RAD,
                0.10,
            )
        )

        correction = (
            params["head_gain"] * heading_error_rad
            + params["lat_gain"] * lateral_error
            + curvature_ff
        )

    if (
        abs(lateral_error) < LFA_LATERAL_DEADBAND_M
        and abs(heading_error_deg) < LFA_HEADING_DEADBAND_DEG
    ):
        correction = 0.0

    correction = clamp(
        correction,
        -params["max_corr"],
        params["max_corr"],
    )

    return (
        correction,
        lateral_error,
        heading_error_deg,
        "ACTIVE",
        params,
    )


def map_accelerator(percent):
    if percent <= ACCEL_DEADZONE:
        return 0.0

    pedal = clamp(percent / 100.0, 0.0, 1.0)
    return clamp(pedal ** ACCEL_EXPONENT, 0.0, 1.0)


def get_speed_steering_gain(speed_kmh):
    """속도에 따라 조향 민감도를 부드럽게 낮춘다."""

    if speed_kmh <= STEERING_LOW_SPEED_KMH:
        return STEERING_GAIN_LOW_SPEED

    if speed_kmh <= STEERING_MID_SPEED_KMH:
        ratio = (
            (speed_kmh - STEERING_LOW_SPEED_KMH)
            / (STEERING_MID_SPEED_KMH - STEERING_LOW_SPEED_KMH)
        )

        return (
            STEERING_GAIN_LOW_SPEED
            + (STEERING_GAIN_MID_SPEED - STEERING_GAIN_LOW_SPEED) * ratio
        )

    if speed_kmh <= STEERING_HIGH_SPEED_KMH:
        ratio = (
            (speed_kmh - STEERING_MID_SPEED_KMH)
            / (STEERING_HIGH_SPEED_KMH - STEERING_MID_SPEED_KMH)
        )

        return (
            STEERING_GAIN_MID_SPEED
            + (STEERING_GAIN_HIGH_SPEED - STEERING_GAIN_MID_SPEED) * ratio
        )

    return STEERING_GAIN_HIGH_SPEED


# ============================================================
# CARLA DEFAULT PHYSICS
#
# manual_control.py와 동일하게 기본 차량 물리는 그대로 두고
# sweep wheel collision만 활성화한다.
# ============================================================


def enable_default_vehicle_physics(vehicle):
    try:
        physics = vehicle.get_physics_control()

        # manual_control.py와 동일하게 sweep wheel collision 활성화
        physics.use_sweep_wheel_collision = True

        # 나머지 물리값은 CARLA 기본값을 그대로 유지하고
        # 엔진 torque curve의 토크 값(y)만 일정 비율로 증가시킨다.
        original_curve = list(physics.torque_curve)

        if ENGINE_TORQUE_MULTIPLIER != 1.0 and original_curve:
            boosted_curve = []

            for point in original_curve:
                boosted_curve.append(
                    carla.Vector2D(
                        x=point.x,
                        y=point.y * ENGINE_TORQUE_MULTIPLIER,
                    )
                )

            physics.torque_curve = boosted_curve

        vehicle.apply_physics_control(physics)

        print(
            f"Engine torque multiplier: x{ENGINE_TORQUE_MULTIPLIER:.2f}"
        )

    except Exception as exc:
        print("Physics setup warning:", exc)


# ============================================================
# CAMERA MANAGER
#
# manual_control.py 방식:
# - spectator를 매 프레임 이동하지 않음
# - 차량에 RGB camera sensor를 attach
# - CARLA 엔진이 카메라를 따라가게 함
# ============================================================


class CameraManager:
    def __init__(self, parent_actor, width, height, gamma=2.2):
        self.parent = parent_actor
        self.world = parent_actor.get_world()
        self.width = width
        self.height = height
        self.gamma = gamma

        self.sensor = None
        self.surface = None
        self.mode = "DRIVER"

        bound_x = 0.5 + self.parent.bounding_box.extent.x
        bound_y = 0.5 + self.parent.bounding_box.extent.y
        bound_z = 0.5 + self.parent.bounding_box.extent.z

        Attachment = carla.AttachmentType

        # manual_control.py의 차량 카메라 위치를 그대로 참고
        self.transforms = {
            "CHASE": (
                carla.Transform(
                    carla.Location(
                        x=-2.0 * bound_x,
                        y=0.0 * bound_y,
                        z=2.0 * bound_z,
                    ),
                    carla.Rotation(pitch=8.0),
                ),
                Attachment.SpringArmGhost,
            ),
            "DRIVER": (
                carla.Transform(
                    carla.Location(
                        x=0.8 * bound_x,
                        y=0.0 * bound_y,
                        z=1.3 * bound_z,
                    )
                ),
                Attachment.Rigid,
            ),
        }

        camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(self.width))
        camera_bp.set_attribute("image_size_y", str(self.height))

        if camera_bp.has_attribute("gamma"):
            camera_bp.set_attribute("gamma", str(self.gamma))

        self.camera_bp = camera_bp

    def start(self, mode="DRIVER"):
        self.mode = mode
        self._spawn_sensor()

    def toggle(self):
        self.mode = "CHASE" if self.mode == "DRIVER" else "DRIVER"
        self._spawn_sensor()
        return self.mode

    def _spawn_sensor(self):
        self._destroy_sensor_only()

        transform, attachment = self.transforms[self.mode]

        self.sensor = self.world.spawn_actor(
            self.camera_bp,
            transform,
            attach_to=self.parent,
            attachment_type=attachment,
        )

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda image: CameraManager._parse_image(weak_self, image)
        )

    def _destroy_sensor_only(self):
        if self.sensor is not None:
            try:
                self.sensor.stop()
            except Exception:
                pass

            try:
                self.sensor.destroy()
            except Exception:
                pass

            self.sensor = None
            self.surface = None

    def destroy(self):
        self._destroy_sensor_only()

    def render(self, display):
        if self.surface is not None:
            display_size = display.get_size()

            if self.surface.get_size() == display_size:
                display.blit(self.surface, (0, 0))
            else:
                # 1920x1080 RGB 영상을 모니터 전체 크기로 확대.
                # 4K 모니터도 16:9이므로 화면 비율이 깨지지 않는다.
                fullscreen_surface = pygame.transform.scale(
                    self.surface,
                    display_size,
                )
                display.blit(fullscreen_surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if self is None:
            return

        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))

        # BGRA -> RGB
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        self.surface = pygame.surfarray.make_surface(
            array.swapaxes(0, 1)
        )



# ============================================================
# FRONT DISTANCE SENSOR
#
# Radar Sensor는 차량 bounding box의 가장 앞쪽에 붙인다.
#
# Radar:
#   0 ~ 50 m 앞 장애물 거리 측정
#
# Collision Sensor:
#   실제 충돌 순간에는 Radar가 물체 안쪽으로 들어가거나
#   return이 사라질 수 있으므로 거리값을 강제로 0.00 m 처리
# ============================================================


class FrontDistanceSensor:
    def __init__(self, parent_actor):
        self.parent = parent_actor
        self.world = parent_actor.get_world()

        self.radar_sensor = None
        self.collision_sensor = None

        self.distance = None

        # 최근 Radar 거리값을 저장해서 중앙값 필터에 사용
        self.distance_history = []

        # 마지막 정상 Radar return 시간
        self.last_valid_time = 0.0

        # 충돌 직후 잠깐 0.00 m를 유지
        self.collision_zero_until = 0.0

        # Independent safety-corridor Radar measurement
        self.safety_corridor_distance = None
        self.last_safety_valid_time = 0.0

        bbox = self.parent.bounding_box

        # ----------------------------------------------------
        # 차량 좌표계 기준 실제 bounding box 가장 앞부분
        #
        # bounding_box.location.x까지 포함해야
        # 차량 모델마다 센서 위치가 정확해진다.
        # ----------------------------------------------------

        front_x = (
            bbox.location.x
            + bbox.extent.x
            + 0.03
        )

        center_z = (
            bbox.location.z
            + bbox.extent.z * 0.55
        )

        # ====================================================
        # RADAR
        # ====================================================

        radar_bp = (
            self.world
            .get_blueprint_library()
            .find("sensor.other.radar")
        )

        if radar_bp.has_attribute("horizontal_fov"):
            radar_bp.set_attribute(
                "horizontal_fov",
                str(FRONT_RADAR_HORIZONTAL_FOV),
            )

        if radar_bp.has_attribute("vertical_fov"):
            radar_bp.set_attribute(
                "vertical_fov",
                str(FRONT_RADAR_VERTICAL_FOV),
            )

        if radar_bp.has_attribute("points_per_second"):
            radar_bp.set_attribute(
                "points_per_second",
                str(FRONT_RADAR_POINTS_PER_SECOND),
            )

        if radar_bp.has_attribute("range"):
            radar_bp.set_attribute(
                "range",
                str(FRONT_DISTANCE_RANGE),
            )

        if radar_bp.has_attribute("sensor_tick"):
            radar_bp.set_attribute(
                "sensor_tick",
                str(FRONT_RADAR_SENSOR_TICK),
            )

        radar_transform = carla.Transform(
            carla.Location(
                x=front_x,
                y=bbox.location.y,
                z=center_z,
            ),
            carla.Rotation(
                pitch=0.0,
                yaw=0.0,
                roll=0.0,
            ),
        )

        self.radar_sensor = self.world.spawn_actor(
            radar_bp,
            radar_transform,
            attach_to=self.parent,
            attachment_type=carla.AttachmentType.Rigid,
        )

        weak_self = weakref.ref(self)

        self.radar_sensor.listen(
            lambda radar_data: FrontDistanceSensor._on_radar(
                weak_self,
                radar_data,
            )
        )

        # ====================================================
        # COLLISION SENSOR
        #
        # 실제로 장애물과 접촉하면 DIST 0.00m가 되도록 사용
        # ====================================================

        collision_bp = (
            self.world
            .get_blueprint_library()
            .find("sensor.other.collision")
        )

        self.collision_sensor = self.world.spawn_actor(
            collision_bp,
            carla.Transform(),
            attach_to=self.parent,
        )

        self.collision_sensor.listen(
            lambda event: FrontDistanceSensor._on_collision(
                weak_self,
                event,
            )
        )

        print(
            "Radar position (vehicle local): "
            f"x={front_x:.2f}, "
            f"y={bbox.location.y:.2f}, "
            f"z={center_z:.2f}"
        )

    @staticmethod
    def _on_radar(weak_self, radar_data):
        self = weak_self()

        if self is None:
            return

        now = time.monotonic()

        valid_depths = []
        safety_depths = []

        for detection in radar_data:
            depth = float(detection.depth)

            if not math.isfinite(depth):
                continue

            if 0.02 < depth <= FRONT_DISTANCE_RANGE:
                # Existing display-distance processing
                valid_depths.append(depth)

                # Safety corridor: only returns near the forward centerline
                azimuth_deg = math.degrees(float(detection.azimuth))
                altitude_deg = math.degrees(float(detection.altitude))

                if (
                    math.isfinite(azimuth_deg)
                    and math.isfinite(altitude_deg)
                    and abs(azimuth_deg) <= SAFETY_RADAR_AZIMUTH_DEG
                    and abs(altitude_deg) <= SAFETY_RADAR_ALTITUDE_DEG
                ):
                    safety_depths.append(depth)

        # Safety measurement is independent from the display filter.
        # Do not retain an old detection when this frame has no valid target.
        if len(safety_depths) >= SAFETY_RADAR_MIN_POINTS:
            safety_depths.sort()

            self.safety_corridor_distance = median(
                safety_depths[:DISTANCE_CLOSEST_POINT_COUNT]
            )
            self.last_safety_valid_time = now
        else:
            self.safety_corridor_distance = None

        # ----------------------------------------------------
        # 감지 점이 있는 경우
        # ----------------------------------------------------
        if valid_depths:
            valid_depths.sort()

            # 가장 가까운 한 점만 쓰면 작은 노이즈에 민감하므로
            # 가까운 N개 점의 중앙값을 이 프레임 대표 거리로 사용
            closest = valid_depths[
                :DISTANCE_CLOSEST_POINT_COUNT
            ]

            frame_distance = median(closest)

            if frame_distance is not None:
                self.distance_history.append(
                    frame_distance
                )

                if (
                    len(self.distance_history)
                    > DISTANCE_HISTORY_SIZE
                ):
                    self.distance_history.pop(0)

                # 최근 여러 프레임의 중앙값
                stable_distance = median(
                    self.distance_history
                )

                if stable_distance is not None:
                    # 화면 표시를 5 cm 단위로 고정
                    quantized = round(
                        stable_distance
                        / DISTANCE_DISPLAY_STEP
                    ) * DISTANCE_DISPLAY_STEP

                    quantized = clamp(
                        quantized,
                        0.0,
                        FRONT_DISTANCE_RANGE,
                    )

                    # 8 cm 미만의 작은 흔들림은 기존 표시값 유지
                    if self.distance is None:
                        self.distance = quantized

                    elif (
                        abs(
                            quantized
                            - self.distance
                        )
                        >= DISTANCE_DEADBAND
                    ):
                        self.distance = quantized

                self.last_valid_time = now

        # ----------------------------------------------------
        # 감지 점이 없는 경우
        #
        # Radar가 한두 프레임 빠지는 경우가 있으므로
        # 바로 NO SIGN으로 바꾸지 않는다.
        # ----------------------------------------------------
        else:
            if (
                now
                - self.last_valid_time
                >= DISTANCE_NO_SIGN_DELAY
            ):
                self.distance = None
                self.distance_history.clear()

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()

        if self is None:
            return

        # 충돌 직후 0.5초 동안 0.00m 유지
        self.collision_zero_until = (
            time.monotonic()
            + 0.5
        )

        # 이전 Radar 값이 충돌 후 다시 튀어나오지 않도록 초기화
        self.distance = 0.0
        self.distance_history.clear()

    def get_distance(self):
        # 실제 충돌 중/직후에는 무조건 0.00m
        if (
            time.monotonic()
            < self.collision_zero_until
        ):
            return 0.0

        return self.distance

    def get_safety_distance(self):
        """Return only fresh Radar distance for Safety ECU."""
        now = time.monotonic()

        # A recent collision must remain visible as zero distance.
        if now < self.collision_zero_until:
            return 0.0

        # No fresh Radar measurement.
        if (
            self.last_valid_time <= 0.0
            or now - self.last_valid_time > 0.2
        ):
            return None

        return self.get_distance()

    def get_safety_corridor_distance(self):
        """Return a fresh Radar target in the forward safety corridor."""
        now = time.monotonic()

        if now < self.collision_zero_until:
            return 0.0

        if self.safety_corridor_distance is None:
            return None

        if (
            self.last_safety_valid_time <= 0.0
            or now - self.last_safety_valid_time > SAFETY_RADAR_FRESHNESS_S
        ):
            return None

        return self.safety_corridor_distance

    def destroy(self):
        if self.radar_sensor is not None:
            try:
                self.radar_sensor.stop()
            except Exception:
                pass

            try:
                self.radar_sensor.destroy()
            except Exception:
                pass

            self.radar_sensor = None

        if self.collision_sensor is not None:
            try:
                self.collision_sensor.stop()
            except Exception:
                pass

            try:
                self.collision_sensor.destroy()
            except Exception:
                pass

            self.collision_sensor = None


# ============================================================
# KEYBOARD CONTROL
#
# manual_control.py의 키보드 입력 감각을 최대한 그대로 사용
# ============================================================


class KeyboardVehicleControl:
    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer_cache = 0.0

    def reset(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer_cache = 0.0

    def update(self, keys, milliseconds):
        # throttle - manual_control.py와 동일한 방식
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            self.throttle = min(self.throttle + 0.1, 1.0)
        else:
            self.throttle = 0.0

        # brake
        if keys[pygame.K_DOWN] or keys[pygame.K_s]:
            self.brake = min(self.brake + 0.2, 1.0)
        else:
            self.brake = 0.0

        # steering
        steer_increment = 5e-4 * milliseconds

        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            if self.steer_cache > 0:
                self.steer_cache = 0.0
            else:
                self.steer_cache -= steer_increment

        elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            if self.steer_cache < 0:
                self.steer_cache = 0.0
            else:
                self.steer_cache += steer_increment

        else:
            self.steer_cache = 0.0

        self.steer_cache = clamp(self.steer_cache, -0.7, 0.7)
        steer = round(self.steer_cache, 1)

        # 기존 프로젝트처럼 SPACE는 풀 브레이크
        if keys[pygame.K_SPACE]:
            self.brake = 1.0
            self.throttle = 0.0

        return self.throttle, self.brake, steer


# ============================================================
# VEHICLE / TRAFFIC SPAWN
# ============================================================


def distance_2d(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    return math.sqrt(dx * dx + dy * dy)


def spawn_mustang(world):
    blueprints = world.get_blueprint_library()

    candidates = blueprints.filter("vehicle.ford.mustang")

    if not candidates:
        candidates = blueprints.filter("vehicle.audi.tt")

    if not candidates:
        raise RuntimeError("Sports car blueprint not found")

    blueprint = candidates[0]

    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", "hero")

    spawn_transform = carla.Transform(
        carla.Location(
            x=HERO_SPAWN_X,
            y=HERO_SPAWN_Y,
            z=HERO_SPAWN_Z,
        ),
        carla.Rotation(
            pitch=0.0,
            yaw=HERO_SPAWN_YAW,
            roll=0.0,
        ),
    )

    vehicle = world.try_spawn_actor(
        blueprint,
        spawn_transform,
    )

    # 지면 겹침 때문에 실패하는 경우만 Z + 0.4 m로 재시도
    if vehicle is None:
        retry_transform = carla.Transform(
            carla.Location(
                x=HERO_SPAWN_X,
                y=HERO_SPAWN_Y,
                z=HERO_SPAWN_Z + 0.40,
            ),
            carla.Rotation(
                pitch=0.0,
                yaw=HERO_SPAWN_YAW,
                roll=0.0,
            ),
        )

        vehicle = world.try_spawn_actor(
            blueprint,
            retry_transform,
        )

    if vehicle is None:
        raise RuntimeError(
            "Fixed Mustang spawn failed. "
            "해당 위치가 다른 actor/prop과 겹치는지 확인하세요."
        )

    vehicle.set_autopilot(False)
    enable_default_vehicle_physics(vehicle)

    return vehicle


def get_traffic_blueprints(world):
    """
    Traffic Manager용 4륜 차량만 사용.
    자전거/오토바이는 제외.
    """
    result = []

    for blueprint in world.get_blueprint_library().filter("vehicle.*"):
        try:
            if blueprint.has_attribute("number_of_wheels"):
                wheels = int(
                    blueprint.get_attribute("number_of_wheels")
                )

                if wheels != 4:
                    continue

            result.append(blueprint)

        except Exception:
            continue

    return result


def spawn_traffic_vehicles(
    world,
    traffic_manager,
    hero_vehicle,
    count,
):
    """
    Town05 도로 spawn point에 AI 차량을 생성하고
    Traffic Manager autopilot을 활성화한다.
    """

    traffic_actors = []

    blueprints = get_traffic_blueprints(world)

    if not blueprints:
        print("WARNING: No traffic vehicle blueprints found")
        return traffic_actors

    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)

    hero_location = hero_vehicle.get_location()

    usable_points = [
        point
        for point in spawn_points
        if distance_2d(
            point.location,
            hero_location,
        ) >= TRAFFIC_MIN_DISTANCE_FROM_HERO
    ]

    for spawn_point in usable_points:
        if len(traffic_actors) >= count:
            break

        blueprint = random.choice(blueprints)

        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute(
                "role_name",
                "autopilot",
            )

        if blueprint.has_attribute("color"):
            colors = blueprint.get_attribute(
                "color"
            ).recommended_values

            if colors:
                blueprint.set_attribute(
                    "color",
                    random.choice(colors),
                )

        if blueprint.has_attribute("driver_id"):
            drivers = blueprint.get_attribute(
                "driver_id"
            ).recommended_values

            if drivers:
                blueprint.set_attribute(
                    "driver_id",
                    random.choice(drivers),
                )

        actor = world.try_spawn_actor(
            blueprint,
            spawn_point,
        )

        if actor is None:
            continue

        try:
            actor.set_autopilot(
                True,
                traffic_manager.get_port(),
            )

            try:
                traffic_manager.update_vehicle_lights(
                    actor,
                    True,
                )
            except Exception:
                pass

            traffic_actors.append(actor)

        except Exception:
            try:
                actor.destroy()
            except Exception:
                pass

    return traffic_actors


def destroy_traffic_vehicles(client, traffic_actors):
    if not traffic_actors:
        return

    commands = [
        carla.command.DestroyActor(actor.id)
        for actor in traffic_actors
        if actor is not None
    ]

    if commands:
        try:
            client.apply_batch(commands)
            return
        except Exception:
            pass

    for actor in traffic_actors:
        try:
            if actor is not None:
                actor.destroy()
        except Exception:
            pass


# ============================================================
# NIGHT LOW-VISIBILITY SCENARIO
# ============================================================


def set_hero_headlights(vehicle, enabled):
    """
    자차 헤드라이트를 켜거나 끈다.
    밤 시나리오용 기능이며 기존 차량 제어값에는 손대지 않는다.
    """
    if vehicle is None:
        return

    if enabled:
        lights = (
            carla.VehicleLightState.Position
            | carla.VehicleLightState.LowBeam
            | carla.VehicleLightState.HighBeam
        )
        vehicle.set_light_state(
            carla.VehicleLightState(lights)
        )
    else:
        vehicle.set_light_state(
            carla.VehicleLightState.NONE
        )


def set_night_environment(world):
    """
    현재 Weather의 다른 값은 그대로 두고 태양 고도만 낮춘다.
    """
    weather = world.get_weather()
    weather.sun_altitude_angle = NIGHT_SUN_ALTITUDE_ANGLE
    world.set_weather(weather)


def capture_and_disable_street_lights(world):
    """
    현재 가로등 상태를 저장한 뒤 자동 day/night cycle을 끄고
    Town05의 Street 그룹 가로등을 모두 OFF 한다.
    """
    try:
        light_manager = world.get_lightmanager()
        street_lights = list(
            light_manager.get_all_lights(
                carla.LightGroup.Street
            )
        )

        street_light_states = []

        if street_lights:
            street_light_states = list(
                light_manager.get_light_state(
                    street_lights
                )
            )

        # 밤으로 바꾸면 CARLA가 자동으로 가로등을 켤 수 있으므로
        # 자동 cycle을 끈 뒤 명시적으로 모두 OFF.
        light_manager.set_day_night_cycle(False)

        if street_lights:
            light_manager.turn_off(street_lights)

        print(
            f">>> STREET LIGHTS OFF: "
            f"{len(street_lights)} lights"
        )

        return (
            light_manager,
            street_lights,
            street_light_states,
        )

    except Exception as exc:
        print()
        print(">>> Street light control warning:", exc)
        return None, [], []


def restore_street_lights(
    light_manager,
    street_lights,
    street_light_states,
):
    """
    시나리오 시작 전에 저장해 둔 가로등 상태를 복원하고
    CARLA의 자동 day/night cycle도 다시 활성화한다.
    """
    if light_manager is None:
        return

    try:
        if (
            street_lights
            and street_light_states
            and len(street_lights)
            == len(street_light_states)
        ):
            light_manager.set_light_states(
                street_lights,
                street_light_states,
            )

        light_manager.set_day_night_cycle(True)

        print(
            f">>> STREET LIGHTS RESTORED: "
            f"{len(street_lights)} lights"
        )

    except Exception as exc:
        print()
        print(">>> Street light restore warning:", exc)


def create_virtual_obstacle_target(world, hero_vehicle):
    """
    O 버튼을 누른 순간 현재 자차 진행 차선의 100 m 전방 위치를
    '가상 장애물 목표 지점'으로 저장한다.

    이 단계에서는 실제 차량 Actor를 생성하지 않는다.
    """
    world_map = world.get_map()

    current_wp = world_map.get_waypoint(
        hero_vehicle.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )

    if current_wp is None:
        print()
        print(">>> NIGHT SCENARIO: driving waypoint not found")
        return None

    next_waypoints = current_wp.next(
        NIGHT_SCENARIO_DISTANCE_M
    )

    if not next_waypoints:
        print()
        print(">>> NIGHT SCENARIO: 50 m target waypoint not found")
        return None

    target_wp = next_waypoints[0]
    source = target_wp.transform

    # 원본 waypoint transform을 직접 수정하지 않고 복사본을 만든다.
    target_transform = carla.Transform(
        carla.Location(
            x=source.location.x,
            y=source.location.y,
            z=source.location.z + 0.25,
        ),
        carla.Rotation(
            pitch=source.rotation.pitch,
            yaw=source.rotation.yaw,
            roll=source.rotation.roll,
        ),
    )

    return target_transform


def spawn_night_obstacle(world, hero_vehicle, target_transform):
    """
    저장해 둔 가상 장애물 목표 지점에 실제 정차 차량을 Spawn한다.

    호출 시점은 가상 장애물 거리가 50 m 이하가 된 순간이다.
    """
    if target_transform is None:
        return None

    blueprint_library = world.get_blueprint_library()

    preferred_ids = [
        "vehicle.tesla.model3",
        "vehicle.audi.a2",
        "vehicle.audi.tt",
    ]

    blueprint = None

    for type_id in preferred_ids:
        found = blueprint_library.filter(type_id)
        if found:
            blueprint = found[0]
            break

    if blueprint is None:
        candidates = blueprint_library.filter("vehicle.*")

        if not candidates:
            print()
            print(">>> NIGHT SCENARIO: vehicle blueprint not found")
            return None

        blueprint = candidates[0]

    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute(
            "role_name",
            "night_obstacle",
        )

    if blueprint.has_attribute("color"):
        try:
            blueprint.set_attribute(
                "color",
                NIGHT_OBSTACLE_COLOR,
            )
        except Exception:
            pass

    obstacle = world.try_spawn_actor(
        blueprint,
        target_transform,
    )

    if obstacle is None:
        # 지면 높이 문제만 한 번 더 보정해서 재시도
        retry_transform = carla.Transform(
            carla.Location(
                x=target_transform.location.x,
                y=target_transform.location.y,
                z=target_transform.location.z + 0.20,
            ),
            carla.Rotation(
                pitch=target_transform.rotation.pitch,
                yaw=target_transform.rotation.yaw,
                roll=target_transform.rotation.roll,
            ),
        )

        obstacle = world.try_spawn_actor(
            blueprint,
            retry_transform,
        )

    if obstacle is None:
        print()
        print(
            ">>> NIGHT SCENARIO: actual obstacle spawn failed "
            "(target position may be occupied)"
        )
        return None

    obstacle.set_autopilot(False)

    obstacle.apply_control(
        carla.VehicleControl(
            throttle=0.0,
            brake=0.0,
            steer=0.0,
            hand_brake=True,
        )
    )

    # 상대 차량 자체의 라이트는 OFF.
    obstacle.set_light_state(
        carla.VehicleLightState.NONE
    )

    actual_distance = hero_vehicle.get_location().distance(
        obstacle.get_location()
    )

    print()
    print("==========================================================")
    print(" ACTUAL NIGHT OBSTACLE SPAWNED")
    print("==========================================================")
    print(
        f"Obstacle : {obstacle.type_id} / "
        f"{actual_distance:.1f} m from hero"
    )
    print("Obstacle lights : OFF")
    print("Hero headlights : AUTO ON")
    print("Radar/Collision : actual actor active")
    print("==========================================================")
    print()

    return obstacle


# ============================================================
# DYNAMIC JUNCTION SCENARIO HELPERS
# ============================================================


def junction_distance_2d(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    return math.sqrt(dx * dx + dy * dy)


def junction_angle_diff_deg(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def junction_heading_score(current_wp, next_wp):
    return junction_angle_diff_deg(
        current_wp.transform.rotation.yaw,
        next_wp.transform.rotation.yaw,
    )


def junction_choose_straightest_waypoint(
    current_wp,
    candidates,
):
    if not candidates:
        return None

    return min(
        candidates,
        key=lambda wp: junction_heading_score(
            current_wp,
            wp,
        ),
    )


def find_next_unprocessed_junction(
    carla_map,
    ego,
    processed_junctions,
):
    """
    EGO가 현재 진행하는 도로를 기준으로 앞쪽을 따라가며
    아직 사용하지 않은 다음 교차로를 찾는다.
    갈림길에서는 현재 진행방향과 가장 직진에 가까운 waypoint를 사용한다.
    """
    current_wp = carla_map.get_waypoint(
        ego.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )

    if current_wp is None:
        return None, None, None

    travelled = 0.0
    wp = current_wp

    while travelled < JUNCTION_SEARCH_DISTANCE:
        next_candidates = wp.next(1.0)

        if not next_candidates:
            return None, None, None

        next_wp = junction_choose_straightest_waypoint(
            wp,
            next_candidates,
        )

        if next_wp is None:
            return None, None, None

        wp = next_wp
        travelled += 1.0

        if wp.is_junction:
            junction = wp.get_junction()

            if junction.id in processed_junctions:
                # 같은 교차로 영역을 빠져나갈 때까지 계속 전진
                while (
                    wp is not None
                    and wp.is_junction
                    and travelled < JUNCTION_SEARCH_DISTANCE
                ):
                    next_candidates = wp.next(1.0)

                    if not next_candidates:
                        return None, None, None

                    wp = junction_choose_straightest_waypoint(
                        wp,
                        next_candidates,
                    )

                    if wp is None:
                        return None, None, None

                    travelled += 1.0

                continue

            return junction, wp, travelled

    return None, None, None


def find_right_side_junction_entry(
    junction,
    ego,
    ego_junction_wp,
):
    """
    교차로 연결 도로 중 EGO 기준 오른쪽에 있고,
    EGO 진행방향과 대략 수직인 진입도로를 선택한다.
    """
    pairs = junction.get_waypoints(
        carla.LaneType.Driving
    )

    ego_transform = ego.get_transform()
    ego_location = ego_transform.location
    ego_yaw = ego_transform.rotation.yaw
    right_vector = ego_transform.get_right_vector()

    candidates = []

    for entry_wp, exit_wp in pairs:
        # EGO가 진행 중인 같은 road는 제외
        if entry_wp.road_id == ego_junction_wp.road_id:
            continue

        entry_location = entry_wp.transform.location

        rel_x = entry_location.x - ego_location.x
        rel_y = entry_location.y - ego_location.y

        right_dot = (
            rel_x * right_vector.x
            + rel_y * right_vector.y
        )

        if right_dot <= 0.0:
            continue

        heading_difference = junction_angle_diff_deg(
            entry_wp.transform.rotation.yaw,
            ego_yaw,
        )

        # EGO와 대략 수직인 도로
        if not (45.0 <= heading_difference <= 135.0):
            continue

        angle_error = abs(
            90.0 - heading_difference
        )

        candidates.append(
            (
                angle_error,
                entry_wp,
                exit_wp,
            )
        )

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0])

    _, entry_wp, exit_wp = candidates[0]

    return entry_wp, exit_wp


def get_junction_npc_spawn_transforms(entry_wp):
    """
    오른쪽 교차로 진입부 뒤쪽의 여러 spawn 후보를 만든다.

    40대 AI 교통 차량이 있을 때 한 위치만 시도하면
    이미 다른 차량이 차지하고 있어 try_spawn_actor()가 계속 실패할 수 있다.
    """
    transforms = []
    seen = set()

    for back_distance in JUNCTION_SPAWN_BACK_CANDIDATES_M:
        previous_wps = entry_wp.previous(
            back_distance
        )

        if previous_wps:
            spawn_wp = min(
                previous_wps,
                key=lambda wp: junction_angle_diff_deg(
                    wp.transform.rotation.yaw,
                    entry_wp.transform.rotation.yaw,
                ),
            )
        else:
            continue

        key = (
            round(spawn_wp.transform.location.x, 1),
            round(spawn_wp.transform.location.y, 1),
            round(spawn_wp.transform.rotation.yaw, 1),
        )

        if key in seen:
            continue

        seen.add(key)

        transforms.append(
            carla.Transform(
                carla.Location(
                    x=spawn_wp.transform.location.x,
                    y=spawn_wp.transform.location.y,
                    z=spawn_wp.transform.location.z + 0.35,
                ),
                carla.Rotation(
                    pitch=spawn_wp.transform.rotation.pitch,
                    yaw=spawn_wp.transform.rotation.yaw,
                    roll=spawn_wp.transform.rotation.roll,
                ),
            )
        )

    # previous() 후보가 하나도 없을 경우 entry 자체도 최후 후보로 사용
    if not transforms:
        source = entry_wp.transform

        transforms.append(
            carla.Transform(
                carla.Location(
                    x=source.location.x,
                    y=source.location.y,
                    z=source.location.z + 0.35,
                ),
                carla.Rotation(
                    pitch=source.rotation.pitch,
                    yaw=source.rotation.yaw,
                    roll=source.rotation.roll,
                ),
            )
        )

    return transforms


def get_junction_npc_blueprint(world):
    library = world.get_blueprint_library()

    preferred = [
        "vehicle.tesla.model3",
        "vehicle.audi.a2",
        "vehicle.nissan.micra",
        "vehicle.mercedes.coupe",
    ]

    available = []

    for name in preferred:
        result = library.filter(name)

        if result:
            available.append(result[0])

    if available:
        bp = random.choice(available)
    else:
        vehicles = library.filter("vehicle.*")

        if not vehicles:
            return None

        bp = vehicles[0]

    if bp.has_attribute("role_name"):
        bp.set_attribute(
            "role_name",
            "scenario_npc",
        )

    return bp


def spawn_junction_npc(
    world,
    spawn_transforms,
):
    """
    여러 spawn 후보를 순서대로 시도한다.
    Traffic Manager 차량 때문에 앞 후보가 막혀도 다음 후보로 넘어간다.
    """
    if not spawn_transforms:
        return None

    bp = get_junction_npc_blueprint(world)

    if bp is None:
        return None

    for spawn_transform in spawn_transforms:
        npc = world.try_spawn_actor(
            bp,
            spawn_transform,
        )

        if npc is None:
            continue

        npc.set_autopilot(False)

        # Trigger 전에는 확실히 정지시킨다.
        npc.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=1.0,
                hand_brake=True,
            )
        )

        return npc

    return None


def launch_junction_npc(npc):
    if npc is None or not npc.is_alive:
        return

    transform = npc.get_transform()
    forward = transform.get_forward_vector()

    # 바로 교차로에 진입하도록 초기 속도를 부여
    npc.set_target_velocity(
        carla.Vector3D(
            x=forward.x * JUNCTION_NPC_INITIAL_SPEED,
            y=forward.y * JUNCTION_NPC_INITIAL_SPEED,
            z=0.0,
        )
    )

    npc.apply_control(
        carla.VehicleControl(
            throttle=JUNCTION_NPC_THROTTLE,
            steer=0.0,
            brake=0.0,
            hand_brake=False,
        )
    )


def destroy_junction_npc(npc):
    if npc is None:
        return

    try:
        if npc.is_alive:
            npc.apply_control(
                carla.VehicleControl(
                    throttle=0.0,
                    steer=0.0,
                    brake=1.0,
                )
            )
            npc.destroy()
    except RuntimeError:
        pass


# ============================================================
# MAIN
# ============================================================



# Independent, send-only lane feedback channel. No CARLA/LFA computation here.
RPI_FFB_IP = "192.168.50.2"
RPI_FFB_PORT = 5007
LANE_FFB_SEND_INTERVAL = 1.0 / 30.0


class LaneFFBSender:
    def __init__(self):
        self.socket = None
        self.sequence = 0
        self.last_send = None
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setblocking(False)
        except OSError:
            self.close()

    def send(self, now, enabled, state, correction):
        if self.socket is None:
            return
        if self.last_send is not None and now - self.last_send < LANE_FFB_SEND_INTERVAL:
            return
        payload = {
            "type": "lane_ffb",
            "sequence": self.sequence,
            "lane_assist_enabled": bool(enabled),
            "lane_assist_active": bool(
                enabled and state == "ACTIVE" and abs(correction) > 0.003
            ),
            "lane_assist_state": state,
            "lane_assist_correction": correction,
        }
        try:
            self.socket.sendto(
                json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                (RPI_FFB_IP, RPI_FFB_PORT),
            )
        except OSError:
            # A failed feedback update must never interrupt vehicle control.
            pass
        self.sequence += 1
        self.last_send = now

    def close(self):
        if self.socket is not None:
            try:
                self.socket.close()
            except OSError:
                pass
            self.socket = None


class SafetyControlReceiver:
    """Apply Safety commands in D/R and maintain fail-safe in every gear."""
    def __init__(self, host="0.0.0.0", port=5008, timeout=0.5):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((host, port))
            self.socket.setblocking(False)
        except OSError:
            self.socket.close()
            raise
        self.timeout = timeout
        self.last_rx = None
        self.last_log = None
        self.mode = 0
        self.limit_percent = 100
        self.risk_level = 0
        self.failsafe = False

        # Safety UDP packet validation
        self.allowed_sender_ip = "192.168.0.37"
        self.last_sequence = None
        self.session_id = None
        self.retired_sessions = set()

    def receive(self, now):
        # Bound work per frame even if a sender floods this port.
        for _ in range(256):
            try:
                data, sender_addr = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as error:
                print(f"[CARLA SAFETY] RECEIVE ERROR: {error}")
                break

            # 1. Sender IP validation
            if sender_addr[0] != self.allowed_sender_ip:
                continue

            try:
                packet = json.loads(data)

                if (
                    not isinstance(packet, dict)
                    or packet.get("type") != "safety_control"
                ):
                    continue

                mode = packet["mode"]
                limit = packet["limit_percent"]
                risk = packet["risk_level"]
                sequence = packet["sequence"]
                failsafe = packet["failsafe"]
                session_id = packet["session_id"]

                # 2. Validate packet fields
                if (
                    type(mode) is not int
                    or mode not in (0, 1, 2)
                    or type(limit) not in (int, float)
                    or not 0 <= limit <= 100
                    or type(risk) is not int
                    or risk not in (0, 1, 2, 3)
                    or type(sequence) is not int
                    or sequence < 0
                    or type(failsafe) is not bool
                    or not isinstance(session_id, str)
                    or len(session_id) != 32
                    or any(c not in "0123456789abcdef" for c in session_id)
                ):
                    continue

                # 3. Reject previously retired sessions
                if session_id in self.retired_sessions:
                    continue

                # 4. Handle sender process restart
                if self.session_id != session_id:
                    if self.session_id is not None:
                        # Do not switch while old session is fresh
                        old_fresh = (
                            self.last_rx is not None
                            and 0 <= now - self.last_rx < self.timeout
                        )
                        if old_fresh:
                            continue

                        # New session must begin in a safe state
                        if not (mode == 2 or failsafe):
                            continue

                        self.retired_sessions.add(self.session_id)

                    self.session_id = session_id
                    self.last_sequence = None

                # 5. Reject duplicate or out-of-order sequences
                if (
                    self.last_sequence is not None
                    and sequence <= self.last_sequence
                ):
                    continue

                # 6. Only accepted packets update control state
                self.last_sequence = sequence
                self.mode = mode
                self.limit_percent = limit
                self.risk_level = risk
                self.failsafe = failsafe
                self.last_rx = now

            except (ValueError, KeyError, TypeError, UnicodeError):
                continue

    def apply(self, control, now, gear="D", speed_kmh=0.0,
              creep_throttle=0.0, driver_throttle=None):
        self.receive(now)
        raw_throttle = control.throttle
        raw_brake = control.brake
        fresh = self.last_rx is not None and 0 <= now - self.last_rx < self.timeout

        # Priority: stale/failsafe > STOP > PARK > LIMIT > NORMAL.
        if not fresh or self.failsafe:
            control.throttle = 0.0
            control.brake = 1.0
            applied_mode = "FAILSAFE_STOP"
        elif gear == "P":
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = True
            applied_mode = "PARK"
        elif self.mode == 2:
            control.throttle = 0.0
            control.brake = 1.0
            applied_mode = "STOP"
        elif self.mode == 1:
            control.throttle = min(
                raw_throttle,
                self.limit_percent / 100.0,
            )
            applied_mode = "LIMIT"
        else:
            applied_mode = "NORMAL"

        if self.last_log is None or now - self.last_log >= 0.5:
            self.last_log = now
            print(
                f"[CARLA SAFETY] GEAR={gear} SPEED_KMH={speed_kmh:.1f} "
                f"CREEP={int(creep_throttle > 0.0)} "
                f"CREEP_THROTTLE={creep_throttle:.3f} "
                f"SAFETY_ACTIVE_FOR_REVERSE={int(gear == 'R')} "
                f"RISK=L{self.risk_level} MODE={applied_mode} "
                f"LIMIT={self.limit_percent:g} FAILSAFE={int(self.failsafe)} "
                f"FRESH={int(fresh)} "
                f"RAW_THROTTLE={raw_throttle if driver_throttle is None else driver_throttle:.2f} "
                f"RAW_BRAKE={raw_brake:.2f} APPLIED_THROTTLE={control.throttle:.3f} "
                f"APPLIED_BRAKE={control.brake:.2f}"
            )
            if not fresh:
                if self.last_rx is None:
                    print("[CARLA SAFETY] WAITING FOR FIRST safety_control PACKET -> STOP")
                print("[CARLA SAFETY] STALE -> STOP")

    def close(self):
        self.socket.close()


def main():
    global control_mode

    safety_control_receiver = None

    pygame.init()
    pygame.font.init()

    # 현재 모니터의 실제 데스크톱 해상도를 사용한다.
    display_info = pygame.display.Info()
    DISPLAY_WIDTH = display_info.current_w
    DISPLAY_HEIGHT = display_info.current_h

    print(
        f"Display fullscreen: "
        f"{DISPLAY_WIDTH}x{DISPLAY_HEIGHT}"
    )
    print(
        f"RGB camera sensor: "
        f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}"
    )

    display = pygame.display.set_mode(
        (DISPLAY_WIDTH, DISPLAY_HEIGHT),
        pygame.FULLSCREEN
        | pygame.HWSURFACE
        | pygame.DOUBLEBUF,
    )

    pygame.mouse.set_visible(False)
    pygame.display.set_caption("PACE - CARLA Mustang FULLSCREEN")

    clock = pygame.time.Clock()

    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(30.0)

    world = None
    vehicle = None
    camera = None
    front_distance_sensor = None
    sock = None
    telemetry_sock = None
    ui_sock = None
    lane_ffb_sender = None
    traffic_manager = None
    traffic_vehicles = []

    # Night scenario state
    night_obstacle = None
    night_scenario_started = False
    hero_headlights_on = False
    original_weather = None

    # Night scenario street-light state
    street_light_manager = None
    street_lights = []
    street_light_original_states = []

    # O 버튼으로 시작되는 가상 장애물 시나리오
    virtual_obstacle_active = False
    virtual_obstacle_distance = None
    virtual_obstacle_target_transform = None
    virtual_obstacle_last_location = None
    scenario_headlights_triggered = False

    # Dynamic junction scenario state
    junction_scenario_armed = False
    junction_processed_ids = set()

    junction_current = None
    junction_current_wp = None
    junction_spawn_transforms = []

    junction_npc = None
    junction_npc_spawned = False
    junction_npc_triggered = False
    junction_npc_launch_time = 0.0

    # 현재 CARLA에 실제 적용된 기어
    applied_gear = "D"

    # 적용 요청 기어
    requested_gear = "D"

    # Raspberry Pi에서 받은 실제 시프터 기어
    hardware_gear = "P"

    gear_change_pending = False

    keyboard = KeyboardVehicleControl()

    accel_percent = 0
    brake_percent = 0
    steer_input = 0.0

    smooth_throttle = 0.0
    smooth_brake = 0.0
    smooth_steer = 0.0

    # Realistic LFA state
    lane_assist_enabled = LANE_ASSIST_ENABLED_DEFAULT
    lane_assist_smoothed = 0.0
    lane_assist_correction = 0.0
    lane_assist_lateral_error = 0.0
    lane_assist_heading_error = 0.0
    lane_assist_state = "OFF"
    driver_steer_for_override = 0.0

    lfa_engage_level = 0.0
    lfa_override_until = 0.0
    lfa_reacquire_after = 0.0
    lfa_was_in_junction = False

    # High-speed lateral-motion damping state
    lfa_prev_lateral_error = 0.0
    lfa_lateral_rate_filtered = 0.0
    lfa_lateral_error_valid = False

    last_packet_time = time.monotonic()
    last_status_time = 0.0
    last_telemetry_send_time = 0.0
    last_ui_send_time = 0.0
    last_radar_debug_time = 0.0
    telemetry_sequence = 0

    try:
        print()
        print("==========================================================")
        print("      PACE - CARLA MANUAL_CONTROL OPTIMIZED")
        print("==========================================================")
        print()

        # ====================================================
        # CONNECT / MAP
        # ====================================================

        world = client.get_world()

        print("Client:", client.get_client_version())
        print("Server:", client.get_server_version())
        print("Current map:", world.get_map().name)

        if "Town05" not in world.get_map().name:
            print("Loading Town05...")
            client.set_timeout(60.0)
            world = client.load_world(MAP_NAME, reset_settings=True)
            client.set_timeout(30.0)
            print("Map loaded:", world.get_map().name)

        # ====================================================
        # ASYNCHRONOUS MODE
        #
        # manual_control.py 기본 방식.
        # Python이 world.tick()으로 서버를 끌고 가지 않는다.
        # ====================================================

        settings = world.get_settings()

        if settings.synchronous_mode or settings.fixed_delta_seconds is not None:
            print("Switching CARLA to asynchronous / variable timestep...")
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        # 서버가 한 프레임 진행하도록 잠깐 기다림
        try:
            world.wait_for_tick(2.0)
        except RuntimeError:
            pass

        # ====================================================
        # TRAFFIC MANAGER
        #
        # 메인 월드는 manual_control.py처럼 ASYNC 유지.
        # 주변 AI 차량만 Traffic Manager가 주행한다.
        # ====================================================

        traffic_manager = client.get_trafficmanager(
            TRAFFIC_MANAGER_PORT
        )

        traffic_manager.set_synchronous_mode(False)

        traffic_manager.set_global_distance_to_leading_vehicle(
            TRAFFIC_GLOBAL_DISTANCE
        )

        if TRAFFIC_HYBRID_PHYSICS:
            try:
                traffic_manager.set_hybrid_physics_mode(True)
                traffic_manager.set_hybrid_physics_radius(
                    TRAFFIC_HYBRID_RADIUS
                )
            except Exception as exc:
                print(
                    "Traffic hybrid physics warning:",
                    exc,
                )

        # ====================================================
        # VEHICLE
        # ====================================================

        vehicle = spawn_mustang(world)

        print("Vehicle:", vehicle.type_id)
        print(
            "Hero spawn:",
            f"X={HERO_SPAWN_X:.2f},",
            f"Y={HERO_SPAWN_Y:.2f},",
            f"Z={HERO_SPAWN_Z:.2f},",
            f"YAW={HERO_SPAWN_YAW:.1f}",
        )

        # ====================================================
        # ROAD TRAFFIC
        # ====================================================

        traffic_vehicles = spawn_traffic_vehicles(
            world,
            traffic_manager,
            vehicle,
            TRAFFIC_VEHICLE_COUNT,
        )

        print(
            f"Traffic vehicles: "
            f"{len(traffic_vehicles)}/{TRAFFIC_VEHICLE_COUNT}"
        )

        # ====================================================
        # CAMERA
        # ====================================================

        camera = CameraManager(
            vehicle,
            CAMERA_WIDTH,
            CAMERA_HEIGHT,
            CAMERA_GAMMA,
        )

        # manual_control.py 기본 transform_index=1에 해당하는 시점
        camera.start("DRIVER")

        # ====================================================
        # FRONT DISTANCE SENSOR
        # ====================================================

        front_distance_sensor = FrontDistanceSensor(
            vehicle
        )

        print(
            f"Front distance sensor: 0 ~ "
            f"{FRONT_DISTANCE_RANGE:.1f} m"
        )

        # ====================================================
        # UDP
        # ====================================================

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        sock.setblocking(False)


        lane_ffb_sender = LaneFFBSender()
        safety_control_receiver = SafetyControlReceiver()

        # PC -> Raspberry Pi HMI UI socket (non-blocking, send only).
        ui_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ui_sock.setblocking(False)

        # PC -> Raspberry Pi telemetry socket
        telemetry_sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        print(
            f"Telemetry target: "
            f"{RPI_IP}:{RPI_TELEMETRY_PORT}"
        )
        print(
            f"Telemetry send rate: "
            f"{TELEMETRY_SEND_HZ:.0f} Hz"
        )

        # ====================================================
        # READY
        # ====================================================

        print()
        print("==========================================================")
        print("                         READY")
        print("==========================================================")
        print()
        print("W / UP       : Accelerator")
        print("S / DOWN     : Brake")
        print("A,D / LEFT,RIGHT : Steering")
        print("SPACE        : Full Brake")
        print("R            : D / R")
        print("K            : KEYBOARD / HARDWARE")
        print("J            : Lane Assist ON / OFF")
        print("I            : Arm/cancel NEXT JUNCTION sudden vehicle")
        print("C            : DRIVER / CHASE Camera")
        print("N            : Toggle DAY / DARK NIGHT")
        print("O            : Start/reset 100m VIRTUAL obstacle scenario")
        print("L            : Headlights ON / OFF")
        print("F11          : Fullscreen / Windowed")
        print("ESC          : Exit")
        print()
        print("START MODE   : KEYBOARD")
        print("CARLA MODE   : ASYNC / VARIABLE TIMESTEP")
        print("CLIENT VIEW  : 60 FPS")
        print(
            f"HERO SPAWN   : "
            f"({HERO_SPAWN_X:.2f}, {HERO_SPAWN_Y:.2f}) "
            f"YAW {HERO_SPAWN_YAW:.1f}"
        )
        print(
            f"ROAD TRAFFIC : "
            f"{len(traffic_vehicles)} AI vehicles"
        )
        print(
            f"FRONT RANGE  : 0 ~ "
            f"{FRONT_DISTANCE_RANGE:.1f} m"
        )
        print(
            f"DIST FILTER  : median {DISTANCE_HISTORY_SIZE} frames, "
            f"{DISTANCE_DISPLAY_STEP:.2f} m step"
        )
        print(
            f"PC -> RPi    : "
            f"{RPI_IP}:{RPI_TELEMETRY_PORT}"
        )
        print(
            "UDP FORMAT   : speed_kmh,distance_m"
        )
        print(
            "NO SIGN      : distance = -1.00"
        )
        print(
            "RPi -> PC    : accel,brake,steer,gear"
        )
        print(f"VEHICLE PHYS : CARLA DEFAULT + TORQUE x{ENGINE_TORQUE_MULTIPLIER:.2f}")
        print(f"THROTTLE FILTER : {THROTTLE_ALPHA:.2f}")
        print(f"BRAKE FILTER    : {BRAKE_ALPHA:.2f}")
        print(f"BRAKE GAIN      : x{BRAKE_INPUT_GAIN:.2f}")
        print(
            "STEERING GAIN   : "
            f"{STEERING_GAIN_LOW_SPEED:.2f} low -> "
            f"{STEERING_GAIN_MID_SPEED:.2f} mid -> "
            f"{STEERING_GAIN_HIGH_SPEED:.2f} high"
        )
        print(
            "LANE ASSIST     : "
            "Realistic LFA / J toggle / "
            f"{LFA_MIN_SPEED_KMH:.0f}~{LFA_MAX_SPEED_KMH:.0f} km/h"
        )
        print()

        running = True

        while running:
            # manual_control.py처럼 클라이언트 렌더만 60 FPS 제한
            milliseconds = clock.tick_busy_loop(CLIENT_FPS)

            # =================================================
            # EVENTS
            # =================================================

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_ESCAPE:
                        running = False

                    elif event.key == pygame.K_k:
                        control_mode = (
                            "HARDWARE"
                            if control_mode == "KEYBOARD"
                            else "KEYBOARD"
                        )

                        keyboard.reset()
                        smooth_throttle = 0.0
                        smooth_brake = 0.0
                        smooth_steer = 0.0

                        print()
                        print(">>> CONTROL MODE:", control_mode)

                    elif event.key == pygame.K_j:
                        lane_assist_enabled = (
                            not lane_assist_enabled
                        )

                        lane_assist_smoothed = 0.0
                        lane_assist_correction = 0.0
                        lfa_engage_level = 0.0
                        lfa_override_until = 0.0
                        lfa_reacquire_after = 0.0
                        lfa_was_in_junction = False
                        lfa_prev_lateral_error = 0.0
                        lfa_lateral_rate_filtered = 0.0
                        lfa_lateral_error_valid = False

                        print()
                        print(
                            ">>> LANE ASSIST:",
                            "ON"
                            if lane_assist_enabled
                            else "OFF",
                        )

                    elif event.key == pygame.K_i:
                        # -----------------------------------------
                        # NEXT JUNCTION SCENARIO
                        #
                        # I를 한 번 누르면 다음 사용 가능한 교차로를
                        # 찾아 오른쪽 돌발 차량 시나리오를 1회 실행.
                        #
                        # 실행 중 다시 I를 누르면 취소.
                        # -----------------------------------------

                        if junction_scenario_armed:
                            junction_scenario_armed = False

                            destroy_junction_npc(
                                junction_npc
                            )

                            junction_npc = None
                            junction_current = None
                            junction_current_wp = None
                            junction_spawn_transforms = []
                            junction_npc_spawned = False
                            junction_npc_triggered = False
                            junction_npc_launch_time = 0.0

                        else:
                            junction_scenario_armed = True

                            destroy_junction_npc(
                                junction_npc
                            )

                            junction_npc = None
                            junction_current = None
                            junction_current_wp = None
                            junction_spawn_transforms = []
                            junction_npc_spawned = False
                            junction_npc_triggered = False
                            junction_npc_launch_time = 0.0

                    elif event.key == pygame.K_r:
                        # 키보드 모드에서는 R키로 D <-> R 토글
                        if requested_gear == "R":
                            requested_gear = "D"
                        else:
                            requested_gear = "R"

                        gear_change_pending = (
                            requested_gear != applied_gear
                        )

                        print()
                        print(
                            f">>> GEAR requested: {requested_gear}"
                        )

                    elif event.key == pygame.K_c:
                        new_mode = camera.toggle()
                        print()
                        print(">>> CAMERA:", new_mode)

                    elif event.key == pygame.K_n:
                        # -------------------------------------------------
                        # DAY / NIGHT TOGGLE
                        #
                        # N을 누를 때마다 밤 <-> 원래 낮 환경으로 전환.
                        # 차량 Spawn 기능(O)과는 독립적으로 동작한다.
                        # -------------------------------------------------

                        if original_weather is None:
                            original_weather = world.get_weather()

                        if not night_scenario_started:
                            (
                                street_light_manager,
                                street_lights,
                                street_light_original_states,
                            ) = capture_and_disable_street_lights(
                                world
                            )

                            set_night_environment(world)
                            night_scenario_started = True

                            print()
                            print("==========================================================")
                            print(" DARK NIGHT ENVIRONMENT APPLIED / STREET LIGHTS OFF")
                            print("==========================================================")
                            print(
                                f"Sun altitude : "
                                f"{NIGHT_SUN_ALTITUDE_ANGLE:.1f} deg"
                            )
                            print("Obstacle spawn : press O")
                            print("Headlights     : press L")
                            print("Press N again  : return to DAY")
                            print("==========================================================")
                            print()

                        else:
                            if original_weather is not None:
                                world.set_weather(original_weather)

                            restore_street_lights(
                                street_light_manager,
                                street_lights,
                                street_light_original_states,
                            )

                            street_light_manager = None
                            street_lights = []
                            street_light_original_states = []

                            night_scenario_started = False

                            print()
                            print("==========================================================")
                            print(" DAY ENVIRONMENT / STREET LIGHTS RESTORED")
                            print("==========================================================")
                            print("Press N again  : DARK NIGHT")
                            print("==========================================================")
                            print()

                    elif event.key == pygame.K_o:
                        # -------------------------------------------------
                        # VIRTUAL OBSTACLE SCENARIO
                        #
                        # O를 누르면:
                        #   1) 실제 차량은 아직 Spawn하지 않음
                        #   2) 100 m 가상 장애물 거리 생성
                        #   3) Raspberry Pi에는 이 거리를 전송
                        #   4) 거리가 50 m 이하가 되면 실제 차량 Spawn
                        #   5) 거리가 20 m 이하가 되면 자차 헤드라이트 자동 ON
                        # -------------------------------------------------

                        if night_obstacle is not None:
                            try:
                                night_obstacle.destroy()
                            except Exception:
                                pass
                            night_obstacle = None

                        virtual_obstacle_target_transform = (
                            create_virtual_obstacle_target(
                                world,
                                vehicle,
                            )
                        )

                        if virtual_obstacle_target_transform is not None:
                            virtual_obstacle_active = True
                            virtual_obstacle_distance = (
                                NIGHT_SCENARIO_DISTANCE_M
                            )
                            current_loc = vehicle.get_location()
                            virtual_obstacle_last_location = (
                                carla.Location(
                                    x=current_loc.x,
                                    y=current_loc.y,
                                    z=current_loc.z,
                                )
                            )

                            # 시나리오 시작 시에는 라이트 OFF.
                            # 차량은 50 m에서 Spawn하고,
                            # 자차 헤드라이트는 20 m에서 자동 ON.
                            scenario_headlights_triggered = False
                            hero_headlights_on = False
                            set_hero_headlights(
                                vehicle,
                                False,
                            )

                            print()
                            print("==========================================================")
                            print(" VIRTUAL OBSTACLE SCENARIO STARTED")
                            print("==========================================================")
                            print(
                                f"Virtual distance : "
                                f"{virtual_obstacle_distance:.1f} m"
                            )
                            print(
                                f"Actual vehicle spawn : "
                                f"{NIGHT_REAL_SPAWN_DISTANCE_M:.1f} m"
                            )
                            print(
                                f"Headlights auto ON    : "
                                f"{NIGHT_HEADLIGHT_ON_DISTANCE_M:.1f} m"
                            )
                            print("Hero headlights : OFF")
                            print(
                                "RPi telemetry uses VIRTUAL scenario "
                                "distance for the whole scenario"
                            )
                            print("==========================================================")
                            print()

                        else:
                            virtual_obstacle_active = False
                            virtual_obstacle_distance = None
                            virtual_obstacle_last_location = None
                            scenario_headlights_triggered = False

                    elif event.key == pygame.K_l:
                        # Night scenario에서 운전자가 헤드라이트를 켜
                        # 어두운 앞차를 뒤늦게 발견하는 연출.
                        hero_headlights_on = (
                            not hero_headlights_on
                        )

                        set_hero_headlights(
                            vehicle,
                            hero_headlights_on,
                        )

                        print()
                        print(
                            ">>> HEADLIGHTS:",
                            "ON"
                            if hero_headlights_on
                            else "OFF",
                        )

                    elif event.key == pygame.K_F11:
                        # 필요할 때만 F11로 전체화면 <-> 창모드 전환
                        if display.get_flags() & pygame.FULLSCREEN:
                            display = pygame.display.set_mode(
                                (1280, 720),
                                pygame.HWSURFACE | pygame.DOUBLEBUF,
                            )
                            pygame.mouse.set_visible(True)
                            print()
                            print(">>> DISPLAY: WINDOWED 1280x720")
                        else:
                            display = pygame.display.set_mode(
                                (DISPLAY_WIDTH, DISPLAY_HEIGHT),
                                pygame.FULLSCREEN
                                | pygame.HWSURFACE
                                | pygame.DOUBLEBUF,
                            )
                            pygame.mouse.set_visible(False)
                            print()
                            print(
                                f">>> DISPLAY: FULLSCREEN "
                                f"{DISPLAY_WIDTH}x{DISPLAY_HEIGHT}"
                            )

            if not running:
                break

            # =================================================
            # VIRTUAL NIGHT OBSTACLE UPDATE
            #
            # O 시작 시 100 m.
            #
            # 이전 버전의 "속도 x 시간" 적분 방식은
            # FPS 저하/프레임 지연 때 가상 거리가 실제 차량보다
            # 늦게 줄어들 수 있었다.
            #
            # v6에서는 CARLA 차량의 실제 위치 변화량으로
            # 가상 거리를 감소시킨다.
            #
            # 50 m 이하에서 실제 차량 Spawn, 20 m 이하에서 헤드라이트 ON.
            # =================================================

            if (
                virtual_obstacle_active
                and virtual_obstacle_distance is not None
            ):
                current_loc = vehicle.get_location()

                if virtual_obstacle_last_location is None:
                    virtual_obstacle_last_location = (
                        carla.Location(
                            x=current_loc.x,
                            y=current_loc.y,
                            z=current_loc.z,
                        )
                    )

                dx = (
                    current_loc.x
                    - virtual_obstacle_last_location.x
                )
                dy = (
                    current_loc.y
                    - virtual_obstacle_last_location.y
                )
                dz = (
                    current_loc.z
                    - virtual_obstacle_last_location.z
                )

                moved_distance = math.sqrt(
                    dx * dx
                    + dy * dy
                    + dz * dz
                )

                # 후진 시에는 가상 장애물 거리를 줄이지 않는다.
                forward = (
                    vehicle
                    .get_transform()
                    .get_forward_vector()
                )

                forward_motion = (
                    dx * forward.x
                    + dy * forward.y
                    + dz * forward.z
                )

                if forward_motion > 0.0:
                    virtual_obstacle_distance = max(
                        0.0,
                        virtual_obstacle_distance
                        - moved_distance,
                    )

                virtual_obstacle_last_location = (
                    carla.Location(
                        x=current_loc.x,
                        y=current_loc.y,
                        z=current_loc.z,
                    )
                )

                # -------------------------------------------------
                # 50 m: 실제 차량 Spawn
                # -------------------------------------------------
                if (
                    night_obstacle is None
                    and virtual_obstacle_distance
                    <= NIGHT_REAL_SPAWN_DISTANCE_M
                ):
                    night_obstacle = spawn_night_obstacle(
                        world,
                        vehicle,
                        virtual_obstacle_target_transform,
                    )

                    if night_obstacle is not None:
                        print()
                        print(
                            ">>> 50 m reached: "
                            "ACTUAL VEHICLE SPAWNED / HEADLIGHTS STILL OFF"
                        )

                # -------------------------------------------------
                # 20 m: 자차 헤드라이트 자동 ON
                # 실제 차량 Spawn 성공 여부와 관계없이 한 번만 실행한다.
                # -------------------------------------------------
                if (
                    not scenario_headlights_triggered
                    and virtual_obstacle_distance
                    <= NIGHT_HEADLIGHT_ON_DISTANCE_M
                ):
                    scenario_headlights_triggered = True
                    hero_headlights_on = True

                    set_hero_headlights(
                        vehicle,
                        True,
                    )

                    print()
                    print(
                        ">>> 20 m reached: "
                        "HERO HEADLIGHTS ON"
                    )

            # =================================================
            # DYNAMIC JUNCTION SCENARIO UPDATE
            #
            # I를 눌러 armed 된 경우에만 동작한다.
            #
            # 1) 다음 사용 가능한 교차로 탐색
            # 2) 60 m 이내 -> 오른쪽 도로 여러 후보에서 NPC 미리 생성
            # 3) 18 m 이내 -> NPC가 직진 돌발 진입
            # 4) 출발 3초 후 NPC 삭제, 시나리오 1회 종료
            # =================================================

            if junction_scenario_armed:
                carla_map = world.get_map()
                ego_location = vehicle.get_location()

                # ---------------------------------------------
                # 아직 목표 교차로가 없으면 다음 교차로 탐색
                # ---------------------------------------------
                if junction_current is None:
                    (
                        found_junction,
                        found_junction_wp,
                        _road_distance,
                    ) = find_next_unprocessed_junction(
                        carla_map,
                        vehicle,
                        junction_processed_ids,
                    )

                    if found_junction is not None:
                        side_pair = find_right_side_junction_entry(
                            found_junction,
                            vehicle,
                            found_junction_wp,
                        )

                        if side_pair is None:
                            # 오른쪽에서 차량이 들어올 수 없는 교차로는
                            # 이번 시나리오 후보에서 제외하고 다음 것을 탐색.
                            junction_processed_ids.add(
                                found_junction.id
                            )

                        else:
                            side_entry_wp, _side_exit_wp = (
                                side_pair
                            )

                            junction_current = found_junction
                            junction_current_wp = (
                                found_junction_wp
                            )

                            junction_spawn_transforms = (
                                get_junction_npc_spawn_transforms(
                                    side_entry_wp
                                )
                            )

                            junction_npc_spawned = False
                            junction_npc_triggered = False
                            junction_npc_launch_time = 0.0

                # ---------------------------------------------
                # 목표 교차로가 있으면 거리 기반 상태 진행
                # ---------------------------------------------
                if (
                    junction_current is not None
                    and junction_current_wp is not None
                ):
                    junction_target_location = (
                        junction_current_wp
                        .transform
                        .location
                    )

                    junction_distance = (
                        junction_distance_2d(
                            ego_location,
                            junction_target_location,
                        )
                    )

                    # 60 m 이내: 오른쪽 차량 여러 후보에서 미리 생성/정지 대기
                    if (
                        not junction_npc_spawned
                        and junction_distance
                        <= JUNCTION_PREPARE_DISTANCE
                    ):
                        junction_npc = spawn_junction_npc(
                            world,
                            junction_spawn_transforms,
                        )

                        if junction_npc is not None:
                            junction_npc_spawned = True

                    # 18 m 이내: 돌발 차량 출발
                    if (
                        junction_npc_spawned
                        and not junction_npc_triggered
                        and junction_distance
                        <= JUNCTION_TRIGGER_DISTANCE
                    ):
                        launch_junction_npc(
                            junction_npc
                        )

                        junction_npc_triggered = True
                        junction_npc_launch_time = (
                            time.monotonic()
                        )

                    # 출발 후 계속 직진
                    if (
                        junction_npc_triggered
                        and junction_npc is not None
                        and junction_npc.is_alive
                    ):
                        junction_npc.apply_control(
                            carla.VehicleControl(
                                throttle=JUNCTION_NPC_THROTTLE,
                                steer=0.0,
                                brake=0.0,
                                hand_brake=False,
                            )
                        )

                        elapsed = (
                            time.monotonic()
                            - junction_npc_launch_time
                        )

                        if elapsed >= JUNCTION_NPC_LIFETIME:
                            completed_junction_id = (
                                junction_current.id
                            )

                            destroy_junction_npc(
                                junction_npc
                            )

                            junction_processed_ids.add(
                                completed_junction_id
                            )

                            junction_npc = None
                            junction_current = None
                            junction_current_wp = None
                            junction_spawn_transforms = []
                            junction_npc_spawned = False
                            junction_npc_triggered = False
                            junction_npc_launch_time = 0.0

                            # 버튼 1회 -> 시나리오 1회
                            junction_scenario_armed = False

                    # NPC가 출발하기 전에 EGO가 교차로를 거의 지나가면
                    # 해당 시나리오는 종료한다.
                    elif (
                        not junction_npc_triggered
                        and junction_distance <= 2.0
                    ):
                        junction_processed_ids.add(
                            junction_current.id
                        )

                        destroy_junction_npc(
                            junction_npc
                        )

                        junction_npc = None
                        junction_current = None
                        junction_current_wp = None
                        junction_spawn_transforms = []
                        junction_npc_spawned = False
                        junction_npc_triggered = False
                        junction_npc_launch_time = 0.0
                        junction_scenario_armed = False

            # =================================================
            # UDP - newest packet only
            # =================================================

            latest_packet = None

            while True:
                try:
                    data, _ = sock.recvfrom(1024)
                    latest_packet = data
                except BlockingIOError:
                    break

            if latest_packet is not None:
                try:
                    values = latest_packet.decode("utf-8").strip().split(",")

                    if len(values) == 4:
                        accel_percent = int(
                            clamp(int(values[0]), 0, 100)
                        )

                        brake_percent = int(
                            clamp(int(values[1]), 0, 100)
                        )

                        steer_input = clamp(
                            float(values[2]), -1.0, 1.0
                        )

                        received_gear = values[3].strip().upper()

                        if received_gear in ("P", "R", "D"):
                            hardware_gear = received_gear

                        last_packet_time = time.monotonic()

                except (ValueError, UnicodeDecodeError):
                    pass

            # =================================================
            # CONTROL INPUT
            # =================================================

            keys = pygame.key.get_pressed()

            communication_lost = False

            # 현재 속도에 맞는 조향 게인 계산
            current_speed_kmh = get_speed_kmh(vehicle)
            current_steering_gain = get_speed_steering_gain(
                current_speed_kmh
            )

            if control_mode == "KEYBOARD":
                throttle, brake, steer = keyboard.update(
                    keys,
                    milliseconds,
                )

                target_throttle = throttle
                target_brake = brake
                driver_throttle = target_throttle
                driver_brake = target_brake

                # 저속에서는 manual_control.py의 키보드 조향 감각을 그대로 두고,
                # 속도가 올라갈수록 같은 비율로 조향량을 줄인다.
                keyboard_steering_scale = (
                    current_steering_gain
                    / STEERING_GAIN_LOW_SPEED
                )

                target_steer = steer * keyboard_steering_scale
                driver_steer_for_override = steer

            else:
                communication_lost = (
                    time.monotonic() - last_packet_time
                    > FAILSAFE_TIMEOUT
                )

                # HARDWARE 모드에서는 Raspberry Pi의 실제 시프터 상태가
                # CARLA 기어의 기준값이다.
                if not communication_lost:
                    if hardware_gear != requested_gear:
                        requested_gear = hardware_gear
                        gear_change_pending = (
                            requested_gear != applied_gear
                        )

                if communication_lost:
                    target_throttle = 0.0
                    target_brake = 1.0
                    target_steer = 0.0
                    driver_steer_for_override = 0.0

                else:
                    target_throttle = map_accelerator(accel_percent)

                    if brake_percent <= BRAKE_DEADZONE:
                        target_brake = 0.0
                    else:
                        target_brake = brake_percent / 100.0

                    target_steer = (
                        steer_input
                        * current_steering_gain
                    )

                    driver_steer_for_override = (
                        steer_input
                    )

                    if abs(target_steer) < STEERING_DEADZONE:
                        target_steer = 0.0

                driver_throttle = target_throttle
                driver_brake = target_brake

                # 실제 페달/G29 입력만 가볍게 필터링
                smooth_throttle += THROTTLE_ALPHA * (
                    target_throttle - smooth_throttle
                )

                smooth_brake += BRAKE_ALPHA * (
                    target_brake - smooth_brake
                )

                smooth_steer += STEER_ALPHA * (
                    target_steer - smooth_steer
                )

                target_throttle = clamp(smooth_throttle, 0.0, 1.0)
                target_brake = clamp(smooth_brake, 0.0, 1.0)
                target_steer = clamp(smooth_steer, -1.0, 1.0)

            # =================================================
            # BRAKE BOOST
            #
            # KEYBOARD / HARDWARE 모두 동일하게 적용.
            # 최대값은 CARLA VehicleControl 범위인 1.0으로 제한.
            # =================================================

            target_brake = clamp(
                target_brake * BRAKE_INPUT_GAIN,
                0.0,
                1.0
            )

            # brake priority
            if target_brake > 0.02:
                target_throttle = 0.0

            # =================================================
            # SAFE P / R / D CHANGE
            #
            # 주행 중 D -> R, R -> D, P 변경을 바로 적용하지 않고
            # 1 km/h 이하가 될 때까지 자동 제동한 뒤 변경한다.
            # =================================================

            changing_gear = False

            if gear_change_pending:
                speed_kmh = get_speed_kmh(vehicle)

                if speed_kmh > GEAR_CHANGE_SPEED_KMH:
                    changing_gear = True
                    target_throttle = 0.0
                    target_brake = 1.0

                else:
                    applied_gear = requested_gear
                    gear_change_pending = False

                    keyboard.reset()
                    smooth_throttle = 0.0
                    smooth_brake = 0.0

                    print()
                    print(
                        f">>> GEAR CHANGED: {applied_gear}"
                    )

            # P에서는 가속을 완전히 차단하고 정차 유지
            if applied_gear == "P":
                target_throttle = 0.0
                target_brake = 1.0

            creep_throttle = 0.0
            if not gear_change_pending and not changing_gear and not communication_lost:
                creep_throttle = calculate_creep_throttle(
                    applied_gear, driver_throttle, driver_brake,
                    current_speed_kmh,
                )
            if creep_throttle > 0.0:
                target_throttle = creep_throttle

            # =================================================
            # REALISTIC LANE FOLLOWING ASSIST
            # =================================================

            lane_assist_correction = 0.0
            lane_assist_lateral_error = 0.0
            lane_assist_heading_error = 0.0

            lfa_now = time.monotonic()
            lane_dt = clamp(
                milliseconds / 1000.0,
                0.0,
                0.05,
            )

            if lane_assist_enabled:
                (
                    raw_lfa_correction,
                    lane_assist_lateral_error,
                    lane_assist_heading_error,
                    lfa_raw_state,
                    lfa_params,
                ) = calculate_realistic_lfa(
                    world,
                    vehicle,
                    current_speed_kmh,
                    applied_gear,
                )

                # ---------------------------------------------
                # HIGH-SPEED DAMPING
                #
                # If lateral error is changing quickly, damp the
                # correction before it crosses the lane center.
                # This is the main anti-oscillation term in v11.
                # ---------------------------------------------
                if (
                    lfa_raw_state == "ACTIVE"
                    and lane_dt > 0.0001
                ):
                    if lfa_lateral_error_valid:
                        lateral_rate = (
                            lane_assist_lateral_error
                            - lfa_prev_lateral_error
                        ) / lane_dt

                        lateral_rate = clamp(
                            lateral_rate,
                            -LFA_MAX_LATERAL_RATE_MPS,
                            LFA_MAX_LATERAL_RATE_MPS,
                        )

                        lfa_lateral_rate_filtered += (
                            LFA_LATERAL_RATE_FILTER_ALPHA
                            * (
                                lateral_rate
                                - lfa_lateral_rate_filtered
                            )
                        )

                    else:
                        lfa_lateral_rate_filtered = 0.0
                        lfa_lateral_error_valid = True

                    lfa_prev_lateral_error = (
                        lane_assist_lateral_error
                    )

                    if (
                        current_speed_kmh
                        > LFA_HIGH_SPEED_CONTROLLER_START_KMH
                    ):
                        raw_lfa_correction += (
                            lfa_params["d_gain"]
                            * lfa_lateral_rate_filtered
                        )

                        raw_lfa_correction = clamp(
                            raw_lfa_correction,
                            -lfa_params["max_corr"],
                            lfa_params["max_corr"],
                        )

                else:
                    lfa_lateral_error_valid = False
                    lfa_lateral_rate_filtered = move_toward(
                        lfa_lateral_rate_filtered,
                        0.0,
                        3.0 * lane_dt,
                    )

                override_scale = get_lfa_override_scale(
                    driver_steer_for_override
                )

                if override_scale <= 0.0:
                    lfa_override_until = (
                        lfa_now + LFA_OVERRIDE_HOLD_SEC
                    )

                override_hold_active = (
                    lfa_now < lfa_override_until
                )

                in_junction = (
                    lfa_raw_state == "JUNCTION"
                )

                if in_junction:
                    lfa_was_in_junction = True
                    lfa_reacquire_after = (
                        lfa_now
                        + LFA_JUNCTION_REACQUIRE_SEC
                    )

                elif lfa_was_in_junction:
                    lfa_was_in_junction = False
                    lfa_reacquire_after = (
                        lfa_now
                        + LFA_JUNCTION_REACQUIRE_SEC
                    )

                reacquire_wait = (
                    lfa_now < lfa_reacquire_after
                )

                assist_available = (
                    lfa_raw_state == "ACTIVE"
                    and not override_hold_active
                    and not reacquire_wait
                )

                if assist_available:
                    raw_lfa_correction *= override_scale

                    engage_rate = (
                        1.0
                        / max(LFA_ENGAGE_RAMP_SEC, 0.01)
                    )

                    lfa_engage_level = move_toward(
                        lfa_engage_level,
                        1.0,
                        engage_rate * lane_dt,
                    )

                    raw_lfa_correction *= (
                        lfa_engage_level
                    )

                    filtered_lfa = (
                        lane_assist_smoothed
                        + lfa_params["alpha"]
                        * (
                            raw_lfa_correction
                            - lane_assist_smoothed
                        )
                    )

                    max_delta = (
                        lfa_params["rate"]
                        * lane_dt
                    )

                    lane_assist_smoothed = move_toward(
                        lane_assist_smoothed,
                        filtered_lfa,
                        max_delta,
                    )

                    lane_assist_correction = (
                        lane_assist_smoothed
                    )

                    if override_scale < 1.0:
                        lane_assist_state = "DRIVER_OVERRIDE"
                    else:
                        lane_assist_state = "ACTIVE"

                else:
                    lfa_engage_level = move_toward(
                        lfa_engage_level,
                        0.0,
                        lane_dt / 0.30,
                    )

                    lane_assist_smoothed = move_toward(
                        lane_assist_smoothed,
                        0.0,
                        lfa_params["rate"] * 1.5 * lane_dt,
                    )

                    lane_assist_correction = (
                        lane_assist_smoothed
                    )

                    if override_hold_active:
                        lane_assist_state = "OVERRIDE_HOLD"

                    elif reacquire_wait:
                        lane_assist_state = "REACQUIRE"

                    else:
                        lane_assist_state = lfa_raw_state

                target_steer = clamp(
                    target_steer
                    + lane_assist_correction,
                    -1.0,
                    1.0,
                )

            else:
                lane_assist_state = "OFF"
                lane_assist_smoothed = 0.0
                lane_assist_correction = 0.0
                lfa_engage_level = 0.0
                lfa_lateral_rate_filtered = 0.0
                lfa_lateral_error_valid = False

            # =================================================
            # APPLY VEHICLE CONTROL
            # =================================================

            control = carla.VehicleControl()
            control.throttle = clamp(target_throttle, 0.0, 1.0)
            control.brake = clamp(target_brake, 0.0, 1.0)
            control.steer = clamp(target_steer, -1.0, 1.0)

            if applied_gear == "P":
                control.hand_brake = True
                control.reverse = False

            elif applied_gear == "R":
                control.hand_brake = False
                control.reverse = True

            else:  # D
                control.hand_brake = False
                control.reverse = False

            control.manual_gear_shift = False

            safety_control_receiver.apply(
                control, time.monotonic(), gear=applied_gear,
                speed_kmh=current_speed_kmh, creep_throttle=creep_throttle,
                driver_throttle=driver_throttle,
            )
            vehicle.apply_control(control)

            # Read this frame's final LFA value only after vehicle control is applied.
            lane_ffb_sender.send(
                time.monotonic(), lane_assist_enabled,
                lane_assist_state, lane_assist_correction,
            )

            # =================================================
            # RENDER
            # =================================================

            display.fill((0, 0, 0))
            camera.render(display)
            pygame.display.flip()

            # =================================================
            # PC -> RASPBERRY PI SAFETY ECU - JSON / 10 Hz
            #
            # UDP 5006 exact schema:
            # {
            #   source,
            #   sequence,
            #   vehicle_speed_kmh,
            #   obstacle_id,
            #   distance_m,
            #   closing_speed_mps,
            #   ttc_s
            # }
            #
            # - No obstacle:
            #   obstacle_id/distance/closing/ttc = null
            #
            # - O-key virtual obstacle:
            #   fixed positive ID 900001
            #
            # - Real front/junction vehicle:
            #   actual CARLA actor.id
            # =================================================

            now = time.monotonic()

            if (
                now - last_telemetry_send_time
                >= TELEMETRY_SEND_INTERVAL
            ):
                telemetry_payload = build_safety_telemetry(
                    world,
                    vehicle,
                    telemetry_sequence,
                    virtual_obstacle_active,
                    virtual_obstacle_distance,
                    virtual_obstacle_materialized=(night_obstacle is not None),
                    gear=applied_gear,
                    front_distance_sensor=front_distance_sensor,
                )

                # Radar / Safety telemetry diagnostic (1 Hz)
                if now - last_radar_debug_time >= 1.0:
                    radar_display_distance = (
                        front_distance_sensor.get_safety_distance()
                        if front_distance_sensor is not None
                        else None
                    )

                    radar_safety_distance = (
                        front_distance_sensor.get_safety_corridor_distance()
                        if front_distance_sensor is not None
                        else None
                    )

                    print(
                        "\n[OBSTACLE DEBUG] "
                        f"GEAR={applied_gear} "
                        f"RADAR_DISPLAY={radar_display_distance} "
                        f"RADAR_SAFETY={radar_safety_distance} "
                        f"TX_DISTANCE={telemetry_payload['distance_m']} "
                        f"TX_TTC={telemetry_payload['ttc_s']} "
                        f"TX_OBSTACLE_ID={telemetry_payload['obstacle_id']}"
                    )

                    last_radar_debug_time = now

                telemetry_message = json.dumps(
                    telemetry_payload,
                    separators=(",", ":"),
                    ensure_ascii=False,
                )

                try:
                    telemetry_sock.sendto(
                        telemetry_message.encode("utf-8"),
                        (
                            RPI_IP,
                            RPI_TELEMETRY_PORT,
                        ),
                    )
                except OSError:
                    # 네트워크가 잠깐 끊겨도 CARLA 주행은 계속
                    pass

                telemetry_sequence = (
                    telemetry_sequence + 1
                ) & 0xFFFF

                last_telemetry_send_time = now

            # Independent UI update after the final LFA correction is applied.
            ui_now = time.monotonic()
            if ui_now - last_ui_send_time >= UI_SEND_INTERVAL:
                ui_payload = {
                    "type": "carla_ui_state",
                    "night_mode": bool(night_scenario_started),
                    "lane_assist_enabled": bool(lane_assist_enabled),
                    "lane_assist_active": bool(
                        lane_assist_enabled
                        and lane_assist_state == "ACTIVE"
                        and abs(lane_assist_correction) > 0.003
                    ),
                }
                ui_message = json.dumps(ui_payload, separators=(",", ":"))
                try:
                    ui_sock.sendto(
                        ui_message.encode("utf-8"),
                        (RPI_UI_IP, RPI_UI_PORT),
                    )
                except OSError:
                    # A failed/dropped UI update must not stop driving.
                    pass
                last_ui_send_time = ui_now

            # =================================================
            # STATUS - 4 Hz only
            # =================================================

            now = time.monotonic()

            if now - last_status_time >= 0.25:
                speed_kmh = get_speed_kmh(vehicle)
                gear_text = applied_gear

                if changing_gear:
                    status = (
                        f"GEAR {applied_gear}->{requested_gear}"
                    )
                elif control_mode == "HARDWARE" and communication_lost:
                    status = "FAIL SAFE"
                else:
                    status = "RUNNING"

                fps = clock.get_fps()

                front_distance = None

                if (
                    virtual_obstacle_active
                    and virtual_obstacle_distance is not None
                ):
                    # 시나리오가 활성화된 동안에는 화면/콘솔 DIST도
                    # 가상 시나리오 거리를 표시한다.
                    front_distance = (
                        virtual_obstacle_distance
                    )

                elif front_distance_sensor is not None:
                    front_distance = (
                        front_distance_sensor.get_distance()
                    )

                if front_distance is None:
                    distance_text = (
                        "NO SIGN"
                    )
                else:
                    distance_text = (
                        f"{front_distance:.2f}m"
                    )

                pygame.display.set_caption(
                    f"PACE | {control_mode} | "
                    f"{gear_text} | "
                    f"{speed_kmh:.1f} km/h | "
                    f"DIST {distance_text} | "
                    f"LANE {lane_assist_state} | "
                    f"{fps:.0f} FPS | "
                    f"CAM {camera.mode}"
                )

                print(
                    "\r"
                    f"ACCEL {control.throttle:.2f}   "
                    f"BRAKE {control.brake:.2f}   "
                    f"STEER {control.steer:+.2f}   "
                    f"SPEED {speed_kmh:6.1f} km/h   "
                    f"OBSTACLE {distance_text:>8}   ",
                    end="",
                    flush=True,
                )

                last_status_time = now

    except KeyboardInterrupt:
        print()
        print("Stopping...")

    except Exception as exc:
        print()
        print("ERROR:", exc)
        raise

    finally:
        print()
        print("Cleaning up...")

        if safety_control_receiver is not None:
            safety_control_receiver.close()

        if lane_ffb_sender is not None:
            lane_ffb_sender.close()

        # vehicle stop
        try:
            if vehicle is not None:
                vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0,
                        hand_brake=True,
                    )
                )
        except Exception:
            pass

        # front distance sensor
        try:
            if front_distance_sensor is not None:
                front_distance_sensor.destroy()
        except Exception:
            pass

        # camera
        try:
            if camera is not None:
                camera.destroy()
        except Exception:
            pass

        # Dynamic junction scenario NPC
        try:
            destroy_junction_npc(
                junction_npc
            )
            junction_npc = None
        except Exception:
            pass

        # Night scenario obstacle
        try:
            if night_obstacle is not None:
                night_obstacle.destroy()
                night_obstacle = None
        except Exception:
            pass

        # Night scenario 시작 전 Weather 복원
        try:
            if world is not None and original_weather is not None:
                world.set_weather(original_weather)
        except Exception:
            pass

        # Night scenario 시작 전 가로등 상태 복원
        try:
            restore_street_lights(
                street_light_manager,
                street_lights,
                street_light_original_states,
            )
        except Exception:
            pass

        # AI traffic
        try:
            destroy_traffic_vehicles(
                client,
                traffic_vehicles,
            )
        except Exception:
            pass

        # then hero vehicle
        try:
            if vehicle is not None:
                vehicle.destroy()
        except Exception:
            pass

        try:
            if sock is not None:
                sock.close()
        except Exception:
            pass

        try:
            if telemetry_sock is not None:
                telemetry_sock.close()
        except Exception:
            pass

        try:
            if ui_sock is not None:
                ui_sock.close()
        except OSError:
            pass

        # Leave CARLA in async mode so the server does not freeze after exit
        try:
            if world is not None:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
        except Exception:
            pass

        pygame.quit()

        print("Finished")


if __name__ == "__main__":
    main()
