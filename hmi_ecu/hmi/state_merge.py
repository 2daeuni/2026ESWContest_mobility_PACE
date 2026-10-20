"""
CAN 수신값 + Safety ECU UDP 상태를 하나의 HMI 표시 상태로 합침.

원칙
1. 페달/햅틱/모터 명령은 버스에 실제로 흐르는 CAN 값이 우선 (가장 직접적인 사실)
2. BNS / PMS / Total / 거리 / TTC / 차속 / Fail-Safe 는 Safety ECU(UDP) 만 알고 있음
3. Safety ECU UDP 가 없어도 0x110 의 risk_level 로 위험 단계는 표시 가능 (fallback)
4. 데이터가 없거나 오래되면 절대 "정상"으로 표시하지 않음 → offline / 판단불가로 표시

Qt 의존성 없는 순수 함수라 단위테스트 가능.
"""

import time

from hmi import config
from hmi.can_decoder import HAPTIC_FAULT_NAMES, MOTOR_MODE_NAMES
from hmi.demo_drive import threshold_labels

DEMO_THRESHOLD_LABELS = threshold_labels()


RISK_NAMES = {0: "NORMAL", 1: "CAUTION", 2: "WARNING", 3: "DANGER"}
RISK_SHORT = {0: "Low Risk", 1: "Caution", 2: "Warning", 3: "High Risk"}

FAILSAFE_REASON_KO = {
    "TOF_COMMUNICATION_ERROR": "거리 센서(ToF) 통신 오류",
    "PEDAL_COMMUNICATION_ERROR": "페달 ECU 통신 오류",
    "PEDAL_SENSOR_ERROR": "페달 센서 이상",
    "HAPTIC_COMMUNICATION_ERROR": "햅틱 ECU 통신 오류",
    "MOTOR_FAILSAFE_MODE": "모터 정지 명령",
    "MOTOR_NO_RESPONSE": "모터 ECU 응답 없음 · 정지 명령",
    "MOTOR_ESTOP": "모터 비상정지(E-STOP) · 페달 떼면 복구",
    "MOTOR_FAULT": "모터 ECU 고장",
    "CARLA_COMMUNICATION_ERROR": "CARLA 통신 오류",
}

GEAR_NAMES = {0: "P", 1: "R", 2: "N", 3: "D"}


def _num(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        if value != value:  # NaN
            return None
        return float(value)
    return None


def _norm_score(value, score_max):
    v = _num(value)
    if v is None:
        return None
    if score_max and score_max > 0:
        v = v / score_max
    return max(0.0, min(1.0, v))


def _conditions(value):
    """
    Safety ECU 판정 근거 → {"1": [거리, TTC, 페달], "2": [...], "3": [...]} (bool)
    허용 형식: 위 dict, 또는 [[L1 3개], [L2 3개], [L3 3개]]
    """
    if isinstance(value, (list, tuple)) and len(value) == 3:
        value = {str(i + 1): v for i, v in enumerate(value)}
    if not isinstance(value, dict):
        return None
    out = {}
    for lv in ("1", "2", "3"):
        row = value.get(lv, value.get(int(lv)) if isinstance(value, dict) else None)
        if not isinstance(row, (list, tuple)) or len(row) != 3:
            return None
        out[lv] = [bool(x) for x in row]
    return out


def _gear(value):
    if isinstance(value, str) and value.upper() in ("P", "R", "N", "D"):
        return value.upper()
    if isinstance(value, int) and not isinstance(value, bool):
        return GEAR_NAMES.get(value)
    return None


def merge(can_store, udp_store, now=None):
    """
    can_store : {name: {"data": dict, "t": float}}
    udp_store : {"data": dict, "t": float} or None
    """
    if now is None:
        now = time.monotonic()

    def can_fresh(name):
        entry = can_store.get(name)
        return entry is not None and (now - entry["t"]) < config.CAN_FRESH_TIMEOUT

    def can_data(name):
        return can_store[name]["data"] if can_fresh(name) else None

    udp_fresh = udp_store is not None and (now - udp_store["t"]) < config.SAFETY_UDP_TIMEOUT
    u = udp_store["data"] if udp_fresh else {}

    pedal_can = can_data("PEDAL_STATUS")
    hcmd_can = can_data("HAPTIC_COMMAND")
    mcmd_can = can_data("MOTOR_COMMAND")
    hsta_can = can_data("HAPTIC_STATUS")
    msta_can = can_data("MOTOR_STATUS")

    s = {}

    # -------------------------------------------------
    # Pedal
    # -------------------------------------------------
    if pedal_can is not None:
        s["accelerator"] = pedal_can["accelerator"]
        s["brake"] = pedal_can["brake"]
        s["accelerator_rate"] = pedal_can["accelerator_rate"]
        s["brake_rate"] = pedal_can["brake_rate"]
        s["pedal_sensor_status"] = pedal_can["sensor_status"]
        s["pedal_source"] = "CAN"
    elif udp_fresh and u.get("pedal_connected"):
        s["accelerator"] = _num(u.get("accelerator"))
        s["brake"] = _num(u.get("brake"))
        s["accelerator_rate"] = _num(u.get("accelerator_rate"))
        s["brake_rate"] = _num(u.get("brake_rate"))
        s["pedal_sensor_status"] = u.get("pedal_sensor_status")
        s["pedal_source"] = "UDP"
    else:
        s["accelerator"] = None
        s["brake"] = None
        s["accelerator_rate"] = None
        s["brake_rate"] = None
        s["pedal_sensor_status"] = None
        s["pedal_source"] = "-"

    # -------------------------------------------------
    # Links (True / False / None=알 수 없음)
    # -------------------------------------------------
    any_can = any(can_fresh(n) for n in can_store)

    s["link_can"] = any_can
    s["link_safety_udp"] = udp_fresh
    s["link_safety"] = udp_fresh or hcmd_can is not None or mcmd_can is not None
    s["link_pedal"] = pedal_can is not None or bool(u.get("pedal_connected"))
    s["link_haptic"] = hsta_can is not None or bool(u.get("haptic_connected"))

    if msta_can is not None:
        s["link_motor"] = True
    elif "motor_connected" in u and u.get("motor_connected") is not None:
        s["link_motor"] = bool(u.get("motor_connected"))
    elif "MOTOR_STATUS" in can_store:
        s["link_motor"] = False   # 한번은 받았는데 끊김
    else:
        s["link_motor"] = None    # 상태 메시지 없음 → 확인 불가

    # ToF hardware is retired; no required link or UI warning.
    s["link_carla"] = bool(u.get("carla_connected")) if udp_fresh and u.get("carla_connected") is not None else None

    # -------------------------------------------------
    # Environment / vehicle (Safety ECU only)
    # -------------------------------------------------
    dist_cm = _num(u.get("distance_cm")) if u.get("distance_valid") else None
    s["distance_m"] = dist_cm / 100.0 if dist_cm is not None else None
    # Safety ECU vehicle_speed 단위: 패킷의 vehicle_speed_unit → 없으면 config.SAFETY_SPEED_UNIT (m/s)
    v = _num(u.get("vehicle_speed"))
    unit = u.get("vehicle_speed_unit") or config.SAFETY_SPEED_UNIT
    s["speed_kmh"] = None if v is None else (v * 3.6 if unit == "m/s" else v)
    s["speed_source"] = u.get("speed_source") if udp_fresh and u.get("speed_source") else ("Safety ECU" if v is not None else None)
    s["operating_mode"] = u.get("operating_mode") if udp_fresh else None
    s["ttc_s"] = _num(u.get("ttc"))
    s["relative_speed_kmh"] = _num(u.get("relative_speed"))
    s["gear"] = _gear(u.get("gear"))
    s["steering"] = _num(u.get("steering"))

    # -------------------------------------------------
    # Risk
    # -------------------------------------------------
    score_max = _num(u.get("score_max")) or config.SCORE_MAX

    s["bns"] = _norm_score(u.get("bns"), score_max) if u.get("bns_available") else None
    s["pms"] = _norm_score(u.get("pms"), score_max) if u.get("pms_available") else None
    s["bns_available"] = bool(u.get("bns_available"))
    s["pms_available"] = bool(u.get("pms_available"))

    if udp_fresh:
        level = u.get("risk_level")
        level = level if level in RISK_NAMES else None
        available = bool(u.get("risk_available")) and level is not None
        s["total_risk"] = _norm_score(u.get("total_risk"), score_max) if available else None
        s["risk_source"] = "Safety ECU"
    elif hcmd_can is not None and hcmd_can["risk_level"] in (1, 2, 3):
        # UDP 없이 CAN 만 보일 때: 0x110 의 risk_level 은 "판단 불가"일 때도 0 이므로
        # 1~3 (실제 개입 명령) 만 신뢰하고 0 은 정상으로 단정하지 않음
        level = hcmd_can["risk_level"]
        available = True
        s["total_risk"] = None
        s["risk_source"] = "CAN 0x110"
    else:
        level = None
        available = False
        s["total_risk"] = None
        s["risk_source"] = "-"

    s["risk_conditions"] = _conditions(u.get("risk_conditions")) if udp_fresh and available else None
    s["risk_thresholds"] = None
    s["risk_level"] = level if available else None
    s["risk_available"] = available
    s["risk_name"] = RISK_NAMES.get(level, "N/A") if available else "N/A"
    s["risk_short"] = RISK_SHORT.get(level, "") if available else ""

    # -------------------------------------------------
    # Fail-safe
    # -------------------------------------------------
    failsafe = bool(u.get("failsafe_active")) if udp_fresh else False
    reason = u.get("failsafe_reason") if failsafe else None
    if not failsafe and msta_can is not None and msta_can.get("estop_active"):
        failsafe, reason = True, "MOTOR_ESTOP"
    elif not failsafe and msta_can is not None and msta_can.get("motor_state") == 3:
        failsafe, reason = True, "MOTOR_FAULT"
    elif not failsafe and mcmd_can is not None and mcmd_can["control_mode"] == 2:
        # safety_ecu main.py: MOTOR_STATUS 0.1 s 이상 없으면 정지 명령(E-STOP)
        failsafe = True
        reason = "MOTOR_NO_RESPONSE" if msta_can is None else "MOTOR_FAILSAFE_MODE"
    s["failsafe_active"] = failsafe
    s["failsafe_reason"] = reason
    s["failsafe_reason_ko"] = FAILSAFE_REASON_KO.get(reason, reason or "")

    # -------------------------------------------------
    # Haptic command (0x110) / status (0x200)
    # -------------------------------------------------
    if hcmd_can is not None:
        s["haptic_cmd_on"] = hcmd_can["vibration_command"] == 1
        s["haptic_cmd_intensity"] = hcmd_can["intensity"]
        s["haptic_cmd_frequency"] = hcmd_can["frequency"]
        s["haptic_cmd_source"] = "CAN"
    elif udp_fresh and "haptic_vibration_command" in u:
        s["haptic_cmd_on"] = u.get("haptic_vibration_command") == 1
        s["haptic_cmd_intensity"] = _num(u.get("haptic_intensity"))
        s["haptic_cmd_frequency"] = _num(u.get("haptic_frequency"))
        s["haptic_cmd_source"] = "UDP"
    else:
        s["haptic_cmd_on"] = None
        s["haptic_cmd_intensity"] = None
        s["haptic_cmd_frequency"] = None
        s["haptic_cmd_source"] = "-"

    if hsta_can is not None:
        s["haptic_vibrating"] = hsta_can["vibration_status"] == 1
        s["haptic_out_intensity"] = hsta_can["intensity"]
        s["haptic_out_frequency"] = hsta_can["frequency"]
        s["haptic_fault"] = HAPTIC_FAULT_NAMES.get(hsta_can["fault_code"], str(hsta_can["fault_code"]))
        s["haptic_ready"] = (
            hsta_can["fault_code"] == 0
            and hsta_can["command_timeout"] == 0
            and hsta_can["ecu_status"] == 1
        )
    else:
        s["haptic_vibrating"] = None
        s["haptic_out_intensity"] = None
        s["haptic_out_frequency"] = None
        fault = u.get("haptic_fault_code") if udp_fresh else None
        s["haptic_fault"] = HAPTIC_FAULT_NAMES.get(fault, None) if fault is not None else None
        s["haptic_ready"] = bool(u.get("haptic_available")) if udp_fresh else None

    # -------------------------------------------------
    # Motor command (0x120)
    # -------------------------------------------------
    if mcmd_can is not None:
        mode = mcmd_can["control_mode"]
        s["motor_limit"] = mcmd_can["output_limit"]
        s["motor_cmd_source"] = "CAN"
    elif udp_fresh and "motor_control_mode" in u:
        mode = u.get("motor_control_mode")
        s["motor_limit"] = _num(u.get("motor_output_limit"))
        s["motor_cmd_source"] = "UDP"
    else:
        mode = None
        s["motor_limit"] = None
        s["motor_cmd_source"] = "-"
    s["motor_mode"] = MOTOR_MODE_NAMES.get(mode, "-") if mode is not None else "-"
    s["motor_limited"] = mode in (1, 2)

    # -------------------------------------------------
    # Motor status (0x210): 실제로 적용된 값
    # -------------------------------------------------
    if msta_can is not None:
        s["motor_pwm"] = msta_can["actual_pwm"]
        s["motor_applied_limit"] = msta_can["applied_limit"]
        s["motor_state"] = msta_can["motor_state_name"]
        s["motor_faults"] = msta_can["fault_errors"]
        s["motor_speed_kmh"] = msta_can["speed_kmh"]
    else:
        s["motor_pwm"] = None
        s["motor_applied_limit"] = None
        s["motor_state"] = None
        s["motor_faults"] = []
        s["motor_speed_kmh"] = None

    # 차속 Source (노션: ACTUAL = Motor ECU, CARLA = CARLA). Safety ECU 값이 없으면 0x210 사용
    if s["speed_kmh"] is None and s["motor_speed_kmh"] is not None:
        s["speed_kmh"] = s["motor_speed_kmh"]
        s["speed_source"] = "Motor ECU"

    # -------------------------------------------------
    # Derived UI flags
    # -------------------------------------------------
    acc = s["accelerator"] or 0
    brk = s["brake"] or 0
    s["brake_now"] = (
        s["link_safety"]
        and s["risk_level"] == 3
        and acc >= config.BRAKE_NOW_ACCEL_MIN
        and brk <= config.BRAKE_NOW_BRAKE_MAX
    )

    s["banner"] = _banner(s, u, udp_fresh)
    s["missing_inputs"] = _missing_inputs(s, u, udp_fresh)

    s["demo_drive"] = False
    s["demo_carla"] = False
    s["rear_distance_m"] = None
    s["rear_kind"] = None
    s["demo_collided"] = False
    s["demo_input"] = ""

    return s


def _missing_inputs(s, u, udp_fresh):
    if not udp_fresh:
        return []
    missing = []
    if not u.get("distance_valid"):
        missing.append("거리")
    if u.get("vehicle_speed") is None:
        missing.append("차속")
    if u.get("ttc") is None:
        missing.append("TTC")
    if not u.get("pedal_connected"):
        missing.append("페달")
    return missing


def _banner(s, u, udp_fresh):
    """(severity, 한글 메시지, 영문 서브타이틀)"""
    if not s["link_safety"]:
        return {"severity": "offline", "title": "Safety ECU 신호 없음", "subtitle": "Waiting for Safety ECU"}

    if s["failsafe_active"]:
        return {
            "severity": "fault",
            "title": "Fail-Safe 동작 중 · " + (s["failsafe_reason_ko"] or "원인 미상"),
            "subtitle": "Fail-Safe Active",
        }

    if not udp_fresh and not s["risk_available"]:
        return {"severity": "info", "title": "Safety ECU 상세 상태(UDP) 미수신 · CAN 명령만 표시", "subtitle": "Waiting for Safety ECU state"}

    if not s["risk_available"]:
        missing = _missing_inputs(s, u, udp_fresh)
        detail = " (" + ", ".join(missing) + " 없음)" if missing else ""
        return {"severity": "info", "title": "위험도 판단 대기 중" + detail, "subtitle": "Risk Evaluation Unavailable"}

    return _level_banner(s)


def _level_banner(s):
    level = s["risk_level"]
    accel = (s["accelerator"] or 0) >= config.BRAKE_NOW_ACCEL_MIN

    if level == 3:
        title = "가속 페달 오조작! 브레이크를 밟으세요" if accel else "충돌 위험! 브레이크를 밟으세요"
        return {"severity": "danger", "title": title, "subtitle": "Danger — Collision Warning"}
    if level == 2:
        title = "페달 조작 확인 · 브레이크를 준비하세요"
        return {"severity": "warning", "title": title, "subtitle": "Warning — Check Pedal Input"}
    if level == 1:
        return {"severity": "caution", "title": "전방 상황에 주의하세요", "subtitle": "Caution"}
    return {"severity": "ok", "title": "정상 주행 중", "subtitle": "Normal"}


# =================================================================
# CARLA 장면 / DEMO 주행 반영
# =================================================================
def apply_carla(s, scene_packet):
    """CARLA 브리지 장면이 살아있을 때: Safety ECU 에 없는 값만 보충."""
    s["link_carla"] = True
    ego = scene_packet.get("ego") or {}
    if s.get("speed_kmh") is None or s.get("speed_source") == "Motor ECU":
        # CARLA 장면이 있으면 CARLA 속도 우선 (노션 CARLA Mode: vehicle_speed = carla_speed)
        carla_speed = _num(ego.get("speed_kmh"))
        if carla_speed is not None:
            s["speed_kmh"] = carla_speed
            s["speed_source"] = "CARLA"
    if s.get("gear") is None:
        s["gear"] = _gear(ego.get("gear"))
    rd, rk = carla_rear(scene_packet)
    s["rear_distance_m"] = rd
    s["rear_kind"] = rk
    if s.get("distance_m") is None:
        lead = scene_packet.get("lead") or {}
        s["distance_m"] = _num(lead.get("distance"))
    return s


def carla_rear(packet):
    """후방 가장 가까운 물체 (차량 / 벽·건물) 범퍼 거리 m 와 종류"""
    c = []
    r = _num((packet.get("rear") or {}).get("distance"))
    if r is not None:
        c.append((r, "vehicle"))
    r = _num(packet.get("rear_static"))
    if r is not None:
        c.append((r, "static"))
    return min(c) if c else (None, None)


def apply_carla_ui(s, ui_store, now):
    """Only UI flags; scene/map/camera/Safety state and freshness are untouched."""
    s["night_mode"] = None
    s["lane_assist_enabled"] = False
    s["lane_assist_active"] = False
    if ui_store is None or now - ui_store["t"] >= config.CARLA_UI_STATE_TIMEOUT:
        return s
    data = ui_store["data"]
    s["night_mode"] = data["night_mode"]
    s["lane_assist_enabled"] = data["lane_assist_enabled"]
    s["lane_assist_active"] = (
        data["lane_assist_enabled"] and data["lane_assist_active"]
    )
    return s


def carla_inputs(s, packet):
    """CARLA 단독 판정용 입력 (전방 거리, 접근 속도, 페달). 페달은 CAN/UDP 가 있으면 그 값."""
    ego = packet.get("ego") or {}
    lead = packet.get("lead") or {}
    cands = []
    if "radar" in packet:
        # 브리지가 Safety 와 같은 근거리 레이더 방식 결과를 보냄 → 그대로 사용 (모든 방향, None = 없음)
        r = packet.get("radar") or {}
        dist = _num(r.get("distance"))
        kind = {"vehicle": "vehicle", "walker": "vehicle"}.get(r.get("kind"), "static") if dist is not None else None
        closing = _num(r.get("closing"))
        s["obstacle_dir"] = r.get("dir") if dist is not None else None
        return _with_pedals(s, ego, dist, closing, kind)
    if s.get("gear") == "R":          # 후진: 후방 거리 + 후진 속도로 판정 (후진 중 가속 페달 오조작)
        dist, kind = carla_rear(packet)
        v = _num(packet.get("ego_v_fwd"))
        closing = -v if v is not None else None
        if dist is not None:
            cands.append((dist, closing, kind))
        lead = {}
    if _num(lead.get("distance")) is not None:
        cands.append((_num(lead.get("distance")), _num(lead.get("closing")), "vehicle"))
    if s.get("gear") != "R" and _num(packet.get("front_static")) is not None:
        v = _num(packet.get("ego_v_fwd"))
        if v is None:
            v = (_num(ego.get("speed_kmh")) or 0.0) / 3.6
        cands.append((_num(packet.get("front_static")), v, "static"))
    dist, closing, kind = min(cands, key=lambda c: c[0]) if cands else (None, None, None)
    return _with_pedals(s, ego, dist, closing, kind)


def _with_pedals(s, ego, dist, closing, kind):
    real_pedal = s.get("pedal_source") in ("CAN", "UDP") and s.get("accelerator") is not None
    if real_pedal:
        acc, brk, rate, src = s["accelerator"], s.get("brake") or 0.0, s.get("accelerator_rate"), s["pedal_source"]
    else:
        acc = (_num(ego.get("throttle")) or 0.0) * 100.0
        brk = (_num(ego.get("brake")) or 0.0) * 100.0
        rate, src = None, "CARLA"
    return {"dist": dist, "closing": closing, "kind": kind, "acc": acc, "brk": brk, "rate": rate, "pedal_src": src}


def apply_carla_local(s, inp, out, collided=False, collided_with=""):
    """DEMO + CARLA: CARLA 실제 주행 값으로 HMI 가 판정한 위험도로 경고 화면을 동작시킴 (실제 운행은 Safety ECU 판정)"""
    s["local_risk"] = True
    s["demo_drive"] = True
    s["demo_carla"] = True
    s["demo_collided"] = collided          # RESET 누를 때까지 충돌 화면 (DEMO 와 동일)
    s["demo_input"] = "PEDAL" if inp["pedal_src"] in ("CAN", "UDP") else "CARLA"
    s["haptic_cmd_source"] = "DEMO"
    s["motor_cmd_source"] = "DEMO"
    if inp["pedal_src"] == "CARLA":
        s["accelerator"] = round(inp["acc"], 1)
        s["brake"] = round(inp["brk"], 1)
        s["accelerator_rate"] = round(out["accelerator_rate"], 1)
        s["brake_rate"] = None
        s["pedal_source"] = "CARLA"
    s["distance_m"] = None if out["distance_m"] is None else round(out["distance_m"], 2)
    s["front_kind"] = inp["kind"]
    s["ttc_s"] = None if out["ttc_s"] is None else round(out["ttc_s"], 2)
    s["bns"], s["pms"], s["total_risk"] = out["bns"], out["pms"], out["total_risk"]
    s["bns_available"] = s["pms_available"] = True
    lv = out["risk_level"]
    s["risk_level"] = lv
    s["risk_available"] = True
    s["risk_name"] = RISK_NAMES[lv]
    s["risk_short"] = RISK_SHORT[lv]
    s["risk_source"] = "DEMO · CARLA (HMI 판정)"
    s["risk_conditions"] = out["conditions"]
    s["risk_thresholds"] = DEMO_THRESHOLD_LABELS
    s["failsafe_active"] = False
    s["failsafe_reason"] = None
    s["failsafe_reason_ko"] = ""
    acc = s.get("accelerator") or 0
    brk = s.get("brake") or 0
    s["brake_now"] = (not collided and lv == 3 and acc >= config.BRAKE_NOW_ACCEL_MIN
                      and brk <= config.BRAKE_NOW_BRAKE_MAX)
    s["carla_collision"] = collided
    if collided:
        s["banner"] = {"severity": "danger", "title": "충돌 발생 · RESET 을 눌러 다시 시작",
                       "subtitle": "DEMO · CARLA — Collision " + (collided_with or "")}
    else:
        b = _level_banner(s)
        d = s.get("obstacle_dir") or "전방"
        if lv >= 1 and inp["kind"] == "static":
            b["title"] = b["title"].replace("전방 상황에", d + " 장애물에")
        elif lv >= 1:
            b["title"] = b["title"].replace("전방 상황에", d + " 상황에")
        b["subtitle"] = "DEMO · CARLA — " + b["subtitle"] + ("" if d == "전방" else " · " + d)
        s["banner"] = b
    s["missing_inputs"] = []
    return s


def apply_demo(s, d):
    """
    DEMO 주행 결과로 표시 상태 덮어쓰기.
    링크 상태(ECU 연결)는 실제 값 유지, 위험/차량 값은 DEMO 값 사용.
    """
    s["demo_drive"] = True
    s["demo_input"] = d["input_source"]
    s["demo_collided"] = d["collided"]
    s["demo_start_distance"] = d["start_distance"]
    s["demo_lead_speed"] = d["lead_speed_kmh"]
    s["demo_travelled_m"] = d["travelled_m"]

    s["accelerator"] = round(d["accelerator"], 1)
    s["brake"] = round(d["brake"], 1)
    s["accelerator_rate"] = round(d["accelerator_rate"], 1)
    if d["input_source"] == "KEYBOARD":
        s["brake_rate"] = None
        s["pedal_source"] = "KEYBOARD"

    s["speed_kmh"] = d["speed_kmh"]
    s["speed_source"] = "DEMO"
    s["gear"] = "D"
    s["distance_m"] = d["distance_m"]
    s["ttc_s"] = d["ttc_s"]
    s["relative_speed_kmh"] = d["relative_speed_kmh"]

    s["bns"] = d["bns"]
    s["pms"] = d["pms"]
    s["bns_available"] = True
    s["pms_available"] = True
    s["total_risk"] = d["total_risk"]
    s["risk_level"] = d["risk_level"]
    s["risk_available"] = True
    s["risk_name"] = RISK_NAMES[d["risk_level"]]
    s["risk_short"] = RISK_SHORT[d["risk_level"]]
    s["risk_source"] = "DEMO (HMI 판정)"
    s["risk_conditions"] = d["conditions"]
    s["risk_thresholds"] = DEMO_THRESHOLD_LABELS

    # 실차 Fail-Safe(예: 벤치에서 ToF 미연결)는 DEMO 체험에 섞지 않음
    s["failsafe_active"] = False
    s["failsafe_reason"] = None
    s["failsafe_reason_ko"] = ""

    s["haptic_cmd_on"] = d["haptic_on"]
    s["haptic_cmd_intensity"] = d["haptic_intensity"]
    s["haptic_cmd_frequency"] = d["haptic_frequency"]
    s["haptic_cmd_source"] = "DEMO"
    s["motor_mode"] = d["motor_mode"]
    s["motor_limit"] = d["motor_limit"]
    s["motor_limited"] = d["motor_mode"] != "NORMAL"
    s["motor_cmd_source"] = "DEMO"

    acc = s["accelerator"] or 0
    brk = s["brake"] or 0
    s["brake_now"] = (
        not d["collided"]
        and s["risk_level"] == 3
        and acc >= config.BRAKE_NOW_ACCEL_MIN
        and brk <= config.BRAKE_NOW_BRAKE_MAX
    )

    if d["collided"]:
        s["banner"] = {"severity": "danger", "title": "충돌 발생 · RESET 을 눌러 다시 시작", "subtitle": "DEMO — Collision"}
    else:
        b = _level_banner(s)
        b["subtitle"] = "DEMO — " + b["subtitle"]
        s["banner"] = b
    s["missing_inputs"] = []
    return s
