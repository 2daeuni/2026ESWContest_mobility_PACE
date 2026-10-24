"""
이전/현재 표시 상태를 비교해서 EVENT LOG 항목 생성.
severity: info | ok | caution | warning | danger | fault
"""

import time


LINK_LABELS = [
    ("link_safety", "Safety ECU"),
    ("link_pedal", "Pedal ECU"),
    ("link_haptic", "Haptic ECU"),
    ("link_motor", "Motor ECU"),
    ("link_carla", "CARLA"),
]

LEVEL_SEVERITY = {0: "ok", 1: "caution", 2: "warning", 3: "danger"}


def _stamp():
    return time.strftime("%H:%M:%S")


def diff(prev, cur):
    """prev 가 None 이면 (첫 프레임) 이벤트 생성 안 함."""
    events = []
    if prev is None:
        return events

    def add(severity, text):
        events.append({"time": _stamp(), "severity": severity, "text": text})

    for key, label in LINK_LABELS:
        a, b = prev.get(key), cur.get(key)
        if a == b:
            continue
        if b is True:
            add("ok", "{} 연결됨".format(label))
        elif b is False and a is True:
            add("fault", "{} 연결 끊김".format(label))

    if prev.get("risk_level") != cur.get("risk_level") and cur.get("risk_available"):
        level = cur["risk_level"]
        add(LEVEL_SEVERITY.get(level, "info"), "Risk Level L{} ({}) · {}".format(
            level, cur["risk_name"], cur.get("risk_cause_ko") or "판정 근거 확인 중"))
    elif (cur.get("risk_available") and cur.get("risk_level") in (1, 2, 3)
          and prev.get("risk_cause") != cur.get("risk_cause")):
        add("info", "위험 원인: {}".format(cur.get("risk_cause_ko") or "판정 근거 확인 중"))
    elif prev.get("risk_available") and not cur.get("risk_available") and cur.get("link_safety"):
        add("info", "위험도 판단 불가 상태")

    if cur.get("failsafe_active") and not prev.get("failsafe_active"):
        add("fault", "Fail-Safe 진입: {}".format(cur.get("failsafe_reason_ko") or "-"))
    elif cur.get("failsafe_active") and prev.get("failsafe_reason") != cur.get("failsafe_reason"):
        add("fault", "Fail-Safe 원인 변경: {}".format(cur.get("failsafe_reason_ko") or "-"))
    elif prev.get("failsafe_active") and not cur.get("failsafe_active"):
        add("ok", "Fail-Safe 해제")

    if cur.get("system_phase") == "WAITING" and (
        prev.get("system_phase") != "WAITING"
        or prev.get("waiting_reason") != cur.get("waiting_reason")
    ):
        add("info", cur.get("waiting_reason_ko") or "시스템 초기화 중")
    elif cur.get("system_phase") == "READY" and prev.get("system_phase") != "READY":
        add("ok", "시스템 준비 완료")

    if cur.get("haptic_cmd_on") and not prev.get("haptic_cmd_on"):
        add("warning", "Haptic On ({:.0f}%, {:.0f}Hz)".format(
            cur.get("haptic_cmd_intensity") or 0, cur.get("haptic_cmd_frequency") or 0))
    elif prev.get("haptic_cmd_on") and cur.get("haptic_cmd_on") is False:
        add("info", "Haptic Off")
    elif (
        cur.get("haptic_cmd_on")
        and prev.get("haptic_cmd_intensity") != cur.get("haptic_cmd_intensity")
    ):
        add("warning", "Haptic {:.0f}%, {:.0f}Hz".format(
            cur.get("haptic_cmd_intensity") or 0, cur.get("haptic_cmd_frequency") or 0))

    if prev.get("motor_limit") != cur.get("motor_limit") and cur.get("motor_limit") is not None:
        if cur.get("motor_limited"):
            add("warning", "Motor Limit {:.0f}%".format(cur["motor_limit"]))
        elif prev.get("motor_limited"):
            add("ok", "Motor Limit 해제")

    if cur.get("brake_now") and not prev.get("brake_now"):
        add("danger", "BRAKE NOW 경고 표시")

    new_faults = set(cur.get("motor_faults") or []) - set(prev.get("motor_faults") or [])
    for f in sorted(new_faults):
        add("fault", "모터 ECU 고장 비트: {}".format(f))
    if prev.get("speed_source") != cur.get("speed_source") and cur.get("speed_source"):
        add("info", "차속 Source: {}".format(cur["speed_source"]))

    if cur.get("pedal_sensor_status") and cur.get("pedal_sensor_status") != prev.get("pedal_sensor_status"):
        add("fault", "페달 센서 이상 (0x{:02X})".format(cur["pedal_sensor_status"]))

    return events
