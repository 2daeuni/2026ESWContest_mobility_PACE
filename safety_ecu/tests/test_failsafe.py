import os
import sys

sys.path.append(
    os.path.dirname(
        os.path.dirname(
            os.path.abspath(__file__)
        )
    )
)

from control.failsafe import FailSafeManager
from control.state import SystemState


def assert_equal(actual, expected, message):
    if actual != expected:
        raise AssertionError(
            f"{message}: expected={expected}, actual={actual}"
        )


def test_tof_not_required():
    state = SystemState()
    failsafe = FailSafeManager()
    state.tof_connected = False
    state.pedal_connected = True
    state.pedal_sensor_status = 0x00

    assert_equal(failsafe.evaluate(state), False, "Retired ToF must not trigger fail-safe")
    assert_equal(failsafe.reason, None, "Retired ToF must not set a reason")
    state.carla_connected = True
    assert_equal(failsafe.evaluate(state, "CARLA"), False, "CARLA must work without ToF")
    state.carla_connected = False
    state.carla_link_state = "LOST"
    assert_equal(failsafe.evaluate(state, "CARLA"), True, "CARLA timeout must remain protected")
    assert_equal(failsafe.reason, "CARLA_COMMUNICATION_ERROR", "CARLA fault changed")
    state.pedal_connected = False
    state.pedal_ever_connected = True
    failsafe.evaluate(state)
    assert_equal(failsafe.reason, "PEDAL_COMMUNICATION_ERROR", "Pedal fault changed")
    print("[PASS] ToF retired; CARLA and pedal protection preserved")


def test_normal_state():
    state = SystemState()
    failsafe = FailSafeManager()

    state.tof_connected = True
    state.pedal_connected = True
    state.pedal_sensor_status = 0x00

    active = failsafe.evaluate(state)

    assert_equal(
        active,
        False,
        "Normal state should not activate fail-safe",
    )

    assert_equal(
        failsafe.reason,
        None,
        "Normal state should clear fail-safe reason",
    )

    print("[PASS] Normal state")


def test_pedal_sensor_error():
    state = SystemState()
    failsafe = FailSafeManager()

    state.tof_connected = True
    state.pedal_connected = True
    state.pedal_sensor_status = 0x01

    active = failsafe.evaluate(state)

    assert_equal(
        active,
        True,
        "Pedal sensor error should activate fail-safe",
    )

    assert_equal(
        failsafe.reason,
        "PEDAL_SENSOR_ERROR",
        "Wrong fail-safe reason for pedal sensor error",
    )

    print("[PASS] Pedal sensor error")


def test_pedal_disconnected_failsafe():
    state = SystemState()
    failsafe = FailSafeManager()

    state.tof_connected = True
    state.pedal_connected = False
    state.pedal_ever_connected = True

    active = failsafe.evaluate(state)

    assert_equal(
        active,
        True,
        "Pedal disconnect should activate fail-safe",
    )

    assert_equal(
        failsafe.reason,
        "PEDAL_COMMUNICATION_ERROR",
        "Wrong fail-safe reason for Pedal communication failure",
    )

    print("[PASS] Pedal communication fail-safe")


def test_recovery():
    state = SystemState()
    failsafe = FailSafeManager()

    # Start with fail-safe active
    state.tof_connected = False
    state.pedal_connected = True
    state.pedal_sensor_status = 0x04

    failsafe.evaluate(state)

    assert_equal(
        failsafe.active,
        True,
        "Fail-safe activation before recovery failed",
    )

    # Recover all fault conditions
    state.tof_connected = True
    state.pedal_connected = True
    state.pedal_sensor_status = 0x00

    failsafe.evaluate(state)

    assert_equal(
        failsafe.active,
        False,
        "Fail-safe did not clear after recovery",
    )

    assert_equal(
        failsafe.reason,
        None,
        "Fail-safe reason did not clear after recovery",
    )

    print("[PASS] Fail-safe recovery")

def run_all_tests():
    print("=== FailSafeManager Tests ===")

    test_tof_not_required()
    test_normal_state()
    test_pedal_sensor_error()
    test_pedal_disconnected_failsafe()
    test_recovery()

    print("=============================")
    print("[PASS] ALL FAIL-SAFE TESTS")


if __name__ == "__main__":
    run_all_tests()
