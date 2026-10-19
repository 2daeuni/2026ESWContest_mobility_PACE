import os
import sys

sys.path.append(
    os.path.dirname(
        os.path.dirname(
            os.path.abspath(__file__)
        )
    )
)

from control.command_mapper import CommandMapper


def assert_equal(actual, expected, message):
    if actual != expected:
        raise AssertionError(
            f"{message}: expected={expected}, actual={actual}"
        )


def test_risk_unavailable():
    mapper = CommandMapper()

    commands = mapper.map(
        risk_level=3,
        risk_available=False,
        failsafe_active=False,
    )

    assert_equal(
        commands["haptic"]["vibration_command"],
        0,
        "Haptic must be OFF when risk unavailable",
    )

    assert_equal(
        commands["motor"]["control_mode"],
        0,
        "Motor must be Normal when risk unavailable",
    )

    assert_equal(
        commands["motor"]["output_limit"],
        100,
        "Motor output limit must be 100 when risk unavailable",
    )

    assert_equal(
        commands["motor"]["command_flags"],
        0x01,
        "Motor command must be valid when risk unavailable",
    )

    print("[PASS] Risk unavailable mapping")


def test_level_0():
    mapper = CommandMapper()

    commands = mapper.map(
        risk_level=0,
        risk_available=True,
        failsafe_active=False,
    )

    assert_equal(
        commands["haptic"]["vibration_command"],
        0,
        "Level 0 haptic command",
    )

    assert_equal(
        commands["haptic"]["intensity"],
        0,
        "Level 0 haptic intensity",
    )

    assert_equal(
        commands["haptic"]["frequency"],
        0,
        "Level 0 haptic frequency",
    )

    assert_equal(
        commands["motor"]["control_mode"],
        0,
        "Level 0 motor mode",
    )

    assert_equal(
        commands["motor"]["output_limit"],
        100,
        "Level 0 motor output limit",
    )

    assert_equal(
        commands["motor"]["command_flags"],
        0x01,
        "Level 0 motor flags",
    )

    print("[PASS] Risk Level 0 mapping")


def test_level_1():
    mapper = CommandMapper()

    commands = mapper.map(
        risk_level=1,
        risk_available=True,
        failsafe_active=False,
    )

    assert_equal(
        commands["haptic"]["vibration_command"],
        1,
        "Level 1 haptic command",
    )

    assert_equal(
        commands["haptic"]["intensity"],
        30,
        "Level 1 haptic intensity",
    )

    assert_equal(
        commands["haptic"]["frequency"],
        20,
        "Level 1 haptic frequency",
    )

    assert_equal(
        commands["motor"]["control_mode"],
        0,
        "Level 1 motor mode",
    )

    assert_equal(
        commands["motor"]["output_limit"],
        100,
        "Level 1 motor output limit",
    )

    print("[PASS] Risk Level 1 mapping")

def test_level_2():
    mapper = CommandMapper()

    commands = mapper.map(
        risk_level=2,
        risk_available=True,
        failsafe_active=False,
    )

    assert_equal(
        commands["haptic"]["vibration_command"],
        1,
        "Level 2 haptic command",
    )

    assert_equal(
        commands["haptic"]["intensity"],
        45,
        "Level 2 haptic intensity",
    )

    assert_equal(
        commands["haptic"]["frequency"],
        20,
        "Level 2 haptic frequency",
    )

    assert_equal(
        commands["motor"]["control_mode"],
        1,
        "Level 2 motor mode",
    )

    assert_equal(
        commands["motor"]["output_limit"],
        30,
        "Level 2 motor output limit",
    )

    print("[PASS] Risk Level 2 mapping")


def test_level_3():
    mapper = CommandMapper()

    commands = mapper.map(
        risk_level=3,
        risk_available=True,
        failsafe_active=False,
    )

    assert_equal(
        commands["haptic"]["vibration_command"],
        1,
        "Level 3 haptic command",
    )

    assert_equal(
        commands["haptic"]["intensity"],
        65,
        "Level 3 haptic intensity",
    )

    assert_equal(
        commands["haptic"]["frequency"],
        30,
        "Level 3 haptic frequency",
    )

    assert_equal(
        commands["motor"]["control_mode"],
        2,
        "Level 3 motor mode",
    )

    assert_equal(
        commands["motor"]["output_limit"],
        0,
        "Level 3 motor output limit",
    )

    assert_equal(
        commands["motor"]["command_flags"],
        0x01,
        "Level 3 motor flags",
    )

    print("[PASS] Risk Level 3 mapping")


def test_failsafe_override():
    mapper = CommandMapper()

    commands = mapper.map(
        risk_level=3,
        risk_available=True,
        failsafe_active=True,
    )

    assert_equal(
        commands["haptic"]["vibration_command"],
        0,
        "Fail-safe must force haptic OFF",
    )

    assert_equal(
        commands["haptic"]["intensity"],
        0,
        "Fail-safe haptic intensity",
    )

    assert_equal(
        commands["haptic"]["frequency"],
        0,
        "Fail-safe haptic frequency",
    )

    assert_equal(
        commands["motor"]["control_mode"],
        2,
        "Fail-safe must force Motor Stop",
    )

    assert_equal(
        commands["motor"]["output_limit"],
        0,
        "Fail-safe motor output limit",
    )

    assert_equal(
        commands["motor"]["command_flags"],
        0x03,
        "Fail-safe motor flags",
    )

    print("[PASS] Fail-safe override")


def test_failsafe_has_priority_over_unavailable():
    mapper = CommandMapper()

    commands = mapper.map(
        risk_level=0,
        risk_available=False,
        failsafe_active=True,
    )

    assert_equal(
        commands["motor"]["control_mode"],
        2,
        "Fail-safe priority over risk unavailable",
    )

    assert_equal(
        commands["motor"]["output_limit"],
        0,
        "Fail-safe output limit priority",
    )

    assert_equal(
        commands["motor"]["command_flags"],
        0x03,
        "Fail-safe flags priority",
    )

    print("[PASS] Fail-safe priority")


def run_all_tests():
    print("=== CommandMapper Tests ===")

    test_risk_unavailable()
    test_level_0()
    test_level_1()
    test_level_2()
    test_level_3()
    test_failsafe_override()
    test_failsafe_has_priority_over_unavailable()

    print("===========================")
    print("[PASS] ALL COMMAND MAPPER TESTS")


if __name__ == "__main__":
    run_all_tests()
