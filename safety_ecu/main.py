import time

from canbus.receiver import CANReceiver
from canbus.sender import CANSender
from canbus.decoder import (
    PEDAL_STATUS_ID,
    HAPTIC_STATUS_ID,
    MOTOR_STATUS_ID,
    decode_pedal_status,
    decode_haptic_status,
    decode_motor_status,
)

from control.state import SystemState
from control.failsafe import FailSafeManager
from control.command_mapper import CommandMapper

from logger.pedal_logger import PedalLogger
from logger.system_logger import SystemLogger
from logger.pedal_diagnostics import PedalDiagnostics

from risk.bns import BNSCalculator
from risk.pms import PMSCalculator
from risk.risk_manager import RiskManager

from carla.receiver_v2 import CarlaReceiverV2
from carla.control_sender import CarlaControlSender
from hmi_sender import HmiSender

# -------------------------------------------------
# Environment input mode
# -------------------------------------------------
INPUT_MODE_HARDWARE = "HARDWARE"
INPUT_MODE_CARLA = "CARLA"

INPUT_MODE = INPUT_MODE_CARLA

def main():
    # -------------------------------------------------
    # Modules
    # -------------------------------------------------
    receiver = CANReceiver("can0")
    sender = CANSender("can0")

    state = SystemState()

    logger = PedalLogger(
        "pedal_data.csv"
    )

    # -------------------------------------------------
    # CARLA receiver
    # -------------------------------------------------
    carla_receiver = None

    if INPUT_MODE == INPUT_MODE_CARLA:
        carla_receiver = CarlaReceiverV2(
            host="192.168.50.2",
            port=5006,
            timeout=0.5,
        )

    system_logger = SystemLogger(
        "system_data.csv"
    )

    failsafe = FailSafeManager()
    pedal_diagnostics = PedalDiagnostics()
    command_mapper = CommandMapper()
    # Motor ECU status
    motor_status = None
    motor_last_rx_time = None

    motor_communication_fault = True
    motor_recovery_required = True
    motor_timeout_reported = False

    bns_calculator = BNSCalculator()
    pms_calculator = PMSCalculator()
    risk_manager = RiskManager()
    hmi_sender = HmiSender()
    carla_control_sender = CarlaControlSender()

    # -------------------------------------------------
    # Pedal communication monitoring
    # -------------------------------------------------
    previous_alive_counter = None
    recovery_alive_candidate = None

    last_pedal_time = time.monotonic()

    pedal_timeout_reported = False

    # -------------------------------------------------
    # Haptic ECU communication monitoring
    # -------------------------------------------------
    haptic_timeout_reported = False

    # -------------------------------------------------
    # Fail-safe monitoring
    # -------------------------------------------------
    previous_failsafe_active = None
    previous_failsafe_reason = None

    # -------------------------------------------------
    # CARLA state logging
    # -------------------------------------------------
    last_carla_log_time = 0.0
    CARLA_LOG_INTERVAL = 0.5
    last_risk_log_time = 0.0
    RISK_LOG_INTERVAL = 0.5
    last_risk_debug_time = 0.0

    # -------------------------------------------------
    # CAN command transmission
    # -------------------------------------------------
    last_command_tx_time = 0.0

    haptic_alive_counter = 0
    motor_alive_counter = 0

    try:
        hmi_sender.start()
        # -------------------------------------------------
        # Open CAN
        # -------------------------------------------------
        receiver.open()
        sender.open()

        # -------------------------------------------------
        # Start CARLA UDP receiver
        # -------------------------------------------------
        if carla_receiver is not None:
            carla_receiver.start()
            print("[Safety ECU] CARLA receiver ready")

        print("[Safety ECU] Started")

        print(
            "[Safety ECU] "
            "Waiting for PEDAL_STATUS..."
        )

        while True:
            pedal_diagnostic_sample = None
            pedal_diagnostic_accepted = False
            # =================================================
            # CARLA UDP receive and freshness monitoring
            # =================================================
            if carla_receiver is not None:
                if carla_receiver.receive():
                    state.update_carla(carla_receiver)

                if not carla_receiver.check_freshness():
                    if state.carla_connected:
                        print(
                            "[CARLA] DATA TIMEOUT"
                        )

                    state.invalidate_carla()

            # =================================================
            # CAN receive
            # =================================================
            msg = receiver.receive(
                timeout=0.001
            )

            if msg is not None:

                # ---------------------------------------------
                # PEDAL_STATUS
                # ---------------------------------------------
                if (
                    msg.arbitration_id
                    == PEDAL_STATUS_ID
                ):
                    pedal_diagnostic_sample = pedal_diagnostics.capture(
                        msg.data, msg.timestamp
                    )
                    try:
                        pedal = decode_pedal_status(
                            msg.data
                        )

                        current_alive_counter = (
                            pedal["alive_counter"]
                        )

                        alive_valid = False

                        if not state.pedal_connected:
                            if (
                                recovery_alive_candidate
                                is None
                            ):
                                recovery_alive_candidate = (
                                    current_alive_counter
                                )

                            else:
                                expected_recovery = (
                                    recovery_alive_candidate
                                    + 1
                                ) & 0xFF

                                if (
                                    current_alive_counter
                                    == expected_recovery
                                ):
                                    alive_valid = True

                                    previous_alive_counter = (
                                        current_alive_counter
                                    )

                                    recovery_alive_candidate = (
                                        None
                                    )

                                else:
                                    print(
                                        "[PEDAL] "
                                        "ALIVE RECOVERY WAIT: "
                                        f"expected="
                                        f"{expected_recovery}, "
                                        f"received="
                                        f"{current_alive_counter}"
                                    )

                                    recovery_alive_candidate = (
                                        current_alive_counter
                                    )

                        else:
                            alive_valid = True

                            if (
                                previous_alive_counter
                                is not None
                            ):
                                delta = (
                                    current_alive_counter
                                    - previous_alive_counter
                                ) & 0xFF

                                expected_alive_counter = (
                                    previous_alive_counter
                                    + 1
                                ) & 0xFF

                                if delta == 0:
                                    alive_valid = False

                                    print(
                                        "[PEDAL] "
                                        "ALIVE COUNTER DUPLICATE: "
                                        f"expected="
                                        f"{expected_alive_counter}, "
                                        f"received="
                                        f"{current_alive_counter}"
                                    )

                                elif 2 <= delta < 128:
                                    print(
                                        "[PEDAL] "
                                        "ALIVE COUNTER SKIP: "
                                        f"expected="
                                        f"{expected_alive_counter}, "
                                        f"received="
                                        f"{current_alive_counter}, "
                                        f"missed="
                                        f"{delta - 1}"
                                    )

                                elif delta >= 128:
                                    alive_valid = False

                                    print(
                                        "[PEDAL] "
                                        "ALIVE COUNTER BACKWARD: "
                                        f"expected="
                                        f"{expected_alive_counter}, "
                                        f"received="
                                        f"{current_alive_counter}"
                                    )

                            if alive_valid:
                                previous_alive_counter = (
                                    current_alive_counter
                                )

                        if alive_valid:
                            state.update_pedal(
                                pedal
                            )
                            pedal_diagnostic_accepted = True

                            logger.log(
                                state
                            )

                            last_pedal_time = (
                                time.monotonic()
                            )

                            pedal_timeout_reported = (
                                False
                            )

                            sensor_errors_text = ",".join(
                                pedal["sensor_errors"]
                            )

                            print(
                                f"[PEDAL] "
                                f"ACC={pedal['accelerator']}% "
                                f"BRAKE={pedal['brake']}% "
                                f"ACC_RATE={pedal['accelerator_rate']:.1f}%/s "
                                f"BRAKE_RATE={pedal['brake_rate']:.1f}%/s "
                                f"STATUS=0x{pedal['sensor_status']:02X} "
                                f"ERRORS={sensor_errors_text} "
                                f"ALIVE={pedal['alive_counter']}"
                            )

                    except ValueError as error:
                        print(
                            "[PEDAL] "
                            f"Invalid message: {error}"
                        )

                # ---------------------------------------------
                # HAPTIC_STATUS
                # ---------------------------------------------
                elif (
                    msg.arbitration_id
                    == HAPTIC_STATUS_ID
                ):
                    try:
                        haptic = decode_haptic_status(
                            msg.data
                        )

                        state.update_haptic(
                            haptic
                        )

                        haptic_timeout_reported = (
                            False
                        )

                        print(
                            f"[HAPTIC] "
                            f"RISK="
                            f"{haptic['risk_level']} "
                            f"VIB="
                            f"{haptic['vibration_status']} "
                            f"INTENSITY="
                            f"{haptic['intensity']}% "
                            f"FREQ="
                            f"{haptic['frequency']}Hz "
                            f"FAULT="
                            f"{haptic['fault_code']} "
                            f"CMD_TIMEOUT="
                            f"{haptic['command_timeout']} "
                            f"ECU_STATUS="
                            f"{haptic['ecu_status']} "
                            f"ALIVE="
                            f"{haptic['alive_counter']}"
                        )

                    except ValueError as error:
                        print(
                            "[HAPTIC] "
                            f"Invalid message: {error}"
                        )

                # ---------------------------------------------
                # MOTOR_STATUS
                # ---------------------------------------------
                elif (
                    msg.arbitration_id
                    == MOTOR_STATUS_ID
                ):
                    try:
                        motor = decode_motor_status(
                            msg.data
                        )

                        motor_status = motor
                        motor_last_rx_time = (
                            time.monotonic()
                        )
                        print(
                            f"[MOTOR] "
                            f"STATE={motor['motor_state_name']} "
                            f"PWM={motor['actual_pwm']}% "
                            f"LIMIT={motor['applied_output_limit']}% "
                            f"FAULT=0x{motor['fault_flags']:02X} "
                            f"APPLIED_CMD={motor['applied_command_counter']} "
                            f"ALIVE={motor['alive_counter']}"
                        )

                    except ValueError as error:
                        print(
                            "[MOTOR] "
                            f"Invalid message: {error}"
                        )

            # =================================================
            # Pedal timeout
            # =================================================
            if (
                time.monotonic()
                - last_pedal_time
                >= 0.1
            ):
                state.pedal_connected = False

                if not pedal_timeout_reported:
                    print(
                        "[PEDAL] "
                        "COMMUNICATION TIMEOUT"
                    )

                    recovery_alive_candidate = None

                    pedal_timeout_reported = (
                        True
                    )

            # =================================================
            # Haptic ECU freshness check
            # =================================================
            was_haptic_connected = (
                state.haptic_connected
            )

            state.check_haptic_freshness(
                timeout=0.1
            )

            if (
                was_haptic_connected
                and not state.haptic_connected
            ):
                if not haptic_timeout_reported:
                    print(
                        "[HAPTIC] "
                        "COMMUNICATION TIMEOUT"
                    )

                    haptic_timeout_reported = (
                        True
                    )

            # =================================================
            # BNS calculation
            # =================================================
            # Select environment inputs for BNS
            if INPUT_MODE == INPUT_MODE_CARLA:
                bns_distance_cm = state.carla_distance_cm
                bns_distance_valid = state.carla_distance_valid
                bns_vehicle_speed = state.carla_vehicle_speed
                bns_ttc = state.carla_ttc
            else:
                bns_distance_cm = state.distance_cm
                bns_distance_valid = state.distance_valid
                bns_vehicle_speed = state.vehicle_speed
                bns_ttc = state.ttc

            state.bns = bns_calculator.calculate(
                distance_cm=bns_distance_cm,
                distance_valid=bns_distance_valid,
                vehicle_speed=bns_vehicle_speed,
                ttc=bns_ttc,
                closing_speed_mps=(state.carla_closing_speed_mps if INPUT_MODE == INPUT_MODE_CARLA else None),
                environment_connected=(state.carla_connected if INPUT_MODE == INPUT_MODE_CARLA else bns_distance_valid),
            )

            # =================================================
            # PMS calculation
            # =================================================
            state.pms = (
                pms_calculator.calculate(
                    accelerator=(
                        state.accelerator
                    ),
                    accelerator_rate=(
                        state.accelerator_rate
                    ),
                    pedal_connected=(
                        state.pedal_connected
                    ),
                    sensor_status=(
                        state.pedal_sensor_status
                    ),
                    brake=state.brake,
                    brake_rate=state.brake_rate,  # signed %/s (decoder int16 LE * 0.1)
                )
            )

            # =================================================
            # Integrated risk evaluation
            # =================================================
            (
                state.total_risk,
                state.risk_level,
            ) = risk_manager.evaluate(
                bns=state.bns,
                bns_available=(
                    bns_calculator.available
                ),
                pms=state.pms,
                pms_available=(
                    pms_calculator.available
                ),
                level1_conditions=(bns_calculator.distance_l1, bns_calculator.ttc_l1, pms_calculator.pedal_l1),
                level2_conditions=(bns_calculator.distance_l2, bns_calculator.ttc_l2, pms_calculator.pedal_l2),
                level3_conditions=(bns_calculator.distance_l3, bns_calculator.ttc_l3, pms_calculator.pedal_l3),
            )

            risk_log_time = time.monotonic()
            if risk_log_time - last_risk_log_time >= RISK_LOG_INTERVAL:
                last_risk_log_time = risk_log_time
                print(
                    f"[RISK] BNS={state.bns:.2f} PMS={state.pms:.2f} "
                    f"TOTAL={state.total_risk:.2f} LEVEL=L{state.risk_level} "
                    f"DIST={bns_distance_cm / 100.0 if bns_distance_cm is not None else None}m "
                    f"TTC={bns_ttc}s ACC={state.accelerator}% ACC_RATE={state.accelerator_rate:.1f}%/s "
                    f"CANDIDATE=L{risk_manager.candidate_level}", flush=True,
                )

            # =================================================
            # Fail-safe evaluation
            # =================================================
            failsafe.evaluate(
                state,
                input_mode=INPUT_MODE,
            )

            if pedal_diagnostic_sample is not None:
                pedal_diagnostics.report(
                    pedal_diagnostic_sample,
                    pedal_diagnostic_accepted,
                    state,
                    failsafe,
                )

            if (
                failsafe.active
                != previous_failsafe_active
                or failsafe.reason
                != previous_failsafe_reason
            ):
                if failsafe.active:
                    print(
                        "[FAIL-SAFE] "
                        f"ACTIVE "
                        f"REASON="
                        f"{failsafe.reason}"
                    )

                else:
                    print(
                        "[FAIL-SAFE] "
                        "CLEARED"
                    )

                previous_failsafe_active = (
                    failsafe.active
                )

                previous_failsafe_reason = (
                    failsafe.reason
                )

            # =================================================
            # Haptic / Motor command transmission
            # =================================================
            current_time = time.monotonic()

            if (
                current_time
                - last_command_tx_time
                >= 0.01
            ):
                last_command_tx_time = (
                    current_time
                )

                commands = command_mapper.map(
                    risk_level=(
                        state.risk_level
                    ),
                    risk_available=(
                        risk_manager.available
                    ),
                    failsafe_active=(
                        failsafe.active
                    ),
                )

                # -------------------------------------------------
                # Motor ECU communication / E-STOP recovery
                # -------------------------------------------------
                motor_status_fresh = (
                    motor_status is not None
                    and motor_last_rx_time is not None
                    and 0 <= current_time - motor_last_rx_time <= 0.1
                )

                if not motor_status_fresh:
                    motor_communication_fault = True
                    motor_recovery_required = True

                    if not motor_timeout_reported:
                        print("[MOTOR] STATUS TIMEOUT - E-STOP REQUESTED")
                        motor_timeout_reported = True

                    commands["motor"] = {
                        "control_mode": 2,
                        "output_limit": 0,
                        "command_flags": 0x03,
                    }

                else:
                    motor_communication_fault = False
                    motor_timeout_reported = False

                    motor_estop_active = (
                        motor_status["fault_flags"] & 0x08
                    ) != 0

                    motor_other_fault = (
                        motor_status["fault_flags"] & 0x07
                    ) != 0

                    motor_safe_to_recover = (
                        not failsafe.active
                        and state.pedal_connected
                        and state.pedal_sensor_status == 0
                        and state.accelerator <= 3
                        and state.brake < 5
                        and not motor_other_fault
                    )

                    if failsafe.active:
                        motor_recovery_required = True

                    elif motor_estop_active:
                        motor_recovery_required = True

                        if motor_safe_to_recover:
                            commands["motor"] = {
                                "control_mode": 2,
                                "output_limit": 0,
                                "command_flags": 0x01,
                            }

                    elif motor_recovery_required:
                        if (
                            motor_safe_to_recover
                            and motor_status["motor_state"] == 0
                            and motor_status["actual_pwm"] == 0
                        ):
                            motor_recovery_required = False
                            print("[MOTOR] E-STOP RECOVERY CONFIRMED")

                        # Maintain STOP during recovery confirmation.
                        commands["motor"] = {
                            "control_mode": 2,
                            "output_limit": 0,
                            "command_flags": 0x01,
                        }

                if current_time - last_risk_debug_time >= 0.5:
                    last_risk_debug_time = current_time
                    print(
                        f"[RISKDBG] OBS={int(state.carla_obstacle_present if INPUT_MODE == INPUT_MODE_CARLA else bns_distance_valid)} "
                        f"DIST={bns_distance_cm / 100.0 if bns_distance_cm is not None else None} "
                        f"TTC={bns_ttc} CLOSING={state.carla_closing_speed_mps if INPUT_MODE == INPUT_MODE_CARLA else None} "
                        f"BNS={state.bns:.3f} PMS={state.pms:.3f} "
                        + " ".join(f"D{i}={int(getattr(bns_calculator, f'distance_l{i}'))}" for i in (1, 2, 3)) + " "
                        + " ".join(f"T{i}={int(getattr(bns_calculator, f'ttc_l{i}'))}" for i in (1, 2, 3)) + " "
                        + " ".join(f"P{i}={int(getattr(pms_calculator, f'pedal_l{i}'))}" for i in (1, 2, 3))
                        + f" CAND=L{risk_manager.candidate_level} LEVEL=L{state.risk_level} "
                        f"CMD_MODE={commands['motor']['control_mode']} LIMIT={commands['motor']['output_limit']}",
                        flush=True,
                    )

                # ---------------------------------------------
                # Haptic command
                # ---------------------------------------------
                sender.send_haptic_command(
                    risk_level=(
                        state.risk_level
                    ),
                    vibration_command=(
                        commands["haptic"][
                            "vibration_command"
                        ]
                    ),
                    intensity=(
                        commands["haptic"][
                            "intensity"
                        ]
                    ),
                    frequency=(
                        commands["haptic"][
                            "frequency"
                        ]
                    ),
                    alive_counter=(
                        haptic_alive_counter
                    ),
                )

                # ---------------------------------------------
                # Motor command
                # ---------------------------------------------
                sender.send_motor_command(
                    control_mode=(
                        commands["motor"][
                            "control_mode"
                        ]
                    ),
                    output_limit=(
                        commands["motor"][
                            "output_limit"
                        ]
                    ),
                    risk_level=(
                        state.risk_level
                    ),
                    command_flags=(
                        commands["motor"][
                            "command_flags"
                        ]
                    ),
                    alive_counter=(
                        motor_alive_counter
                    ),
                )

                # ---------------------------------------------
                # System logger
                # ---------------------------------------------
                system_logger.log(
                    state=state,
                    bns_available=(
                        bns_calculator.available
                    ),
                    pms_available=(
                        pms_calculator.available
                    ),
                    risk_available=(
                        risk_manager.available
                    ),
                    failsafe_active=(
                        failsafe.active
                    ),
                    failsafe_reason=(
                        failsafe.reason
                    ),
                    commands=commands,
                )

                carla_control_sender.send(
                    commands["motor"], state.risk_level, failsafe.active,
                    now=current_time,
                )

                # Snapshot final commands after Motor E-STOP/recovery overrides.
                # JSON encoding and UDP transmission run outside the control loop.
                hmi_sender.publish(
                    state,
                    input_mode=INPUT_MODE,
                    bns_available=bns_calculator.available,
                    pms_available=pms_calculator.available,
                    risk_available=risk_manager.available,
                    failsafe_active=failsafe.active,
                    failsafe_reason=failsafe.reason,
                    commands=commands,
                    motor_connected=motor_status_fresh,
                )

                haptic_alive_counter = (
                    haptic_alive_counter + 1
                ) % 256

                motor_alive_counter = (
                    motor_alive_counter + 1
                ) % 256

            # =================================================
            # State output
            # =================================================
            # -------------------------------------------------
            # CARLA state output
            # -------------------------------------------------
            if (
                INPUT_MODE == INPUT_MODE_CARLA
                and time.monotonic() - last_carla_log_time
                >= CARLA_LOG_INTERVAL
            ):
                last_carla_log_time = time.monotonic()

                print(
                    f"[CARLA STATE] "
                    f"CONNECTED={state.carla_connected} "
                    f"SPEED={state.carla_vehicle_speed}km/h "
                    f"DIST={state.carla_distance_cm}cm "
                    f"DIST_VALID={state.carla_distance_valid} "
                    f"CLOSING={state.carla_closing_speed_mps}m/s "
                    f"TTC={state.carla_ttc}s "
                    f"OBSTACLE_ID={state.carla_obstacle_id} "
                    f"BNS={state.bns:.1f} "
                    f"BNS_AVAILABLE={bns_calculator.available} "
                    f"FAILSAFE={failsafe.active} "
                    f"FAILSAFE_REASON={failsafe.reason}",
                    flush=True
                )

            if (
                INPUT_MODE == INPUT_MODE_HARDWARE
                and state.distance_valid
            ):
                print(
                    f"[STATE] "
                    f"ACC={state.accelerator}% "
                    f"ACC_RATE="
                    f"{state.accelerator_rate:.1f}%/s "
                    f"DIST="
                    f"{state.distance_cm:.1f}cm "
                    f"PEDAL_CONNECTED="
                    f"{state.pedal_connected} "
                    f"TOF_CONNECTED="
                    f"{state.tof_connected} "
                    f"DIST_VALID="
                    f"{state.distance_valid} "
                    f"BNS="
                    f"{state.bns:.1f} "
                    f"BNS_AVAILABLE="
                    f"{bns_calculator.available} "
                    f"PMS="
                    f"{state.pms:.1f} "
                    f"PMS_AVAILABLE="
                    f"{pms_calculator.available} "
                    f"TOTAL_RISK="
                    f"{state.total_risk:.1f} "
                    f"RISK_LEVEL="
                    f"{state.risk_level} "
                    f"RISK_AVAILABLE="
                    f"{risk_manager.available} "
                    f"HAPTIC_CONNECTED="
                    f"{state.haptic_connected} "
                    f"HAPTIC_AVAILABLE="
                    f"{state.haptic_available} "
                    f"HAPTIC_FAULT="
                    f"{state.haptic_fault_code} "
                    f"HAPTIC_CMD_TIMEOUT="
                    f"{state.haptic_command_timeout}"
                )

    except KeyboardInterrupt:
        print(
            "\n[Safety ECU] Stopped"
        )

    finally:
        carla_control_sender.close()
        hmi_sender.close()
        if carla_receiver is not None:
            carla_receiver.close()

        sender.close()
        receiver.close()


if __name__ == "__main__":
    main()
