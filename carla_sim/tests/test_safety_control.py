"""Test real receiver code without Windows/CARLA dependencies."""
import ast
import contextlib
import io
import json
from pathlib import Path
import socket
import textwrap
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from safety_ecu.carla.control_sender import CarlaControlSender
from safety_ecu.carla.receiver_v2 import CarlaReceiverV2
from safety_ecu.control.command_mapper import CommandMapper

SOURCE = Path(__file__).resolve().parents[1] / 'carla_pedal_control_v17_safety_json_5006.py'
TREE = ast.parse(SOURCE.read_text())
NS = {'socket': socket, 'json': json, 'time': time}
exec(compile(ast.Module(body=[n for n in TREE.body if isinstance(n, ast.ClassDef)
     and n.name == 'SafetyControlReceiver'], type_ignores=[]), str(SOURCE), 'exec'), NS)

RADAR_CLASS = next(
    node for node in TREE.body
    if isinstance(node, ast.ClassDef)
    and node.name == "FrontDistanceSensor"
)

exec(
    compile(
        ast.Module(body=[RADAR_CLASS], type_ignores=[]),
        str(SOURCE),
        "exec"
    ),
    NS
)

import math

TELEMETRY_FUNCTION = next(
    node for node in TREE.body
    if isinstance(node, ast.FunctionDef)
    and node.name == "build_safety_telemetry"
)

exec(
    compile(
        ast.Module(body=[TELEMETRY_FUNCTION], type_ignores=[]),
        str(SOURCE),
        "exec",
    ),
    NS,
)

EXTRA_OBSTACLE_FUNCTION = next(
    node for node in TREE.body
    if isinstance(node, ast.FunctionDef)
    and node.name == "find_extra_obstacle_for_safety"
)
exec(
    compile(
        ast.Module(body=[EXTRA_OBSTACLE_FUNCTION], type_ignores=[]),
        str(SOURCE),
        "exec",
    ),
    NS,
)

CREEP_NODES = [n for n in TREE.body if (
    isinstance(n, ast.Assign) and any(
        isinstance(t, ast.Name) and t.id.startswith('CREEP_') for t in n.targets
    )
) or (isinstance(n, ast.FunctionDef) and n.name in ('clamp', 'calculate_creep_throttle'))]
exec(compile(ast.Module(body=CREEP_NODES, type_ignores=[]), str(SOURCE), 'exec'), NS)


class SafetyControlTests(unittest.TestCase):
    def test_extra_obstacle_accepts_walker_and_static_only(self):
        detector = Mock(return_value={"kind": "walker", "obstacle_id": 42})
        world = object()
        ego = object()
        carla_module = object()
        with patch.dict(NS, {
            "carla": carla_module,
            "SAFETY_FRONT_MAX_DISTANCE_M": 50.0,
            "find_path_obstacle_for_safety": detector,
        }):
            self.assertEqual(
                NS["find_extra_obstacle_for_safety"](world, ego, "R"),
                {"kind": "walker", "obstacle_id": 42},
            )
            detector.assert_called_once_with(
                carla_module, world, ego, max_distance=50.0, reverse=True,
                kinds=("walker", "static"),
            )
            detector.return_value = {"kind": "static", "obstacle_id": 43}
            self.assertEqual(
                NS["find_extra_obstacle_for_safety"](world, ego, "D")["obstacle_id"],
                43,
            )
            detector.return_value = {"kind": "vehicle", "obstacle_id": 44}
            self.assertIsNone(NS["find_extra_obstacle_for_safety"](world, ego, "D"))
            detector.reset_mock()
            self.assertIsNone(NS["find_extra_obstacle_for_safety"](world, ego, "P"))
            detector.assert_not_called()

    def test_radar_safety_distance_freshness(self):
        sensor = NS["FrontDistanceSensor"].__new__(
            NS["FrontDistanceSensor"]
        )

        sensor.distance = 5.0
        sensor.last_valid_time = 9.9
        sensor.collision_zero_until = 0.0

        # Fresh Radar measurement
        with patch.object(time, "monotonic", return_value=10.0):
            self.assertEqual(sensor.get_safety_distance(), 5.0)

        # Stale Radar measurement
        with patch.object(time, "monotonic", return_value=10.2):
            self.assertIsNone(sensor.get_safety_distance())

        # Recent collision takes priority
        sensor.collision_zero_until = 10.5

        with patch.object(time, "monotonic", return_value=10.3):
            self.assertEqual(sensor.get_safety_distance(), 0.0)

        # No valid measurement
        sensor.collision_zero_until = 0.0
        sensor.last_valid_time = 0.0

        with patch.object(time, "monotonic", return_value=10.4):
            self.assertIsNone(sensor.get_safety_distance())

    def test_radar_safety_corridor_filter(self):
        import math
        import time
        import weakref

        from types import SimpleNamespace
        from unittest.mock import patch

        sensor_class = NS["FrontDistanceSensor"]

        sensor = sensor_class.__new__(sensor_class)
        sensor.distance = None
        sensor.distance_history = []
        sensor.last_valid_time = 0.0
        sensor.collision_zero_until = 0.0
        sensor.safety_corridor_distance = None
        sensor.last_safety_valid_time = 0.0

        def detection(depth, azimuth_deg=0.0, altitude_deg=0.0):
            return SimpleNamespace(
                depth=depth,
                azimuth=math.radians(azimuth_deg),
                altitude=math.radians(altitude_deg),
            )

        def median_value(values):
            ordered = sorted(values)
            middle = len(ordered) // 2
            if len(ordered) % 2:
                return ordered[middle]
            return (ordered[middle - 1] + ordered[middle]) / 2

        test_globals = {
            "math": math,
            "median": median_value,
            "clamp": lambda x, low, high: max(low, min(high, x)),
            "FRONT_DISTANCE_RANGE": 50.0,
            "SAFETY_RADAR_AZIMUTH_DEG": 4.0,
            "SAFETY_RADAR_ALTITUDE_DEG": 3.0,
            "SAFETY_RADAR_MIN_POINTS": 3,
            "SAFETY_RADAR_FRESHNESS_S": 0.2,
            "DISTANCE_CLOSEST_POINT_COUNT": 5,
            "DISTANCE_HISTORY_SIZE": 7,
            "DISTANCE_DISPLAY_STEP": 0.05,
            "DISTANCE_DEADBAND": 0.08,
            "DISTANCE_NO_SIGN_DELAY": 0.25,
        }

        weak_sensor = weakref.ref(sensor)

        with patch.dict(NS, test_globals):

            # 1. 정면에 유효한 반사점 3개
            with patch.object(time, "monotonic", return_value=10.0):
                sensor_class._on_radar(
                    weak_sensor,
                    [
                        detection(5.0),
                        detection(5.1, 1.0),
                        detection(5.2, -1.0),
                    ],
                )
                self.assertAlmostEqual(
                    sensor.get_safety_corridor_distance(), 5.1
                )

            # 2. 좌우 범위를 벗어난 반사점
            with patch.object(time, "monotonic", return_value=10.1):
                sensor_class._on_radar(
                    weak_sensor,
                    [
                        detection(4.0, 8.0),
                        detection(4.1, 9.0),
                        detection(4.2, -8.0),
                    ],
                )
                self.assertIsNone(
                    sensor.get_safety_corridor_distance()
                )

            # 3. 우상하 범위를 벗어난 반사점
            with patch.object(time, "monotonic", return_value=10.2):
                sensor_class._on_radar(
                    weak_sensor,
                    [
                        detection(4.0, 0.0, 6.0),
                        detection(4.1, 0.0, 7.0),
                        detection(4.2, 0.0, -6.0),
                    ],
                )
                self.assertIsNone(
                    sensor.get_safety_corridor_distance()
                )

            # 4. 유효 반사점이 2개뿐인 경우
            with patch.object(time, "monotonic", return_value=10.3):
                sensor_class._on_radar(
                    weak_sensor,
                    [detection(3.0), detection(3.1)],
                )
                self.assertIsNone(
                    sensor.get_safety_corridor_distance()
                )

            # 5. 정상 측정 후 오래된 값 차단
            with patch.object(time, "monotonic", return_value=10.4):
                sensor_class._on_radar(
                    weak_sensor,
                    [
                        detection(6.0),
                        detection(6.1),
                        detection(6.2),
                    ],
                )
                self.assertAlmostEqual(
                    sensor.get_safety_corridor_distance(), 6.1
                )

            with patch.object(time, "monotonic", return_value=10.7):
                self.assertIsNone(
                    sensor.get_safety_corridor_distance()
                )

            # 6. 새 프로엠에 반사점이 없는 경우
            with patch.object(time, "monotonic", return_value=10.8):
                sensor_class._on_radar(weak_sensor, [])
                self.assertIsNone(
                    sensor.get_safety_corridor_distance()
                )

    def test_radar_telemetry_integration(self):
        build = NS["build_safety_telemetry"]

        ego = SimpleNamespace(
            get_transform=lambda: SimpleNamespace(
                get_forward_vector=lambda: SimpleNamespace(
                    x=1.0, y=0.0, z=0.0
                )
            )
        )

        world = object()
        actor = SimpleNamespace(id=1234)

        front_vehicle = {
            "actor": actor,
            "distance_m": 12.0,
            "closing_speed_mps": 4.0,
            "ttc_s": 3.0,
        }

        rear_vehicle = {
            "actor": actor,
            "distance_m": 8.0,
            "closing_speed_mps": 2.0,
            "ttc_s": 4.0,
        }

        def run_case(
            gear="D",
            radar_distance=None,
            front=None,
            rear=None,
            extra=None,
            speed_mps=10.0,
            virtual_active=False,
            virtual_distance=None,
            materialized=False,
        ):
            sensor = SimpleNamespace(
                get_safety_corridor_distance=lambda: radar_distance
            )

            globals_for_test = {
                "math": math,
                "FRONT_DISTANCE_RANGE": 50.0,
                "RADAR_OBSTACLE_ID": 900002,
                "NIGHT_SCENARIO_OBSTACLE_ID": 900001,
                "get_speed_kmh": lambda vehicle: abs(speed_mps) * 3.6,
                "get_forward_speed_mps": (
                    lambda vehicle, direction: speed_mps
                ),
                "find_front_vehicle_for_safety": (
                    lambda world, vehicle: front
                ),
                "find_rear_vehicle_for_safety": (
                    lambda world, vehicle: rear
                ),
                "find_extra_obstacle_for_safety": (
                    lambda world, vehicle, gear: extra
                ),
            }

            with patch.dict(NS, globals_for_test):
                return build(
                    world,
                    ego,
                    7,
                    virtual_active,
                    virtual_distance,
                    virtual_obstacle_materialized=materialized,
                    gear=gear,
                    front_distance_sensor=sensor,
                )

        # 1. 자동차가 없으면 Radar 장애물 사용
        result = run_case(radar_distance=5.0)
        self.assertEqual(result["obstacle_id"], 900002)
        self.assertEqual(result["distance_m"], 5.0)
        self.assertEqual(result["closing_speed_mps"], 10.0)
        self.assertEqual(result["ttc_s"], 0.5)

        # 2. The closest physical obstacle wins across sources.
        result = run_case(radar_distance=20.0, front=front_vehicle)
        self.assertEqual(result["obstacle_id"], 1234)
        self.assertEqual(result["distance_m"], 12.0)
        self.assertEqual(result["ttc_s"], 3.0)

        walker = {"obstacle_id": 42, "distance_m": 4.0,
                  "closing_speed_mps": 2.0, "ttc_s": 2.0}
        result = run_case(radar_distance=5.0, extra=walker)
        self.assertEqual(result["obstacle_id"], 42)
        self.assertEqual(result["ttc_s"], 2.0)

        result = run_case(radar_distance=5.0, front=front_vehicle, extra=walker)
        self.assertEqual(result["obstacle_id"], 42)

        distant_vehicle = dict(front_vehicle, distance_m=30.0, ttc_s=7.5)
        pedestrian = dict(walker, distance_m=5.0, ttc_s=2.5)
        result = run_case(radar_distance=3.0, front=distant_vehicle, extra=pedestrian)
        self.assertEqual(result["obstacle_id"], 900002)
        self.assertEqual(result["distance_m"], 3.0)
        self.assertAlmostEqual(result["ttc_s"], 0.3)
        receiver = CarlaReceiverV2()
        receiver.sock = Mock()
        receiver.sock.recvfrom.return_value = (
            json.dumps(result).encode("utf-8"), ("192.168.0.57", 5006)
        )
        self.assertEqual((receiver.host, receiver.port), ("192.168.0.37", 5006))
        self.assertTrue(receiver.receive())
        self.assertEqual(receiver.obstacle_id, 900002)
        self.assertEqual(receiver.distance_m, 3.0)

        # 3. 자동차와 Radar 모두 없을 때
        result = run_case()
        for key in (
            "obstacle_id",
            "distance_m",
            "closing_speed_mps",
            "ttc_s",
        ):
            self.assertIsNone(result[key])

        # 4. 후진에서는 후방 자동차만 선택
        result = run_case(
            gear="R",
            radar_distance=5.0,
            rear=rear_vehicle,
        )
        self.assertEqual(result["obstacle_id"], 1234)
        self.assertEqual(result["distance_m"], 8.0)

        rear_static = {"obstacle_id": 800123, "distance_m": 4.0,
                       "closing_speed_mps": 2.0, "ttc_s": 2.0}
        result = run_case(gear="R", radar_distance=3.0, rear=rear_vehicle,
                          extra=rear_static)
        self.assertEqual(result["obstacle_id"], 800123)

        # 5. 주차 상태에서는 장애물 선택 안 함
        result = run_case(gear="P", radar_distance=5.0)
        self.assertIsNone(result["obstacle_id"])

        # 6. O키 가상 장애물 시나리오 우선
        result = run_case(
            radar_distance=5.0,
            front=front_vehicle,
            virtual_active=True,
            virtual_distance=30.0,
        )
        self.assertEqual(result["obstacle_id"], 900001)
        self.assertEqual(result["distance_m"], 30.0)
        self.assertEqual(result["ttc_s"], 3.0)

        # 7. 정지 상태에서는 TTC 없음
        result = run_case(radar_distance=5.0, speed_mps=0.0)
        self.assertEqual(result["closing_speed_mps"], 0.0)
        self.assertIsNone(result["ttc_s"])

        # 8. 유효하지 않은 Radar 거리 제외
        for bad_distance in (
            float("nan"),
            float("inf"),
            -1.0,
            51.0,
        ):
            with self.subTest(radar_distance=bad_distance):
                result = run_case(radar_distance=bad_distance)
                self.assertIsNone(result["obstacle_id"])

        # 9. 가상 장애물이 실제 Actor로 전환된 뒤
        result = run_case(
            radar_distance=20.0,
            front=front_vehicle,
            virtual_active=True,
            virtual_distance=30.0,
            materialized=True,
        )
        self.assertEqual(result["obstacle_id"], 1234)



    def setUp(self):
        self.sock = Mock()
        self.sock.recvfrom.side_effect = BlockingIOError

        with patch.object(socket, 'socket', return_value=self.sock):
            self.receiver = NS['SafetyControlReceiver']()

        self.output = io.StringIO()
        self.test_sequence = 0
        self.test_session_id = "a" * 32
        self.receiver.allowed_sender_ip = "192.168.0.37"

    def test_old_normal_cannot_override_stop(self):
        session = "a" * 32

        def send_packet(sequence, mode, now):
            packet = {
                "type": "safety_control",
                "session_id": session,
                "sequence": sequence,
                "mode": mode,
                "limit_percent": 0 if mode == 2 else 100,
                "risk_level": 3 if mode == 2 else 0,
                "failsafe": False,
            }

            self.sock.recvfrom.side_effect = [
                (
                    json.dumps(packet).encode(),
                    ("192.168.0.37", 5008)
                ),
                BlockingIOError
            ]

            self.receiver.receive(now)

        # Accept STOP sequence 100
        send_packet(100, 2, 1.0)

        self.assertEqual(self.receiver.mode, 2)
        self.assertEqual(self.receiver.last_sequence, 100)

        # Reject old NORMAL sequence 99
        send_packet(99, 0, 1.1)

        self.assertEqual(self.receiver.mode, 2)
        self.assertEqual(self.receiver.last_sequence, 100)
        self.assertEqual(self.receiver.last_rx, 1.0)

        # Reject duplicate sequence 100
        send_packet(100, 0, 1.2)

        self.assertEqual(self.receiver.mode, 2)
        self.assertEqual(self.receiver.last_rx, 1.0)

        # Accept genuinely newer NORMAL sequence 101
        send_packet(101, 0, 1.3)

        self.assertEqual(self.receiver.mode, 0)
        self.assertEqual(self.receiver.last_sequence, 101)
        self.assertEqual(self.receiver.last_rx, 1.3)

    def apply_packet(self, mode, limit, throttle, risk=0, now=1., failsafe=False):
        self.test_sequence += 1

        packet = dict(
            type='safety_control',
            session_id=self.test_session_id,
            sequence=self.test_sequence,
            mode=mode,
            limit_percent=limit,
            risk_level=risk,
            failsafe=failsafe
        )

        self.sock.recvfrom.side_effect = [
            (json.dumps(packet).encode(), ('192.168.0.37', 5008)),
            BlockingIOError
        ]

        control = SimpleNamespace(throttle=throttle, brake=.37, steer=-.42)
        with contextlib.redirect_stdout(self.output):
            self.receiver.apply(control, now)
        self.assertEqual(control.brake, 1. if failsafe or mode == 2 else .37)
        self.assertEqual(control.steer, -.42)
        return control.throttle

    def test_creep_inputs_and_taper(self):
        creep = NS['calculate_creep_throttle']
        for speed, expected in ((0, .18), (3, .126), (5, .09), (8, .036),
                                (10, 0.), (15, 0.)):
            with self.subTest(speed=speed):
                self.assertAlmostEqual(creep('D', 0., 0., speed), expected)
        self.assertEqual(creep('D', 0., .03, 0), 0.)
        self.assertEqual(creep('D', .03, 0., 0), 0.)
        self.assertEqual(creep('R', 0., 0., 0), 0.)
        self.assertEqual(creep('P', 0., 0., 0), 0.)
        with patch.dict(NS, CREEP_ENABLED=False):
            self.assertEqual(creep('D', 0., 0., 0), 0.)


    def test_reverse_and_park_safety_states(self):
        for gear in ("R", "P"):
            for mode, failsafe, last_rx in (
                (0, False, 1.),
                (1, False, 1.),
                (2, False, 1.),
                (0, True, 1.),
                (2, True, None),
                (2, False, 0.),
            ):
                with self.subTest(
                    gear=gear,
                    mode=mode,
                    failsafe=failsafe,
                    last_rx=last_rx,
                ):
                    self.receiver.mode = mode
                    self.receiver.limit_percent = 30
                    self.receiver.failsafe = failsafe
                    self.receiver.last_rx = last_rx
                    self.receiver.last_log = None

                    control = SimpleNamespace(
                        throttle=.7,
                        brake=.12,
                        steer=-.4,
                        reverse=(gear == "R"),
                        hand_brake=(gear == "P"),
                    )

                    self.sock.recvfrom.side_effect = BlockingIOError

                    with contextlib.redirect_stdout(self.output):
                        self.receiver.apply(
                            control,
                            1.,
                            gear=gear,
                        )

                    stale = last_rx is None or last_rx == 0.

                    if stale or failsafe or mode == 2:
                        expected_throttle = 0.
                        expected_brake = 1.
                    elif gear == "P":
                        expected_throttle = 0.
                        expected_brake = 1.
                    elif mode == 1:
                        expected_throttle = .3
                        expected_brake = .12
                    else:
                        expected_throttle = .7
                        expected_brake = .12

                    self.assertAlmostEqual(
                        control.throttle,
                        expected_throttle,
                    )
                    self.assertEqual(control.brake, expected_brake)
                    self.assertEqual(control.steer, -.4)
                    self.assertEqual(control.reverse, gear == "R")
                    self.assertEqual(control.hand_brake, gear == "P")

        logs = self.output.getvalue()
        self.assertIn("GEAR=R", logs)
        self.assertIn("GEAR=P", logs)
        self.assertIn("SAFETY_ACTIVE_FOR_REVERSE=1", logs)
        self.assertIn("MODE=LIMIT", logs)
        self.assertIn("MODE=STOP", logs)
        self.assertIn("MODE=PARK", logs)
        self.assertIn("MODE=FAILSAFE_STOP", logs)
        self.assertIn("STALE -> STOP", logs)


    def test_main_driver_priority_and_gear_transition(self):
        source = SOURCE.read_text()
        start = source.index('            keys = pygame.key.get_pressed()')
        end = source.index('            # REALISTIC LANE FOLLOWING ASSIST', start)
        code = compile(textwrap.dedent(source[start:end]), str(SOURCE), 'exec')
        for gear, accel, brake, speed, pending, expected_throttle in (
                ('D', 0., 0., 0., False, .18),
                ('D', 0., 0., 10., False, 0.),
                ('D', .65, 0., 0., False, .65),
                ('D', 0., .03, 0., False, 0.),
                ('R', .65, 0., 0., False, .65),
                ('R', 0., 0., 0., False, 0.),
                ('D', 0., 0., 5., True, 0.),
                ('D', 0., 0., 0., True, 0.)):
            with self.subTest(gear=gear, accel=accel, brake=brake, speed=speed, pending=pending):
                ns = dict(NS, pygame=SimpleNamespace(key=SimpleNamespace(get_pressed=lambda: [])),
                          get_speed_kmh=lambda vehicle: speed,
                          get_speed_steering_gain=lambda speed: .82,
                          STEERING_GAIN_LOW_SPEED=.82, BRAKE_INPUT_GAIN=1.2,
                          GEAR_CHANGE_SPEED_KMH=1., control_mode='KEYBOARD',
                          keyboard=SimpleNamespace(update=lambda *args: (accel, brake, 0.),
                                                   reset=lambda: None),
                          milliseconds=16, vehicle=object(), applied_gear=gear,
                          requested_gear='R' if pending else gear,
                          gear_change_pending=pending, smooth_throttle=0., smooth_brake=0.)
                with contextlib.redirect_stdout(self.output):
                    exec(code, ns)
                self.assertAlmostEqual(ns['target_throttle'], expected_throttle)
                if brake > .02 or (pending and speed > 1.):
                    self.assertEqual(ns['creep_throttle'], 0.)
                    self.assertGreater(ns['target_brake'], 0.)

    def test_safety_overrides_creep(self):
        creep = NS['calculate_creep_throttle']('D', 0., 0., 0.)
        for mode, failsafe, fresh, expected in (
                (0, False, True, (.18, 0.)), (1, False, True, (.18, 0.)),
                (2, False, True, (0., 1.)), (0, True, True, (0., 1.)),
                (0, False, False, (0., 1.))):
            self.receiver.mode = mode
            self.receiver.limit_percent = 30
            self.receiver.failsafe = failsafe
            self.receiver.last_rx = 1. if fresh else None
            self.receiver.last_log = None
            control = SimpleNamespace(throttle=creep, brake=0.)
            with contextlib.redirect_stdout(self.output):
                self.receiver.apply(control, 1., gear='D', speed_kmh=0.,
                                    creep_throttle=creep, driver_throttle=0.)
            self.assertEqual((control.throttle, control.brake), expected)
        for field in ('GEAR=D', 'SPEED_KMH=0.0', 'CREEP=1',
                      'CREEP_THROTTLE=0.180', 'RAW_THROTTLE=0.00'):
            self.assertIn(field, self.output.getvalue())

    def test_requested_limits(self):
        for mode, limit, raw, expected in [(0, 100, 1., 1.), (1, 30, 1., .3),
                (1, 30, .2, .2), (1, 50, .8, .5)]:
            with self.subTest(mode=mode, limit=limit, raw=raw):
                self.assertEqual(self.apply_packet(mode, limit, raw), expected)

    def test_mapper_levels(self):
        for risk in range(4):
            motor = CommandMapper().map(risk, True)['motor']
            self.assertEqual(self.apply_packet(motor['control_mode'], motor['output_limit'],
                                             1., risk), .3 if risk == 2 else 0. if risk == 3 else 1.)

    def test_stop_without_failsafe(self):
        self.assertEqual(self.apply_packet(2, 0, .8, risk=3), 0.)
        self.assertIn('MODE=STOP', self.output.getvalue())
        self.assertIn('FAILSAFE=0', self.output.getvalue())

    def test_failsafe_overrides_all_modes(self):
        for mode in (0, 1, 2):
            self.assertEqual(self.apply_packet(mode, 100, 1., failsafe=True), 0.)

    def test_stale_stops_at_timeout_boundary(self):
        for mode in (0, 1, 2):
            self.apply_packet(mode, 30, 1.)
            self.sock.recvfrom.side_effect = BlockingIOError
            for now in (1.5, 1.6):
                control = SimpleNamespace(throttle=.8, brake=.37, steer=-.42)
                with contextlib.redirect_stdout(self.output):
                    self.receiver.apply(control, now)
                self.assertEqual((control.throttle, control.brake, control.steer), (0., 1., -.42))
        self.assertIn('FRESH=0', self.output.getvalue())
        self.assertIn('[CARLA SAFETY] STALE -> STOP', self.output.getvalue())

    def test_startup_stops_and_fresh_command_recovers(self):
        control = SimpleNamespace(throttle=1., brake=0., steer=.61)
        with contextlib.redirect_stdout(self.output):
            self.receiver.apply(control, .1)
        self.assertEqual((control.throttle, control.brake, control.steer), (0., 1., .61))
        self.assertIn('WAITING FOR FIRST safety_control PACKET -> STOP', self.output.getvalue())
        self.assertEqual(self.apply_packet(0, 100, 1.), 1.)
        self.sock.recvfrom.side_effect = BlockingIOError
        with contextlib.redirect_stdout(self.output):
            self.receiver.apply(control, 1.5)
        self.assertEqual(control.brake, 1.)
        self.assertEqual(self.apply_packet(0, 100, .7, now=1.6), .7)

    def test_latest_valid_packet_and_log_values(self):
        packets = [
            dict(
                type='safety_control',
                session_id="a" * 32,
                sequence=i,
                mode=mode,
                limit_percent=limit,
                risk_level=risk,
                failsafe=False
            )
            for i, mode, limit, risk in (
                (1, 2, 0, 3),
                (2, 1, 30, 2)
            )
        ]
        self.sock.recvfrom.side_effect = [(json.dumps(p).encode(), ('192.168.0.37', 5008))
                                        for p in packets] + [BlockingIOError]
        control = SimpleNamespace(throttle=1., brake=.2, steer=.4)
        with contextlib.redirect_stdout(self.output):
            self.receiver.apply(control, 1.)
        self.assertEqual((control.throttle, control.brake, control.steer), (.3, .2, .4))
        for field in ('RISK=L2', 'MODE=LIMIT', 'LIMIT=30', 'FAILSAFE=0', 'FRESH=1',
                      'RAW_THROTTLE=1.00', 'RAW_BRAKE=0.20',
                      'APPLIED_THROTTLE=0.300', 'APPLIED_BRAKE=0.20'):
            self.assertIn(field, self.output.getvalue())
        self.sock.recvfrom.side_effect = BlockingIOError
        with contextlib.redirect_stdout(self.output):
            self.receiver.apply(control, 1.1)
        self.assertEqual(self.output.getvalue().count('RISK=L2'), 1)

    def test_bad_packets_do_not_refresh(self):
        for data in [b'{', b'[]', b'{}', b'\xff', json.dumps(dict(type='safety_control',
                sequence=0, mode=1, limit_percent=float('nan'), risk_level=3, failsafe=False)).encode()]:
            self.sock.recvfrom.side_effect = [(data, ('192.168.0.37', 5008)), BlockingIOError]
            self.receiver.receive(1.)
            self.assertIsNone(self.receiver.last_rx)

    def test_socket_and_cleanup(self):
        self.sock.bind.assert_called_once_with(('0.0.0.0', 5008))
        self.sock.setblocking.assert_called_once_with(False)
        self.receiver.close()
        self.sock.close.assert_called_once()

    def test_sender_rate_schema_and_final_command(self):
        tx = Mock()
        with patch.object(socket, 'socket', return_value=tx):
            sender = CarlaControlSender()
        motor = dict(control_mode=1, output_limit=30, command_flags=1)
        sender.send(motor, 3, False, now=1.)
        sender.send(motor, 3, False, now=1.01)
        self.assertEqual(tx.sendto.call_count, 1)
        data, dest = tx.sendto.call_args.args
        self.assertEqual(dest, ('192.168.0.57', 5008))

        packet = json.loads(data)

        self.assertEqual(packet["type"], "safety_control")
        self.assertEqual(packet["sequence"], 0)
        self.assertEqual(packet["session_id"], sender.session_id)
        self.assertEqual(packet["mode"], 2)
        self.assertEqual(packet["limit_percent"], 0)
        self.assertEqual(packet["risk_level"], 3)
        self.assertTrue(packet["failsafe"])
        sender.send(dict(control_mode=2, output_limit=0), 3, True, now=1.06)
        self.assertEqual(json.loads(tx.sendto.call_args.args[0])['mode'], 2)
        tx.sendto.side_effect = OSError('unreachable')
        with contextlib.redirect_stdout(self.output):
            sender.send(motor, 3, False, now=2.)
        sender.close()
        tx.close.assert_called_once()

    def test_sender_continuous_rate_above_20hz(self):
        tx = Mock()
        with patch.object(socket, 'socket', return_value=tx):
            sender = CarlaControlSender()
        for tick in range(101):
            sender.send(dict(control_mode=0, output_limit=100), 0, False,
                        now=1. + tick * .01)
        self.assertGreaterEqual(tx.sendto.call_count, 20)
        self.assertEqual([json.loads(call.args[0])['sequence']
                          for call in tx.sendto.call_args_list],
                         list(range(tx.sendto.call_count)))
        sender.close()

    def test_main_applies_immediately_before_vehicle(self):
        main = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        for node in ast.walk(main):
            for field in ('body', 'orelse', 'finalbody'):
                statements = getattr(node, field, [])
                for index, statement in enumerate(statements):
                    if ast.unparse(statement) == 'vehicle.apply_control(control)':
                        self.assertEqual(ast.unparse(statements[index - 1]),
                            'safety_control_receiver.apply(control, time.monotonic(), gear=applied_gear, speed_kmh=current_speed_kmh, creep_throttle=creep_throttle, driver_throttle=driver_throttle)')
                        return
        self.fail('vehicle control call missing')
