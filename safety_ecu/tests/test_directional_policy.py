import ast
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from risk.directional_policy import DirectionalRiskPolicy
from risk.bns import BNSCalculator
from carla.receiver_v2 import CarlaReceiverV2


class DirectionalPolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = DirectionalRiskPolicy()

    def step(self, now, gear="D", distance=50.0, speed=30.0, closing=5.0,
             ttc=None, obstacle=True, bns_available=True, brake=0,
             accelerator=0, sequence=None, obstacle_id=7, signed_speed=None,
             escape_id=None, escape_distance=None, pedal_l1=False,
             pedal_l2=False, pedal_l3=False):
        return self.policy.evaluate(
            gear=gear, distance_m=distance, speed_kmh=speed,
            closing_speed_mps=closing if obstacle else 0.0, ttc_s=ttc,
            obstacle_present=obstacle, bns=0.5, bns_available=bns_available,
            pms=0.0, pms_available=True, now=now,
            brake=brake, accelerator=accelerator, sequence=sequence,
            obstacle_id=obstacle_id if obstacle else None,
            signed_speed_mps=signed_speed,
            escape_obstacle_id=escape_id, escape_distance_m=escape_distance,
            pedal_l1=pedal_l1, pedal_l2=pedal_l2, pedal_l3=pedal_l3,
        )[1]

    def test_stopped_braking_ignores_static_obstacle_but_rapid_accel_warns(self):
        self.assertEqual(self.step(0.0, distance=1.0, speed=0.0,
                                   closing=0.0, brake=20), 0)
        self.assertEqual(self.step(0.1, distance=1.0, speed=0.0,
                                   closing=0.0, accelerator=80, pedal_l2=True), 0)
        self.assertEqual(self.policy.candidate_level, 2)
        self.assertEqual(self.step(0.31, distance=1.0, speed=0.0,
                                   closing=0.0, accelerator=80, pedal_l2=True), 2)
        self.assertEqual(self.step(0.4, distance=1.0, speed=0.0,
                                   closing=0.0, brake=20), 0)
        self.assertEqual(self.step(0.5, distance=0.1, speed=0.0,
                                   closing=3.0, brake=20, ttc=0.1), 3)

    def test_single_valid_imminent_condition_stops_immediately_in_both_gears(self):
        for gear in ("D", "R"):
            self.policy = DirectionalRiskPolicy()
            self.assertEqual(self.step(0.0, gear=gear, distance=20, ttc=1.2), 3)
            self.assertEqual(self.policy.reason, "STOPPING_DISTANCE_OR_TTC")

    def test_speed_dependent_stopping_distance_and_receding_target(self):
        self.assertEqual(self.step(0.0, distance=10, closing=12), 3)
        self.assertGreater(self.policy.stopping_distance_m, 10)
        self.policy = DirectionalRiskPolicy()
        self.assertEqual(self.step(0.0, distance=10, closing=-2), 0)
        self.assertIsNone(self.policy.stopping_distance_m)

    def test_confirm_nonemergency_and_hold_before_release(self):
        self.assertEqual(self.step(0.0, distance=7, closing=5), 0)
        self.assertEqual(self.policy.candidate_level, 2)
        self.assertEqual(self.step(0.19, distance=7, closing=5), 0)
        self.assertEqual(self.step(0.2, distance=7, closing=5), 2)
        self.assertEqual(self.step(0.3, obstacle=False), 2)
        self.assertEqual(self.step(0.79, obstacle=False), 2)
        self.assertEqual(self.step(0.8, obstacle=False), 1)
        self.assertEqual(self.step(1.3, obstacle=False), 0)

    def test_gear_change_and_invalid_data(self):
        self.assertEqual(self.step(0.0, distance=1), 3)
        self.assertEqual(self.step(0.1, gear="R", obstacle=False), 3)
        self.assertEqual(self.step(0.2, gear="R", distance=1), 3)
        self.assertEqual(self.step(0.3, gear="R", bns_available=False), 0)
        self.assertFalse(self.policy.available)
        self.assertEqual(self.step(0.4, gear="P", distance=1), 0)
        self.assertEqual(self.step(0.5, gear="D", distance=math.nan), 0)
        self.assertFalse(self.policy.available)

    def test_gear_change_restarts_nonemergency_confirmation(self):
        self.assertEqual(self.step(0.0, distance=7), 0)
        self.assertEqual(self.step(0.2, distance=7), 2)
        self.assertEqual(self.step(0.21, gear="R", distance=7), 2)
        self.assertEqual(self.step(0.42, gear="R", distance=7), 2)

    def test_brake_evidence_speeds_release_but_never_bypasses_emergency(self):
        self.assertEqual(self.step(0.0, distance=4, ttc=1.0), 3)
        self.assertEqual(self.step(0.1, distance=7, speed=28, closing=3,
                                   ttc=2.3, brake=3, sequence=2), 3)
        self.assertGreater(self.policy.observed_deceleration_mps2, 0.2)
        self.assertLess(self.policy.bns_score, 0.5)
        self.assertEqual(self.step(0.36, distance=7, speed=28, closing=3,
                                   ttc=2.3, brake=3, sequence=2), 2)
        self.assertEqual(self.step(0.4, distance=1, speed=25, closing=3,
                                   ttc=0.3, brake=30, sequence=3), 3)

    def test_gear_change_requires_verified_retreat_for_fast_release(self):
        self.assertEqual(self.step(0.0, distance=0, sequence=1), 3)
        self.assertEqual(self.step(0.1, gear="R", obstacle=False, sequence=2,
                                   signed_speed=-0.5, escape_id=7, escape_distance=0), 3)
        self.assertEqual(self.step(0.2, gear="R", obstacle=False, sequence=3,
                                   signed_speed=-1, escape_id=7, escape_distance=0.2), 3)
        self.assertEqual(self.step(0.36, gear="R", obstacle=False, sequence=4,
                                   signed_speed=-1, escape_id=7, escape_distance=0.4), 2)
        self.assertEqual(self.policy.reason, "ESCAPING")

    def test_reverse_collision_then_forward_escape(self):
        self.assertEqual(self.step(0.0, gear="R", distance=0, sequence=1), 3)
        self.assertEqual(self.step(0.1, gear="D", obstacle=False, sequence=2,
                                   signed_speed=0.5, escape_id=7, escape_distance=0), 3)
        self.assertEqual(self.step(0.2, gear="D", obstacle=False, sequence=3,
                                   signed_speed=1, escape_id=7, escape_distance=0.2), 3)
        self.assertEqual(self.step(0.36, gear="D", obstacle=False, sequence=4,
                                   signed_speed=1, escape_id=7, escape_distance=0.4), 2)

    def test_weak_brake_without_measured_improvement_keeps_normal_delay(self):
        self.assertEqual(self.step(0.0, distance=4, ttc=1.0), 3)
        self.assertEqual(self.step(0.1, distance=7, speed=30, closing=3,
                                   ttc=2.3, brake=3), 3)
        self.assertEqual(self.step(0.36, distance=7, speed=30, closing=3,
                                   ttc=2.3, brake=3), 3)
        self.assertEqual(self.step(0.6, distance=7, speed=30, closing=3,
                                   ttc=2.3, brake=3), 2)


SOURCE = Path(__file__).resolve().parents[2] / "carla_sim/carla_pedal_control_v17_safety_json_5006.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))
FINDER_NODES = [node for node in TREE.body if (
    isinstance(node, ast.FunctionDef)
    and node.name in ("cached_safety_box", "safety_motion_state", "predict_safety_pose",
                      "safety_boxes_overlap", "safety_vertical_bounds",
                      "safety_future_path_occupancy", "predicted_safety_ttc",
                      "safety_waypoint_path", "safety_path_projection",
                      "safety_path_connected", "safety_static_grid",
                      "safety_route_pose", "find_static_obstacle_for_safety",
                      "safety_candidate_id", "safety_steering_geometry",
                      "find_front_vehicle_for_safety", "build_safety_telemetry")
) or (
    isinstance(node, ast.Assign)
    and any(isinstance(target, ast.Name) and target.id.startswith("SAFETY_")
            for target in node.targets)
)]


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def actor(actor_id, location_x, velocity_x, actor_type="vehicle.test",
          location_y=0.0, velocity_y=0.0, heading=0.0, yaw_rate=0.0):
    forward = vector(x=math.cos(heading), y=math.sin(heading))
    return SimpleNamespace(
        id=actor_id, type_id=actor_type,
        bounding_box=SimpleNamespace(extent=vector(x=1.0, y=0.6)),
        get_location=lambda: vector(x=location_x, y=location_y),
        get_velocity=lambda: vector(x=velocity_x, y=velocity_y),
        get_angular_velocity=lambda: vector(z=yaw_rate),
        get_transform=lambda: SimpleNamespace(location=vector(x=location_x, y=location_y),
                                               get_forward_vector=lambda: forward),
    )


class DirectionalDetectionTests(unittest.TestCase):
    def setUp(self):
        self.ego = actor(1, 0.0, -5.0)
        self.ego.get_transform = lambda: SimpleNamespace(
            location=vector(), get_forward_vector=lambda: vector(x=1),
            get_right_vector=lambda: vector(y=1),
        )
        self.namespace = dict(math=math, SAFETY_FRONT_MAX_DISTANCE_M=50.0,
                              SAFETY_FRONT_MAX_LOOKAHEAD_M=120.0,
                              SAFETY_FRONT_LOOKAHEAD_S=4.5,
                              SAFETY_FRONT_LATERAL_LIMIT_M=2.5,
                              NIGHT_SCENARIO_OBSTACLE_ID=900001,
                              get_speed_kmh=lambda item: abs(item.get_velocity().x) * 3.6,
                              get_forward_speed_mps=lambda item, forward: item.get_velocity().x)
        exec(compile(ast.Module(body=FINDER_NODES, type_ignores=[]), str(SOURCE), "exec"), self.namespace)

    def test_reverse_selects_rear_actor_and_signed_closing(self):
        front = actor(2, 8.0, 0.0)
        rear = actor(3, -8.0, 0.0)
        world = SimpleNamespace(get_actors=lambda: [self.ego, front, rear])
        find = self.namespace["find_front_vehicle_for_safety"]
        self.assertEqual(find(world, self.ego, gear="R")["actor"].id, 3)
        self.assertEqual(find(world, self.ego, gear="R")["closing_speed_mps"], 5.0)
        self.assertIsNone(find(world, self.ego, gear="D"))

    def test_reverse_pedestrian_and_receding_actor(self):
        self.ego.get_velocity = lambda: vector(x=0)
        rear = actor(4, -6, -2, "walker.pedestrian.test")
        world = SimpleNamespace(get_actors=lambda: [self.ego, rear])
        self.assertIsNone(self.namespace["find_front_vehicle_for_safety"](world, self.ego, gear="R"))

    def test_crossing_path_selected_but_diverging_turn_ignored(self):
        self.ego.get_velocity = lambda: vector(x=5)
        crossing = actor(5, 6, 0, location_y=-6, velocity_y=5,
                         heading=math.pi / 2)
        diverging = actor(6, 6, 0, location_y=4, velocity_y=5,
                          heading=math.pi / 2)
        find = self.namespace["find_front_vehicle_for_safety"]
        self.assertEqual(find(SimpleNamespace(get_actors=lambda: [self.ego, crossing]),
                              self.ego, gear="D")["actor"].id, 5)
        self.assertIsNone(find(SimpleNamespace(get_actors=lambda: [self.ego, diverging]),
                               self.ego, gear="D"))

    def test_steering_path_avoids_straight_oncoming_target(self):
        self.ego.get_velocity = lambda: vector(x=5, y=0.5)
        self.ego.get_physics_control = lambda: SimpleNamespace(wheels=[
            SimpleNamespace(position=vector(x=140), max_steer_angle=35),
            SimpleNamespace(position=vector(x=140), max_steer_angle=35),
            SimpleNamespace(position=vector(x=-140), max_steer_angle=0),
            SimpleNamespace(position=vector(x=-140), max_steer_angle=0),
        ])
        oncoming = actor(7, 14, -5, heading=math.pi)
        world = SimpleNamespace(get_actors=lambda: [self.ego, oncoming])
        find = self.namespace["find_front_vehicle_for_safety"]
        self.assertIsNotNone(find(world, self.ego, gear="D", steer=0.0))
        self.assertIsNone(find(world, self.ego, gear="D", steer=0.6))

    def test_reverse_telemetry_keeps_old_target_distance_for_escape(self):
        self.ego.get_velocity = lambda: vector(x=5)
        front = actor(8, 8, 0)
        world = SimpleNamespace(get_actors=lambda: [self.ego, front],
                                get_actor=lambda actor_id: front if actor_id == 8 else None)
        build = self.namespace["build_safety_telemetry"]
        forward_packet = build(world, self.ego, 1, False, None, gear="D")
        self.assertEqual(forward_packet["obstacle_id"], 8)
        reversing = actor(1, -1, -1)
        reversing.get_transform = lambda: SimpleNamespace(
            location=vector(x=-1), get_forward_vector=lambda: vector(x=1),
            get_right_vector=lambda: vector(y=1),
        )
        world.get_actors = Mock(return_value=[reversing, front])
        world.get_actor = lambda actor_id: self.fail("escape tracking must reuse actor list")
        reverse_packet = build(world, reversing, 2, False, None, gear="R",
                               escape_target_id=8)
        self.assertIsNone(reverse_packet["obstacle_id"])
        self.assertEqual(reverse_packet["escape_obstacle_id"], 8)
        self.assertLess(forward_packet["distance_m"], reverse_packet["escape_distance_m"])
        self.assertLess(reverse_packet["signed_speed_mps"], 0)
        world.get_actors.assert_called_once_with()

    def test_receding_overlap_does_not_hide_approaching_target(self):
        self.ego.get_velocity = lambda: vector(x=5)
        receding = actor(9, 1, 10)
        hazard = actor(10, 10, 0)
        world = SimpleNamespace(get_actors=lambda: [self.ego, receding, hazard])
        detected = self.namespace["find_front_vehicle_for_safety"](world, self.ego)
        self.assertEqual(detected["actor"].id, 10)

    def test_same_speed_leader_is_reported_for_headway(self):
        self.ego.get_velocity = lambda: vector(x=10)
        leader = actor(40, 15, 10)
        world = SimpleNamespace(get_actors=lambda: [self.ego, leader])
        detected = self.namespace["find_front_vehicle_for_safety"](world, self.ego)
        self.assertEqual(detected["actor"].id, 40)
        self.assertEqual(detected["closing_speed_mps"], 0)
        self.assertIsNone(detected["ttc_s"])

    def test_high_speed_lookahead_extends_beyond_fifty_meters(self):
        obstacle = actor(42, 75, 0)
        world = SimpleNamespace(get_actors=lambda: [self.ego, obstacle])
        build = self.namespace["build_safety_telemetry"]
        self.ego.get_velocity = lambda: vector(x=16.67)
        self.assertEqual(build(world, self.ego, 1, False, None)["obstacle_id"], 42)
        self.ego.get_velocity = lambda: vector(x=2.78)
        self.assertIsNone(build(world, self.ego, 2, False, None)["obstacle_id"])

    def test_diverging_turn_is_not_reported_as_following(self):
        self.ego.get_velocity = lambda: vector(x=10)
        diverging = actor(41, 15, 10, yaw_rate=45)
        world = SimpleNamespace(get_actors=lambda: [self.ego, diverging])
        self.assertIsNone(self.namespace["find_front_vehicle_for_safety"](world, self.ego))

    def test_prediction_reads_each_actor_motion_once(self):
        self.ego.get_velocity = lambda: vector(x=5)
        targets = [actor(20, 10, 0), actor(21, 15, 0)]
        calls = {target.id: {"transform": 0, "velocity": 0, "angular": 0}
                 for target in targets}
        for target in targets:
            for getter, label in (("get_transform", "transform"),
                                  ("get_velocity", "velocity"),
                                  ("get_angular_velocity", "angular")):
                original = getattr(target, getter)
                def counted(original=original, label=label, actor_id=target.id):
                    calls[actor_id][label] += 1
                    return original()
                setattr(target, getter, counted)
        world = SimpleNamespace(get_actors=lambda: [self.ego, *targets])
        self.assertEqual(self.namespace["find_front_vehicle_for_safety"](
            world, self.ego)["actor"].id, 20)
        for counts in calls.values():
            self.assertEqual(counts["transform"], 1)
            self.assertEqual(counts["velocity"], 1)
            self.assertEqual(counts["angular"], 1)

    def test_bounding_boxes_are_cached_until_actor_disappears(self):
        self.ego.get_velocity = lambda: vector(x=5)
        target = actor(30, 10, 0)

        class CountingBoxActor:
            def __init__(self, wrapped):
                self.wrapped = wrapped
                self.box_reads = 0

            def __getattr__(self, name):
                if name == "bounding_box":
                    self.box_reads += 1
                return getattr(self.wrapped, name)

        ego = CountingBoxActor(self.ego)
        obstacle = CountingBoxActor(target)
        active = [ego, obstacle]
        world = SimpleNamespace(get_actors=lambda: active)
        find = self.namespace["find_front_vehicle_for_safety"]
        box_cache = {}
        for _ in range(2):
            self.assertEqual(find(world, ego, box_cache=box_cache)["actor"].id, 30)
        self.assertEqual(ego.box_reads, 1)
        self.assertEqual(obstacle.box_reads, 1)
        active = [ego]
        self.assertIsNone(find(world, ego, box_cache=box_cache))
        self.assertNotIn(30, box_cache)

    def test_close_but_nonclosing_vehicle_has_no_collision_score(self):
        calculator = BNSCalculator()
        self.assertEqual(calculator.calculate(200, True, 10, None,
                         closing_speed_mps=0.1, environment_connected=True), 0.65)
        self.assertTrue(calculator.headway_l2)
        self.assertTrue(calculator.available)
        self.assertGreaterEqual(calculator.calculate(0, True, 0, None,
                                closing_speed_mps=0.0, environment_connected=True), 0.9)


DESKTOP_SOURCE = SOURCE
DESKTOP_TREE = ast.parse(DESKTOP_SOURCE.read_text(encoding="utf-8"))
RECEIVER = next(node for node in DESKTOP_TREE.body if isinstance(node, ast.ClassDef)
                and node.name == "SafetyControlReceiver")
CONTROL_NAMESPACE = dict(socket=__import__("socket"), json=json,
                         GLOBAL_SPEED_LIMIT_KMH=80.0,
                         LEVEL2_SPEED_LIMIT_KMH=30.0,
                         LEVEL2_SPEED_APPROACH_KMH=5.0,
                         LEVEL2_SPEED_MIN_BRAKE=0.15,
                         LEVEL2_SPEED_BRAKE_PER_KMH=0.1)
exec(compile(ast.Module(body=[RECEIVER], type_ignores=[]), str(DESKTOP_SOURCE), "exec"),
     CONTROL_NAMESPACE)


class ReverseControlTests(unittest.TestCase):
    def test_gear_match_limit_and_mismatch_stop(self):
        sock = Mock()
        packet = dict(type="safety_control", sequence=1, mode=1,
                      limit_percent=30, risk_level=2, failsafe=False, gear="R")
        sock.recvfrom.side_effect = [(json.dumps(packet).encode(), ("pi", 5008)),
                                     BlockingIOError]
        with patch.object(CONTROL_NAMESPACE["socket"], "socket", return_value=sock):
            receiver = CONTROL_NAMESPACE["SafetyControlReceiver"]()
        control = SimpleNamespace(throttle=1.0, brake=0.0)
        with patch("builtins.print"):
            receiver.apply(control, 1.0, gear="R", speed_kmh=20.0)
        self.assertEqual((control.throttle, control.brake), (0.3, 0.0))
        sock.recvfrom.side_effect = BlockingIOError
        control = SimpleNamespace(throttle=1.0, brake=0.0)
        with patch("builtins.print"):
            receiver.apply(control, 1.1, gear="D", speed_kmh=20.0)
        self.assertEqual((control.throttle, control.brake), (0.0, 1.0))


class GearTelemetryTests(unittest.TestCase):
    def test_gear_is_required_and_reverse_packet_is_accepted(self):
        receiver = CarlaReceiverV2()
        receiver.sock = Mock()
        packet = dict(source="CARLA", sequence=1, vehicle_speed_kmh=8.0,
                      obstacle_id=7, distance_m=3.0, closing_speed_mps=2.0,
                      ttc_s=1.5)
        receiver.sock.recvfrom.return_value = (json.dumps(packet).encode(),
                                                ("192.168.50.1", 5006))
        self.assertFalse(receiver.receive())
        packet["gear"] = "R"
        receiver.sock.recvfrom.return_value = (json.dumps(packet).encode(),
                                                ("192.168.50.1", 5006))
        self.assertTrue(receiver.receive())
        self.assertEqual(receiver.gear, "R")


if __name__ == "__main__":
    unittest.main()
