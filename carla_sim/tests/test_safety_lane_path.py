import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import time
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / "carla_pedal_control_v17_safety_json_5006.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))
FUNCTIONS = {
    "get_speed_kmh", "get_forward_speed_mps", "cached_safety_box", "safety_steering_geometry",
    "safety_motion_state",
    "predict_safety_pose", "safety_boxes_overlap", "predicted_safety_ttc",
    "safety_vertical_bounds", "safety_future_path_occupancy",
    "safety_waypoint_path", "safety_path_projection", "safety_path_connected",
    "safety_static_grid", "safety_route_pose", "find_static_obstacle_for_safety",
    "safety_candidate_id",
    "draw_safety_route_debug",
    "find_front_vehicle_for_safety",
    "build_safety_telemetry",
}
NODES = [node for node in TREE.body if (
    isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS
) or (
    isinstance(node, ast.Assign) and any(
        isinstance(target, ast.Name)
        and (target.id.startswith("SAFETY_")
             or target.id == "NIGHT_SCENARIO_OBSTACLE_ID")
        for target in node.targets
    )
)]
NAMESPACE = {"math": math, "time": time}
NAMESPACE["carla"] = SimpleNamespace(CityObjectLabel=SimpleNamespace(
    Walls="Walls", Fences="Fences", GuardRail="GuardRail",
    Static="Static", Poles="Poles"))
exec(compile(ast.Module(body=NODES, type_ignores=[]), str(SOURCE), "exec"), NAMESPACE)


class Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Transform:
    def __init__(self, x, y, heading=0.0, z=0.0):
        self.location = Vector(x, y, z)
        self.heading = heading

    def get_forward_vector(self):
        return Vector(math.cos(self.heading), math.sin(self.heading))

    def get_right_vector(self):
        return Vector(-math.sin(self.heading), math.cos(self.heading))


class RoadMap:
    def __init__(self, curve=False):
        self.curve = curve

    def lane_center(self, x, lane_id):
        return (0.02 * x * x if self.curve else 0.0) + lane_id * 3.5

    def lane_height(self, x, lane_id):
        return 0.0

    def get_waypoint(self, location, project_to_road=False):
        lane_id = min((-1, 0, 1), key=lambda lane: abs(location.y - self.lane_center(location.x, lane)))
        if not project_to_road and abs(location.y - self.lane_center(location.x, lane_id)) > 1.75:
            return None
        return Waypoint(self, location.x, lane_id)


class Waypoint:
    def __init__(self, road_map, x, lane_id):
        self.road_map = road_map
        self.x = x
        self.road_id = 1
        self.section_id = 0
        self.lane_id = lane_id
        self.lane_type = "Driving"
        self.lane_width = 3.5
        slope = 0.04 * x if road_map.curve else 0.0
        self.transform = Transform(x, road_map.lane_center(x, lane_id),
                                   math.atan(slope), road_map.lane_height(x, lane_id))

    def next(self, distance):
        return [Waypoint(self.road_map, self.x + distance, self.lane_id)]

    def previous(self, distance):
        return [Waypoint(self.road_map, self.x - distance, self.lane_id)]


class JoinedRoadMap(RoadMap):
    def get_waypoint(self, location, project_to_road=False):
        waypoint = super().get_waypoint(location, project_to_road)
        return JoinedWaypoint(self, waypoint.x, waypoint.lane_id) if waypoint else None


class JoinedWaypoint(Waypoint):
    def __init__(self, road_map, x, physical_lane):
        super().__init__(road_map, x, physical_lane)
        self.physical_lane = physical_lane
        if x >= 15:
            self.road_id = 2
            self.lane_id = 7 + physical_lane

    def next(self, distance):
        return [JoinedWaypoint(self.road_map, self.x + distance, self.physical_lane)]

    def previous(self, distance):
        return [JoinedWaypoint(self.road_map, self.x - distance, self.physical_lane)]


class LayeredRoadMap(RoadMap):
    def get_waypoint(self, location, project_to_road=False):
        lane_id = min((-1, 0, 1), key=lambda lane: abs(location.y - self.lane_center(location.x, lane)))
        if not project_to_road and abs(location.y - self.lane_center(location.x, lane_id)) > 1.75:
            return None
        return LayeredWaypoint(self, location.x, lane_id, round(location.z / 5.0))


class SameIdLayeredRoadMap(LayeredRoadMap):
    def get_waypoint(self, location, project_to_road=False):
        waypoint = super().get_waypoint(location, project_to_road)
        if waypoint is not None:
            waypoint.road_id = 1
        return waypoint


class LayeredWaypoint(Waypoint):
    def __init__(self, road_map, x, lane_id, level):
        super().__init__(road_map, x, lane_id)
        self.level = level
        self.road_id = level + 1
        self.transform.location.z = level * 5.0

    def next(self, distance):
        return [LayeredWaypoint(self.road_map, self.x + distance, self.lane_id, self.level)]

    def previous(self, distance):
        return [LayeredWaypoint(self.road_map, self.x - distance, self.lane_id, self.level)]


class SlopedRoadMap(RoadMap):
    def __init__(self, start_z, grade):
        super().__init__()
        self.start_z = start_z
        self.grade = grade

    def lane_height(self, x, lane_id):
        return self.start_z + self.grade * x


class ForkWaypoint(Waypoint):
    def next(self, distance):
        if self.x >= 10:
            left = Waypoint(self.road_map, self.x + distance, self.lane_id)
            right = Waypoint(self.road_map, self.x + distance, self.lane_id)
            left.transform.heading = 0.1
            right.transform.heading = -0.1
            return [left, right]
        return [ForkWaypoint(self.road_map, self.x + distance, self.lane_id)]


class ForkRoadMap(RoadMap):
    def get_waypoint(self, location, project_to_road=False):
        waypoint = super().get_waypoint(location, project_to_road)
        return ForkWaypoint(self, waypoint.x, waypoint.lane_id) if waypoint else None


class Actor:
    def __init__(self, actor_id, x, y, velocity_x=0.0, velocity_y=0.0,
                 heading=0.0, type_id="vehicle.mock", width=1.0, length=2.0,
                 z=0.0, velocity_z=0.0, height=0.8):
        self.id = actor_id
        self.type_id = type_id
        self.transform = Transform(x, y, heading, z)
        self.velocity = Vector(velocity_x, velocity_y, velocity_z)
        self.bounding_box = SimpleNamespace(extent=Vector(length, width, height), location=Vector())

    def get_transform(self):
        return self.transform

    def get_location(self):
        return self.transform.location

    def get_velocity(self):
        return self.velocity

    def get_angular_velocity(self):
        return Vector()

    def get_physics_control(self):
        return SimpleNamespace(wheels=[
            SimpleNamespace(position=Vector(140), max_steer_angle=35),
            SimpleNamespace(position=Vector(140), max_steer_angle=35),
            SimpleNamespace(position=Vector(-140), max_steer_angle=0),
            SimpleNamespace(position=Vector(-140), max_steer_angle=0),
        ])


class World:
    def __init__(self, road_map, actors):
        self.road_map = road_map
        self.actors = actors

    def get_map(self):
        return self.road_map

    def get_actors(self):
        return self.actors

    def get_environment_objects(self, label):
        return getattr(self, "static_objects", []) if label == "Walls" else []


class SafetyLanePathTest(unittest.TestCase):
    def setUp(self):
        NAMESPACE["SAFETY_STATIC_CACHE"].clear()
        self.road_map = RoadMap()
        self.ego = Actor(1, 0, 0, velocity_x=10)

    def detect(self, actors, road_map=None, ego=None, gear="D", steer=0.0):
        road_map = self.road_map if road_map is None else road_map
        ego = self.ego if ego is None else ego
        world = World(road_map, [ego, *actors])
        return NAMESPACE["find_front_vehicle_for_safety"](
            world, ego, max_distance=50, actors=world.actors,
            road_map=road_map, gear=gear, steer=steer,
        )

    def test_adjacent_left_and_right_lanes_are_not_hazards(self):
        for lane_y in (-3.5, 3.5):
            with self.subTest(lane_y=lane_y):
                self.assertIsNone(self.detect([Actor(2, 12, lane_y, velocity_x=10)]))

    def test_oncoming_vehicle_in_its_own_lane_is_not_hazard(self):
        self.assertIsNone(self.detect([Actor(2, 18, -3.5, velocity_x=-10,
                                              heading=math.pi)]))

    def test_wrong_way_vehicle_in_ego_lane_is_hazard(self):
        self.assertEqual(self.detect([Actor(2, 18, 0, velocity_x=-10,
                                            heading=math.pi)])["actor"].id, 2)

    def test_oncoming_vehicle_crossing_ego_path_is_hazard(self):
        crossing = Actor(2, 12, -3.5, velocity_y=2, heading=math.pi / 2)
        self.assertEqual(self.detect([crossing])["actor"].id, 2)

    def test_uncertain_junction_branch_does_not_extend_following_lane(self):
        road_map = ForkRoadMap()
        route = NAMESPACE["safety_waypoint_path"](
            road_map, self.ego.get_transform(), 50, "D", 0.0)
        self.assertEqual(len(route), 3)
        distant = Actor(2, 40, 0, velocity_x=10)
        self.assertIsNone(self.detect([distant], road_map=road_map))

    def test_lane_change_intent_without_predicted_overlap_is_not_hazard(self):
        changing_ego = Actor(1, 0, 0, velocity_x=10, velocity_y=0.5)
        parallel = Actor(2, 18, 3.5, velocity_x=10)
        self.assertIsNone(self.detect([parallel], ego=changing_ego, steer=0.12))

    def test_steering_command_alone_does_not_predict_adjacent_lane_collision(self):
        neighbor = Actor(2, 12, 3.5, velocity_x=10)
        self.assertIsNone(self.detect([neighbor], steer=0.5))

    def test_previous_target_is_held_only_when_equally_safe(self):
        leader = Actor(2, 21, 0, velocity_x=2)
        near = Actor(3, 20.5, 0, velocity_x=2)
        world = World(self.road_map, [self.ego, leader, near])
        detect = NAMESPACE["find_front_vehicle_for_safety"]
        selected = detect(world, self.ego, actors=world.actors,
                          road_map=self.road_map, preferred_target_id=2)
        self.assertEqual(selected["actor"].id, 2)
        near.transform.location.x = 12
        selected = detect(world, self.ego, actors=world.actors,
                          road_map=self.road_map, preferred_target_id=2)
        self.assertEqual(selected["actor"].id, 3)

    def test_confirmed_turn_away_does_not_keep_distant_lane_target(self):
        changing_ego = Actor(1, 0, 0, velocity_x=10, velocity_y=0.5)
        self.assertIsNone(self.detect([Actor(2, 28, 0)], ego=changing_ego, steer=0.2))

    def test_stopped_ego_ignores_vehicle_turning_away_across_route(self):
        stopped_ego = Actor(1, 0, 0)
        turning = Actor(2, 12, 0, velocity_x=-1, velocity_y=3,
                        heading=math.atan2(3, -1))
        self.assertIsNone(self.detect([turning], ego=stopped_ego))

    def test_stopped_ego_ignores_shallow_turn_away(self):
        stopped_ego = Actor(1, 0, 0)
        turning = Actor(2, 12, 0, velocity_x=-1, velocity_y=3,
                        heading=0.3)
        self.assertIsNone(self.detect([turning], ego=stopped_ego))

    def test_stopped_ego_retains_vehicle_that_will_enter_ego_box(self):
        stopped_ego = Actor(1, 0, 0)
        approaching = Actor(2, 9, 0, velocity_x=-4, heading=math.pi)
        selected = self.detect([approaching], ego=stopped_ego)
        self.assertEqual(selected["actor"].id, 2)
        self.assertIsNotNone(selected["ttc_s"])

    def test_under_bridge_excludes_overhead_vehicle(self):
        road_map = LayeredRoadMap()
        self.assertIsNone(self.detect([Actor(2, 15, 0, z=5)], road_map=road_map))

    def test_over_bridge_excludes_vehicle_below(self):
        road_map = LayeredRoadMap()
        upper_ego = Actor(1, 0, 0, velocity_x=10, z=5)
        self.assertIsNone(self.detect([Actor(2, 15, 0)], road_map=road_map,
                                      ego=upper_ego))

    def test_multilevel_road_excludes_other_level(self):
        road_map = LayeredRoadMap()
        upper_ego = Actor(1, 0, 0, velocity_x=10, z=10)
        self.assertIsNone(self.detect([Actor(2, 15, 0, z=0), Actor(3, 18, 0, z=5)],
                                      road_map=road_map, ego=upper_ego))

    def test_identical_road_and_lane_ids_do_not_override_height(self):
        self.assertIsNone(self.detect([Actor(2, 15, 0, z=5)],
                                      road_map=SameIdLayeredRoadMap()))

    def test_world_bounding_vertices_control_vertical_overlap(self):
        road_map = SameIdLayeredRoadMap()
        upper = Actor(2, 15, 0, z=3)
        upper.bounding_box.get_world_vertices = lambda transform: [
            Vector(z=2.0), Vector(z=4.0)]
        self.assertIsNone(self.detect([upper], road_map=road_map))
        upper.bounding_box.get_world_vertices = lambda transform: [
            Vector(z=0.5), Vector(z=3.5)]
        self.assertEqual(self.detect([upper], road_map=road_map)["actor"].id, 2)

    def test_uphill_and_downhill_same_lane_remain_hazards(self):
        for start_z, grade in ((0, 0.2), (5, -0.2)):
            with self.subTest(grade=grade):
                road_map = SlopedRoadMap(start_z, grade)
                ego = Actor(1, 0, 0, velocity_x=10, z=start_z)
                leader = Actor(2, 20, 0, velocity_x=3,
                               z=road_map.lane_height(20, 0))
                self.assertEqual(self.detect([leader], road_map=road_map,
                                             ego=ego)["actor"].id, 2)

    def test_nearer_upper_road_does_not_replace_connected_leader(self):
        road_map = LayeredRoadMap()
        upper = Actor(2, 10, 0, z=5)
        leader = Actor(3, 25, 0, velocity_x=4)
        self.assertEqual(self.detect([upper, leader], road_map=road_map)["actor"].id, 3)

    def test_vehicle_diverging_to_upper_level_is_not_following_hazard(self):
        diverging = Actor(2, 15, 0, velocity_x=10, velocity_z=2)
        self.assertIsNone(self.detect([diverging]))

    def test_removed_leader_does_not_leave_upper_road_distance_in_telemetry(self):
        road_map = LayeredRoadMap()
        upper = Actor(2, 10, 0, z=5)
        leader = Actor(3, 25, 0, velocity_x=4)
        world = World(road_map, [self.ego, upper, leader])
        build = NAMESPACE["build_safety_telemetry"]
        first = build(world, self.ego, 1, False, None, road_map=road_map)
        self.assertEqual(first["obstacle_id"], 3)
        world.actors = [self.ego, upper]
        cleared = build(world, self.ego, 2, False, None, road_map=road_map,
                        escape_target_id=3)
        self.assertIsNone(cleared["obstacle_id"])
        self.assertIsNone(cleared["distance_m"])
        self.assertIsNone(cleared["closing_speed_mps"])
        self.assertIsNone(cleared["ttc_s"])

    def test_pedestrians_are_not_safety_targets_in_vehicle_only_mode(self):
        road_map = LayeredRoadMap()
        overhead = Actor(2, 8, 0, type_id="walker.pedestrian.mock", z=5)
        crossing = Actor(3, 15, -6, velocity_y=4,
                         heading=math.pi / 2, type_id="walker.pedestrian.mock",
                         width=0.4, length=0.4)
        self.assertIsNone(self.detect([overhead, crossing], road_map=road_map))

    def test_ramp_merging_into_ego_path_remains_hazard(self):
        road_map = SlopedRoadMap(0, 0.2)
        merging = Actor(2, 18, 3.5, velocity_y=-2, heading=-math.pi / 2,
                        z=road_map.lane_height(18, 1))
        result = self.detect([merging], road_map=road_map)
        self.assertIsNotNone(result)
        self.assertEqual(result["actor"].id, 2)
        self.assertIsNotNone(result["ttc_s"])

    def test_debug_reports_height_and_route_exclusion(self):
        road_map = LayeredRoadMap()
        records = []
        world = World(road_map, [self.ego, Actor(2, 15, 0, z=5)])
        result = NAMESPACE["find_front_vehicle_for_safety"](
            world, self.ego, actors=world.actors, road_map=road_map,
            debug=records.append)
        self.assertIsNone(result)
        self.assertEqual(records[0]["reason"], "height_separation")
        self.assertEqual(records[0]["road_lane"], (2, 0, 0))
        self.assertFalse(records[0]["on_path"])

    def test_debug_draws_route_and_selected_bounding_box(self):
        target = Actor(2, 15, 0)
        target.bounding_box.get_world_vertices = lambda transform: [Vector() for _ in range(8)]
        world = World(self.road_map, [self.ego, target])
        world.debug = SimpleNamespace(draw_line=Mock())
        world.get_actor = lambda actor_id: target if actor_id == 2 else None
        original_carla = NAMESPACE["carla"]
        NAMESPACE["carla"] = SimpleNamespace(
            Location=Vector, Color=lambda red, green, blue: (red, green, blue))
        try:
            NAMESPACE["draw_safety_route_debug"](
                world, self.ego, self.road_map, "D", 0.0, target.id)
        finally:
            NAMESPACE["carla"] = original_carla
        self.assertGreater(world.debug.draw_line.call_count, 12)
        self.assertEqual(world.debug.draw_line.call_args.kwargs["color"], (255, 0, 0))

    def test_static_wall_is_not_a_safety_target_in_vehicle_only_mode(self):
        world = World(self.road_map, [self.ego])
        world.static_objects = [SimpleNamespace(id=7, bounding_box=SimpleNamespace(
            location=Vector(12, 0, 0), extent=Vector(0.2, 2, 1),
            rotation=SimpleNamespace(yaw=0)))]
        selected = NAMESPACE["find_front_vehicle_for_safety"](
            world, self.ego, actors=world.actors, road_map=self.road_map)
        self.assertIsNone(selected)
        packet = NAMESPACE["build_safety_telemetry"](
            world, self.ego, 1, False, None, road_map=self.road_map)
        self.assertIsNone(packet["obstacle_id"])
        self.assertIsNone(packet["distance_m"])

    def test_virtual_preview_is_not_a_safety_target(self):
        world = World(self.road_map, [self.ego])
        preview = NAMESPACE["build_safety_telemetry"](
            world, self.ego, 1, True, 10.0, road_map=self.road_map)
        self.assertIsNone(preview["obstacle_id"])
        world.actors.append(Actor(2, 15, 0))
        materialized = NAMESPACE["build_safety_telemetry"](
            world, self.ego, 2, True, 10.0, True, road_map=self.road_map)
        self.assertEqual(materialized["obstacle_id"], 2)

    def test_overhead_and_roadside_static_objects_are_excluded(self):
        world = World(self.road_map, [self.ego])
        world.static_objects = [
            SimpleNamespace(id=7, bounding_box=SimpleNamespace(
                location=Vector(12, 0, 5), extent=Vector(0.2, 2, 1),
                rotation=SimpleNamespace(yaw=0))),
            SimpleNamespace(id=8, bounding_box=SimpleNamespace(
                location=Vector(13, 5, 0), extent=Vector(0.2, 0.5, 1),
                rotation=SimpleNamespace(yaw=0))),
        ]
        self.assertIsNone(NAMESPACE["find_front_vehicle_for_safety"](
            world, self.ego, actors=world.actors, road_map=self.road_map))

    def test_same_lane_leader_and_nearer_neighbors(self):
        leader = Actor(2, 25, 0, velocity_x=4)
        self.assertEqual(self.detect([leader])["actor"].id, 2)
        neighbors = [Actor(3, 10, -3.5, velocity_x=8),
                     Actor(4, 11, 3.5, velocity_x=8)]
        self.assertEqual(self.detect([*neighbors, leader])["actor"].id, 2)

    def test_rear_vehicle_is_not_front_hazard(self):
        self.assertIsNone(self.detect([Actor(2, -12, 0, velocity_x=12)]))

    def test_cutting_in_vehicle_is_detected(self):
        intruder = Actor(2, 18, 3.5, velocity_y=-2, heading=-math.pi / 2)
        result = self.detect([intruder])
        self.assertIsNotNone(result)
        self.assertEqual(result["actor"].id, 2)
        self.assertIsNotNone(result["ttc_s"])

    def test_confirmed_ego_lane_change_includes_target_lane(self):
        changing_ego = Actor(1, 0, 0, velocity_x=10, velocity_y=0.5)
        target = Actor(2, 18, 3.5)
        result = self.detect([target], ego=changing_ego, steer=0.12)
        self.assertIsNotNone(result)
        self.assertEqual(result["actor"].id, 2)

    def test_connected_road_and_lane_ids_follow_geometry(self):
        leader = Actor(2, 25, 0, velocity_x=4)
        result = self.detect([leader], road_map=JoinedRoadMap())
        self.assertIsNotNone(result)
        self.assertEqual(result["actor"].id, 2)

    def test_curve_uses_route_not_world_lateral_offset(self):
        road_map = RoadMap(curve=True)
        leader = Actor(2, 20, road_map.lane_center(20, 0), velocity_x=2,
                       heading=math.atan(0.8))
        neighbor = Actor(3, 15, road_map.lane_center(15, 1), velocity_x=2,
                         heading=math.atan(0.6))
        result = self.detect([neighbor, leader], road_map=road_map)
        self.assertIsNotNone(result)
        self.assertEqual(result["actor"].id, 2)
        self.assertIsNone(self.detect([neighbor], road_map=road_map))

    def test_intersection_crossing_vehicle_remains_hazard(self):
        crossing_car = Actor(2, 20, -10, velocity_y=5, heading=math.pi / 2)
        pedestrian = Actor(3, 15, -6, velocity_y=4, heading=math.pi / 2,
                           type_id="walker.pedestrian.mock", width=0.4, length=0.4)
        self.assertEqual(self.detect([crossing_car])["actor"].id, 2)
        self.assertIsNone(self.detect([pedestrian]))

    def test_reverse_path_detects_rear_obstacle(self):
        reversing_ego = Actor(1, 0, 0, velocity_x=-10)
        self.assertEqual(self.detect([Actor(2, -12, 0)], ego=reversing_ego,
                                     gear="R")["actor"].id, 2)

    def test_filtered_target_populates_existing_safety_udp_fields(self):
        nearby_neighbor = Actor(2, 10, 3.5, velocity_x=5)
        leader = Actor(3, 25, 0, velocity_x=4)
        world = World(self.road_map, [self.ego, nearby_neighbor, leader])
        packet = NAMESPACE["build_safety_telemetry"](
            world, self.ego, 11, False, None, gear="D", road_map=self.road_map,
        )
        self.assertEqual(packet["source"], "CARLA")
        self.assertEqual(packet["sequence"], 11)
        self.assertEqual(packet["obstacle_id"], 3)
        self.assertGreater(packet["distance_m"], 0)
        self.assertGreater(packet["closing_speed_mps"], 0)
        self.assertGreater(packet["ttc_s"], 0)

        world.actors = [self.ego, nearby_neighbor]
        empty = NAMESPACE["build_safety_telemetry"](
            world, self.ego, 12, False, None, gear="D", road_map=self.road_map,
        )
        self.assertIsNone(empty["obstacle_id"])
        self.assertIsNone(empty["distance_m"])

    def test_target_clears_through_udp_risk_and_hmi(self):
        project_root = SOURCE.parents[1]
        sys.path.insert(0, str(project_root / "safety_ecu"))
        sys.path.insert(0, str(project_root / "hmi_ecu"))
        from carla.receiver_v2 import CarlaReceiverV2
        from control.state import SystemState
        from hmi.state_merge import apply_carla, merge
        from hmi_sender import build_packet
        from risk.bns import BNSCalculator
        from risk.directional_policy import DirectionalRiskPolicy

        leader = Actor(2, 8, 0)
        neighbor = Actor(3, 7, 3.5, velocity_x=10)
        world = World(self.road_map, [self.ego, leader, neighbor])
        receiver = CarlaReceiverV2()
        receiver.sock = Mock()
        state = SystemState()
        state.pedal_connected = True
        calculator = BNSCalculator()
        policy = DirectionalRiskPolicy()

        def cycle(sequence, now):
            packet = NAMESPACE["build_safety_telemetry"](
                world, self.ego, sequence, False, None, road_map=self.road_map)
            receiver.sock.recvfrom.return_value = (
                json.dumps(packet).encode(), ("192.168.50.1", 5006))
            self.assertTrue(receiver.receive())
            state.update_carla(receiver)
            state.bns = calculator.calculate(
                state.carla_distance_cm, state.carla_distance_valid,
                state.carla_vehicle_speed, state.carla_ttc,
                closing_speed_mps=state.carla_closing_speed_mps,
                environment_connected=state.carla_connected)
            state.pms = 0.0
            state.total_risk, state.risk_level = policy.evaluate(
                gear=state.carla_gear,
                distance_m=(state.carla_distance_cm / 100.0
                            if state.carla_distance_cm is not None else None),
                speed_kmh=state.carla_vehicle_speed,
                closing_speed_mps=state.carla_closing_speed_mps,
                ttc_s=state.carla_ttc,
                obstacle_present=state.carla_obstacle_present,
                bns=state.bns, bns_available=calculator.available,
                pms=state.pms, pms_available=True, now=now,
                sequence=state.carla_sequence, obstacle_id=state.carla_obstacle_id)
            hmi_packet = build_packet(
                state, seq=sequence, input_mode="CARLA",
                bns_available=calculator.available, pms_available=True,
                risk_available=policy.available, failsafe_active=False,
                failsafe_reason=None, motor_connected=True,
                commands={"haptic": {"vibration_command": 0, "intensity": 0,
                                      "frequency": 0},
                          "motor": {"control_mode": 0, "output_limit": 100,
                                    "command_flags": 0}},
                risk_cause=policy.risk_cause)
            hmi_state = merge({}, {"data": hmi_packet, "t": now}, now=now)
            apply_carla(hmi_state, {"lead": {"distance": 4.0}})
            return packet, hmi_state

        for sequence in (1, 2, 3):
            hazard_packet, hazard_hmi = cycle(sequence, sequence * 0.1)
        self.assertEqual(hazard_packet["obstacle_id"], 2)
        self.assertEqual(hazard_hmi["risk_level"], 3)

        world.actors = [self.ego, neighbor]
        for sequence, now in ((4, 0.4), (5, 1.0), (6, 1.6), (7, 2.2)):
            clear_packet, clear_hmi = cycle(sequence, now)
        self.assertIsNone(clear_packet["obstacle_id"])
        self.assertIsNone(clear_packet["distance_m"])
        self.assertIsNone(state.carla_distance_cm)
        self.assertTrue(state.carla_connected)
        self.assertEqual(clear_hmi["risk_level"], 0)
        self.assertIsNone(clear_hmi["distance_m"])
        self.assertEqual(clear_hmi["banner"]["severity"], "ok")


if __name__ == "__main__":
    unittest.main()
