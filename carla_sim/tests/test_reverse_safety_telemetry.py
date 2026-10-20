
"""Unit tests for CARLA reverse obstacle detection and Safety telemetry."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "carla_pedal_control_v17_safety_json_5006.py"
)

TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))

FUNCTION_NAMES = {
    "get_speed_kmh",
    "get_forward_speed_mps",
    "find_front_vehicle_for_safety",
    "find_rear_vehicle_for_safety",
    "build_safety_telemetry",
}

NAMESPACE = {
    "math": math,
    "SAFETY_FRONT_MAX_DISTANCE_M": 50.0,
    "SAFETY_FRONT_LATERAL_LIMIT_M": 2.5,
    "NIGHT_SCENARIO_OBSTACLE_ID": 900001,
}

functions = [
    node
    for node in TREE.body
    if isinstance(node, ast.FunctionDef)
    and node.name in FUNCTION_NAMES
]

assert len(functions) == len(FUNCTION_NAMES), (
    "Required CARLA Safety functions are missing"
)

exec(
    compile(
        ast.Module(body=functions, type_ignores=[]),
        str(SOURCE),
        "exec",
    ),
    NAMESPACE,
)
NAMESPACE["find_extra_obstacle_for_safety"] = lambda world, ego, gear: None

def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


class FakeVehicle:
    def __init__(
        self,
        actor_id,
        x,
        y=0.0,
        velocity_x=0.0,
        extent_x=2.0,
    ):
        self.id = actor_id
        self.location = vector(x, y)
        self.velocity = vector(velocity_x)
        self.bounding_box = SimpleNamespace(
            extent=vector(extent_x)
        )

    def get_location(self):
        return self.location

    def get_velocity(self):
        return self.velocity

    def get_transform(self):
        return SimpleNamespace(
            location=self.location,
            get_forward_vector=lambda: vector(1.0),
            get_right_vector=lambda: vector(0.0, 1.0),
        )

class FakeActors:
    def __init__(self, vehicles):
        self.vehicles = vehicles

    def filter(self, pattern):
        assert pattern == "vehicle.*"
        return self.vehicles


class FakeWorld:
    def __init__(self, vehicles):
        self.vehicles = vehicles

    def get_actors(self):
        return FakeActors(self.vehicles)


class ReverseSafetyTelemetryTests(unittest.TestCase):

    def setUp(self):
        self.ego = FakeVehicle(
            actor_id=1,
            x=0.0,
        )

    def test_rear_vehicle_distance(self):
        rear = FakeVehicle(2, x=-14.0)
        world = FakeWorld([self.ego, rear])

        result = NAMESPACE["find_rear_vehicle_for_safety"](
            world, self.ego
        )

        self.assertIsNotNone(result)
        self.assertEqual(result["actor"].id, 2)
        self.assertAlmostEqual(result["distance_m"], 10.0)

    def test_reverse_closing_speed_and_ttc(self):
        self.ego.velocity = vector(-5.0)
        rear = FakeVehicle(2, x=-14.0, velocity_x=0.0)
        world = FakeWorld([self.ego, rear])

        result = NAMESPACE["find_rear_vehicle_for_safety"](
            world, self.ego
        )

        self.assertAlmostEqual(
            result["closing_speed_mps"], 5.0
        )
        self.assertAlmostEqual(result["distance_m"], 10.0)
        self.assertAlmostEqual(result["ttc_s"], 2.0)

    def test_rear_vehicle_moving_away(self):
        self.ego.velocity = vector(-2.0)
        rear = FakeVehicle(2, x=-14.0, velocity_x=-5.0)
        world = FakeWorld([self.ego, rear])

        result = NAMESPACE["find_rear_vehicle_for_safety"](
            world, self.ego
        )

        self.assertAlmostEqual(
            result["closing_speed_mps"], -3.0
        )
        self.assertIsNone(result["ttc_s"])

    def test_front_and_side_vehicles_are_excluded(self):
        front = FakeVehicle(2, x=10.0)
        side = FakeVehicle(3, x=-10.0, y=4.0)
        world = FakeWorld([self.ego, front, side])

        result = NAMESPACE["find_rear_vehicle_for_safety"](
            world, self.ego
        )

        self.assertIsNone(result)

    def test_nearest_rear_vehicle_selected(self):
        far = FakeVehicle(2, x=-24.0)
        near = FakeVehicle(3, x=-9.0)
        world = FakeWorld([self.ego, far, near])

        result = NAMESPACE["find_rear_vehicle_for_safety"](
            world, self.ego
        )

        self.assertEqual(result["actor"].id, 3)
        self.assertAlmostEqual(result["distance_m"], 5.0)

    def test_rear_vehicle_outside_detection_range(self):
        rear = FakeVehicle(2, x=-60.0)
        world = FakeWorld([self.ego, rear])

        result = NAMESPACE["find_rear_vehicle_for_safety"](
            world, self.ego
        )

        self.assertIsNone(result)

    def test_reverse_telemetry_uses_rear_vehicle(self):
        self.ego.velocity = vector(-5.0)
        rear = FakeVehicle(2, x=-14.0)
        front = FakeVehicle(3, x=8.0)
        world = FakeWorld([self.ego, rear, front])

        payload = NAMESPACE["build_safety_telemetry"](
            world,
            self.ego,
            sequence=7,
            virtual_obstacle_active=False,
            virtual_obstacle_distance=None,
            gear="R",
        )

        self.assertEqual(payload["source"], "CARLA")
        self.assertEqual(payload["sequence"], 7)
        self.assertEqual(payload["obstacle_id"], 2)
        self.assertAlmostEqual(payload["distance_m"], 10.0)
        self.assertAlmostEqual(
            payload["closing_speed_mps"], 5.0
        )
        self.assertAlmostEqual(payload["ttc_s"], 2.0)
        self.assertAlmostEqual(
            payload["vehicle_speed_kmh"], 18.0
        )

    def test_reverse_ignores_front_virtual_obstacle(self):
        rear = FakeVehicle(2, x=-14.0)
        world = FakeWorld([self.ego, rear])

        payload = NAMESPACE["build_safety_telemetry"](
            world,
            self.ego,
            sequence=8,
            virtual_obstacle_active=True,
            virtual_obstacle_distance=20.0,
            gear="R",
        )

        self.assertEqual(payload["obstacle_id"], 2)
        self.assertAlmostEqual(payload["distance_m"], 10.0)

    def test_no_rear_vehicle_returns_null_fields(self):
        front = FakeVehicle(2, x=10.0)
        world = FakeWorld([self.ego, front])

        payload = NAMESPACE["build_safety_telemetry"](
            world,
            self.ego,
            sequence=9,
            virtual_obstacle_active=False,
            virtual_obstacle_distance=None,
            gear="R",
        )

        for field in (
            "obstacle_id",
            "distance_m",
            "closing_speed_mps",
            "ttc_s",
        ):
            self.assertIsNone(payload[field])

    def test_park_has_no_obstacle_target(self):
        rear = FakeVehicle(2, x=-14.0)
        world = FakeWorld([self.ego, rear])

        payload = NAMESPACE["build_safety_telemetry"](
            world,
            self.ego,
            sequence=10,
            virtual_obstacle_active=False,
            virtual_obstacle_distance=None,
            gear="P",
        )

        self.assertIsNone(payload["obstacle_id"])
        self.assertIsNone(payload["distance_m"])
        self.assertIsNone(payload["ttc_s"])


if __name__ == "__main__":
    unittest.main()
