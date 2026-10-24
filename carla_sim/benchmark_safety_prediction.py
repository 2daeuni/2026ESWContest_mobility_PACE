"""Compare the safety detector without starting CARLA or changing simulation settings."""

import argparse
import ast
import math
import statistics
import tarfile
import time
from pathlib import Path
from types import SimpleNamespace


SCRIPT = Path(__file__).with_name("carla_pedal_control_v17_safety_json_5006.py")
FUNCTIONS = {"get_forward_speed_mps", "cached_safety_box", "safety_motion_state", "predict_safety_pose",
             "safety_boxes_overlap", "safety_vertical_bounds", "safety_future_path_occupancy",
             "predicted_safety_ttc", "safety_static_grid", "safety_route_pose",
             "find_static_obstacle_for_safety", "safety_candidate_id",
             "find_front_vehicle_for_safety"}


class Actor:
    def __init__(self, actor_id, x, y, speed):
        self.id = actor_id
        self.type_id = "vehicle.test"
        self.x = x
        self.y = y
        self.speed = speed
        self._bounding_box = SimpleNamespace(
            extent=SimpleNamespace(x=1.0, y=0.6, z=0.8),
            location=SimpleNamespace(x=0.0, y=0.0, z=0.0))
        self.calls = {name: 0 for name in ("get_location", "get_transform", "get_velocity",
                                            "get_angular_velocity", "bounding_box")}

    @property
    def bounding_box(self):
        self.calls["bounding_box"] += 1
        return self._bounding_box

    def get_location(self):
        self.calls["get_location"] += 1
        return SimpleNamespace(x=self.x, y=self.y, z=0.0)

    def get_transform(self):
        self.calls["get_transform"] += 1
        return SimpleNamespace(location=SimpleNamespace(x=self.x, y=self.y, z=0.0),
                               get_forward_vector=lambda: SimpleNamespace(x=1.0, y=0.0, z=0.0),
                               get_right_vector=lambda: SimpleNamespace(x=0.0, y=1.0, z=0.0))

    def get_velocity(self):
        self.calls["get_velocity"] += 1
        return SimpleNamespace(x=self.speed, y=0.0, z=0.0)

    def get_angular_velocity(self):
        self.calls["get_angular_velocity"] += 1
        return SimpleNamespace(z=0.0)


def load_detector(source):
    tree = ast.parse(source)
    nodes = [node for node in tree.body if (
        isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS
    ) or (
        isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id.startswith("SAFETY_")
            for target in node.targets)
    )]
    namespace = {"math": math, "time": time, "SAFETY_FRONT_MAX_DISTANCE_M": 50.0,
                 "SAFETY_FRONT_LATERAL_LIMIT_M": 2.5}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), "safety_detector", "exec"), namespace)
    return namespace["find_front_vehicle_for_safety"]


def run(label, source, iterations, count, use_cache=False):
    detect = load_detector(source)
    ego = Actor(1, 0.0, 0.0, 12.0)
    targets = [Actor(index + 2, 5.0 + index, 0.0, 0.0)
               for index in range(count)]
    world = SimpleNamespace(get_actors=lambda: [ego, *targets])
    box_cache = {}
    durations = []
    for _ in range(iterations):
        started = time.perf_counter()
        result = detect(world, ego, box_cache=box_cache) if use_cache else detect(world, ego)
        durations.append((time.perf_counter() - started) * 1000)
        if result is None:
            raise RuntimeError("The benchmark must retain collision detection")
    calls = {name: sum(actor.calls[name] for actor in [ego, *targets]) // iterations
             for name in ego.calls}
    print(f"{label}: actors={count}, mean={statistics.mean(durations):.3f} ms, "
          f"p95={sorted(durations)[int(0.95 * (len(durations) - 1))]:.3f} ms, "
          f"getter calls/scan={calls}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-archive", type=Path)
    parser.add_argument("--baseline-source", type=Path)
    parser.add_argument("--actors", type=int, default=40)
    parser.add_argument("--iterations", type=int, default=50)
    args = parser.parse_args()
    if args.baseline_archive:
        with tarfile.open(args.baseline_archive) as archive:
            source = archive.extractfile(
                "carla_sim/carla_pedal_control_v17_safety_json_5006.py").read().decode()
        run("before", source, args.iterations, args.actors)
    if args.baseline_source:
        run("before", args.baseline_source.read_text(encoding="utf-8"),
            args.iterations, args.actors)
    run("after", SCRIPT.read_text(encoding="utf-8"), args.iterations, args.actors,
        use_cache=True)


if __name__ == "__main__":
    main()
