"""Read-only CARLA motion clock check; does not tick or change world settings."""

import argparse
import json
import math
import time


def measure(world, vehicle, duration):
    first = world.get_snapshot()
    start_location = vehicle.get_location()
    started = time.monotonic()
    speeds = []
    frames = set()
    while time.monotonic() - started < duration:
        snapshot = world.wait_for_tick(1.0)
        if snapshot.frame in frames:
            continue
        frames.add(snapshot.frame)
        velocity = vehicle.get_velocity()
        speeds.append(3.6 * math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2))
    last = world.get_snapshot()
    end_location = vehicle.get_location()
    wall_seconds = time.monotonic() - started
    sim_seconds = last.timestamp.elapsed_seconds - first.timestamp.elapsed_seconds
    displacement = math.hypot(end_location.x - start_location.x,
                              end_location.y - start_location.y)
    return {
        "synchronous_mode": world.get_settings().synchronous_mode,
        "fixed_delta_seconds": world.get_settings().fixed_delta_seconds,
        "wall_seconds": round(wall_seconds, 3),
        "sim_seconds": round(sim_seconds, 3),
        "sim_time_ratio": round(sim_seconds / wall_seconds, 3),
        "server_fps": round((last.frame - first.frame) / wall_seconds, 2),
        "sampled_fps": round(len(frames) / wall_seconds, 2),
        "mean_velocity_kmh": round(sum(speeds) / len(speeds), 2) if speeds else None,
        "displacement_m": round(displacement, 2),
        "wall_displacement_kmh": round(displacement / wall_seconds * 3.6, 2),
        "sim_displacement_kmh": round(displacement / sim_seconds * 3.6, 2)
        if sim_seconds > 0 else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--role-name", default="hero")
    parser.add_argument("--actor-id", type=int)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--hmi-speed-kmh", type=float)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    import carla

    client = carla.Client(args.host, args.port)
    client.set_timeout(3.0)
    world = client.get_world()
    vehicles = world.get_actors().filter("vehicle.*")
    vehicle = next((actor for actor in vehicles if actor.id == args.actor_id), None) if args.actor_id else next(
        (actor for actor in vehicles if actor.attributes.get("role_name") == args.role_name), None)
    if vehicle is None:
        parser.error("ego vehicle not found; use --actor-id or --role-name")
    result = measure(world, vehicle, args.seconds)
    result["actor_id"] = vehicle.id
    if args.hmi_speed_kmh is not None:
        result["hmi_speed_kmh"] = args.hmi_speed_kmh
        result["hmi_minus_carla_kmh"] = round(
            args.hmi_speed_kmh - result["mean_velocity_kmh"], 2)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
