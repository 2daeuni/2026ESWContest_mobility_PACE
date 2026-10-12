import math


class ForwardObstacleDetector:
    def __init__(self, max_distance=50.0):
        self.max_distance = max_distance

    def detect(self, world, ego):
        if ego is None or not ego.is_alive:
            return None

        ego_transform = ego.get_transform()
        ego_location = ego_transform.location
        forward = ego_transform.get_forward_vector()

        ego_velocity = ego.get_velocity()

        closest = None
        closest_gap = float("inf")

        vehicles = world.get_actors().filter("vehicle.*")

        for vehicle in vehicles:
            if vehicle.id == ego.id:
                continue

            if not vehicle.is_alive:
                continue

            obstacle_transform = vehicle.get_transform()
            obstacle_location = obstacle_transform.location

            relative = obstacle_location - ego_location

            longitudinal = (
                relative.x * forward.x
                + relative.y * forward.y
            )

            lateral = abs(
                relative.x * (-forward.y)
                + relative.y * forward.x
            )

            if longitudinal <= 0:
                continue

            if longitudinal > self.max_distance:
                continue

            if lateral > 2.5:
                continue

            ego_half_length = ego.bounding_box.extent.x
            obstacle_half_length = vehicle.bounding_box.extent.x

            gap = max(
                0.0,
                longitudinal
                - ego_half_length
                - obstacle_half_length
            )

            if gap >= closest_gap:
                continue

            obstacle_velocity = vehicle.get_velocity()

            ego_forward_speed = (
                ego_velocity.x * forward.x
                + ego_velocity.y * forward.y
            )

            obstacle_forward_speed = (
                obstacle_velocity.x * forward.x
                + obstacle_velocity.y * forward.y
            )

            closing_speed = (
                ego_forward_speed - obstacle_forward_speed
            )

            if gap <= 0:
                ttc = 0.0
            elif closing_speed > 0.01:
                ttc = gap / closing_speed
            else:
                ttc = None

            closest_gap = gap

            closest = {
                "obstacle_id": vehicle.id,
                "distance_m": gap,
                "closing_speed_mps": closing_speed,
                "ttc_s": ttc,
            }

        return closest