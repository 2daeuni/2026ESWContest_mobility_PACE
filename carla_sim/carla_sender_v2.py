import carla
import json
import math
import socket
import time

from forward_obstacle import ForwardObstacleDetector


CARLA_HOST = "127.0.0.1"
CARLA_PORT = 2000

RASPBERRY_PI_IP = "192.168.50.2"
UDP_PORT = 5006

VEHICLE_ID = 24
SEND_INTERVAL = 0.1


def main():
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(5.0)

    world = client.get_world()
    vehicle = world.get_actor(VEHICLE_ID)

    if vehicle is None or not vehicle.is_alive:
        print("[CARLA] Vehicle not found")
        return

    detector = ForwardObstacleDetector(
        max_distance=50.0
    )

    udp = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    sequence = 0

    print("[CARLA] Sender V2 started")
    print(f"[CARLA] Vehicle ID: {vehicle.id}")
    print(f"[CARLA] UDP destination: {RASPBERRY_PI_IP}:{UDP_PORT}")

    try:
        while True:
            if not vehicle.is_alive:
                print("[CARLA] Vehicle destroyed")
                break

            velocity = vehicle.get_velocity()

            speed_kmh = 3.6 * math.sqrt(
                velocity.x ** 2
                + velocity.y ** 2
                + velocity.z ** 2
            )

            obstacle = detector.detect(
                world,
                vehicle
            )

            distance_m = None
            closing_speed_mps = None
            ttc_s = None
            obstacle_id = None

            if obstacle is not None:
                obstacle_id = obstacle["obstacle_id"]
                distance_m = obstacle["distance_m"]
                closing_speed_mps = obstacle["closing_speed_mps"]
                ttc_s = obstacle["ttc_s"]

            payload = {
                "source": "CARLA",
                "sequence": sequence,
                "vehicle_speed_kmh": round(speed_kmh, 2),
                "obstacle_id": obstacle_id,
                "distance_m": (
                    round(distance_m, 2)
                    if distance_m is not None
                    else None
                ),
                "closing_speed_mps": (
                    round(closing_speed_mps, 2)
                    if closing_speed_mps is not None
                    else None
                ),
                "ttc_s": (
                    round(ttc_s, 2)
                    if ttc_s is not None
                    else None
                )
            }

            message = json.dumps(
                payload,
                allow_nan=False
            )

            udp.sendto(
                message.encode("utf-8"),
                (RASPBERRY_PI_IP, UDP_PORT)
            )

            print(
                f"[CARLA] UDP SENT: {message}",
                flush=True
            )

            sequence = (sequence + 1) % 65536

            time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        print("[CARLA] Sender V2 stopped by user")

    except (RuntimeError, OSError, ValueError) as error:
        print(f"[CARLA] Sender V2 error: {error}")

    finally:
        udp.close()
        print("[CARLA] UDP socket closed")


if __name__ == "__main__":
    main()