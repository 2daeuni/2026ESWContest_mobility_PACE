import carla
import math
import socket
import time


CARLA_HOST = "127.0.0.1"
CARLA_PORT = 2000

RASPBERRY_PI_IP = "192.168.50.2"
UDP_PORT = 5005

VEHICLE_ID = 24
SEND_INTERVAL = 0.1


def main():
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(5.0)

    world = client.get_world()
    vehicle = world.get_actor(VEHICLE_ID)

    if vehicle is None:
        print("[CARLA] Vehicle not found")
        return

    udp = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    print("[CARLA] Sender started")
    print(f"[CARLA] Vehicle ID: {vehicle.id}")

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

            message = f"CARLA_SPEED={speed_kmh:.2f}"

            udp.sendto(
                message.encode("utf-8"),
                (RASPBERRY_PI_IP, UDP_PORT),
            )

            print(f"[CARLA] UDP SENT: {message}")

            time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        print("[CARLA] Sender stopped by user")

    except (RuntimeError, OSError) as error:
        print(f"[CARLA] Sender error: {error}")

    finally:
        udp.close()
        print("[CARLA] UDP socket closed")


if __name__ == "__main__":
    main()