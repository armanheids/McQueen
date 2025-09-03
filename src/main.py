# src/main.py
import carla
import random
import time
import os


def main():
    # Connection parameters
    host = os.getenv("CARLA_HOST", "localhost")
    port = int(os.getenv("CARLA_PORT", "2000"))

    print(f"Connecting to CARLA server at {host}:{port}")

    try:
        # Connect to CARLA server
        client = carla.Client(host, port)
        client.set_timeout(10.0)

        # Get world
        world = client.get_world()
        print("Connected to CARLA world successfully!")

        # Get blueprint library
        blueprint_library = world.get_blueprint_library()

        # Spawn a vehicle
        vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = random.choice(spawn_points)

        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"Spawned vehicle: {vehicle.type_id}")

        # Enable autopilot
        vehicle.set_autopilot(True)

        # Run simulation for 30 seconds
        print("Running simulation for 30 seconds...")
        for i in range(300):  # 30 seconds at 10 FPS
            # Get vehicle location
            location = vehicle.get_location()
            velocity = vehicle.get_velocity()
            speed = 3.6 * velocity.length()  # Convert to km/h

            print(
                f"Step {i}: Position({location.x:.2f}, {location.y:.2f}) Speed: {speed:.2f} km/h"
            )

            time.sleep(0.1)

        print("Simulation completed!")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Clean up
        if "vehicle" in locals():
            vehicle.destroy()
            print("Vehicle destroyed")


if __name__ == "__main__":
    main()
