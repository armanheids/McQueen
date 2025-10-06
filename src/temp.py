import carla
import random
import time
import numpy as np

def main():
    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    # Get the world
    world = client.get_world()
    
    # Get the spectator (camera view)
    spectator = world.get_spectator()
    
    # Get the blueprint library
    blueprint_library = world.get_blueprint_library()
    
    # Find a straight road spawn point
    spawn_points = world.get_map().get_spawn_points()
    
    # Choose a spawn point (you can filter for straight roads)
    vehicle_spawn_point = spawn_points[0]  # Try different indices if needed
    
    # Spawn a vehicle
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    vehicle = world.spawn_actor(vehicle_bp, vehicle_spawn_point)
    print(f"Spawned vehicle at {vehicle_spawn_point.location}")
    
    # Add a camera sensor to the front of the vehicle
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    
    # Camera transform (front of vehicle)
    camera_transform = carla.Transform(
        carla.Location(x=2.5, y=0.0, z=1.5),  # Front of car
        carla.Rotation(pitch=0, yaw=0, roll=0)
    )
    
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    print("Camera attached to vehicle")
    
    # Variable to store camera data
    camera_data = {'frame': None}
    
    # Camera callback function
    def process_image(image):
        """Process camera images and detect pedestrians"""
        # Convert image to numpy array
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        
        camera_data['frame'] = array
        
        # Simple analysis: check if pedestrian is in view
        # In a real scenario, you would use object detection here
        print(f"[Camera Analysis] Frame captured at {image.frame}")
    
    # Start listening to camera
    camera.listen(process_image)
    
    # Spawn a pedestrian ahead on the straight road
    pedestrian_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
    
    # Calculate pedestrian spawn location (straight ahead)
    forward_vector = vehicle_spawn_point.get_forward_vector()
    pedestrian_location = carla.Location(
        x=vehicle_spawn_point.location.x + forward_vector.x * 40,
        y=vehicle_spawn_point.location.y + forward_vector.y * 40,
        z=vehicle_spawn_point.location.z + 1.0
    )
    pedestrian_transform = carla.Transform(pedestrian_location)
    
    pedestrian = world.spawn_actor(pedestrian_bp, pedestrian_transform)
    print(f"Spawned pedestrian at {pedestrian_location}")
    
    # Make the pedestrian stand still
    pedestrian_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    pedestrian_controller = world.spawn_actor(pedestrian_controller_bp, carla.Transform(), pedestrian)
    pedestrian_controller.start()
    pedestrian_controller.set_max_speed(0)  # Pedestrian stands still
    
    # Set the vehicle to move forward
    vehicle.set_autopilot(False)
    
    print("Vehicle is moving toward pedestrian on straight road...")
    print("Camera is analyzing front view...")
    print("Press Ctrl+C to stop")
    
    try:
        # Keep the simulation running
        while True:
            # Apply throttle to vehicle (straight line)
            vehicle.apply_control(carla.VehicleControl(throttle=0.3, steer=0.0))
            
            # Update spectator camera to follow the vehicle (NEW METHOD)
            transform = carla.Transform(
                vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)),
                vehicle.get_transform().rotation
            )
            spectator.set_transform(transform)
            
            # Calculate distance between vehicle and pedestrian
            vehicle_location = vehicle.get_location()
            pedestrian_location = pedestrian.get_location()
            distance = vehicle_location.distance(pedestrian_location)
            
            # Analyze what's in front of the car
            if distance < 30:
                print(f"[Vehicle Analysis] Pedestrian detected! Distance: {distance:.2f} meters")
                
                # Calculate if pedestrian is in front (angle check)
                vehicle_forward = vehicle.get_transform().get_forward_vector()
                to_pedestrian = pedestrian_location - vehicle_location
                to_pedestrian_normalized = carla.Vector3D(
                    x=to_pedestrian.x,
                    y=to_pedestrian.y,
                    z=to_pedestrian.z
                )
                
                # Dot product to check if pedestrian is ahead
                dot = (vehicle_forward.x * to_pedestrian.x + 
                       vehicle_forward.y * to_pedestrian.y)
                
                if dot > 0:
                    print(f"[Vehicle Analysis] Pedestrian is DIRECTLY AHEAD!")
            
            if distance < 5:
                print("⚠️ WARNING: COLLISION IMMINENT!")
                # Optional: apply brakes
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                time.sleep(2)
                break
                
            time.sleep(0.005)  # Changed from 0.1 to 0.005 for smoother camera
                
    except KeyboardInterrupt:
        print("\nStopping simulation...")
    
    finally:
        # Clean up
        print("Cleaning up actors...")
        camera.stop()
        camera.destroy()
        if pedestrian_controller is not None:
            pedestrian_controller.stop()
            pedestrian_controller.destroy()
        if pedestrian is not None:
            pedestrian.destroy()
        if vehicle is not None:
            vehicle.destroy()
        print("Actors destroyed")

if __name__ == '__main__':
    main()
