#!/usr/bin/env python
"""
CARLA Waypoint Following with Obstacle Avoidance (sd_7/__main__.py)

This script demonstrates waypoint following in CARLA using PID for speed control
and the Stanley method for steering control. It includes basic obstacle detection
using a CARLA obstacle sensor and implements emergency braking.

Features:
- Spawns an ego vehicle (Tesla Model 3) and a static obstacle vehicle (Audi TT).
- Defines a complex path using a list of WAYPOINTS: [x, y, desired_speed_kmh].
- Uses a PID controller (`tools.pid_controller.PIDController`) for speed control.
- Uses the Stanley steering controller (`calculate_stanley_steering_darpa`) for steering.
- Attaches an 'obstacle' sensor to the ego vehicle to detect other vehicles/walkers.
- Implements emergency braking (`obstacle_detected_state`) when the sensor detects an obstacle.
- Switches target waypoints based on proximity (WAYPOINT_THRESHOLD).
- Includes plotting (`tools.plotter_path_obstacle.Plotter`) showing trajectory, speed, and obstacle position.
- Includes Pygame display (`tools.pygame_display.PygameDisplay`) for visualizing the vehicle's camera view.
- Sets a fixed initial spectator viewpoint.
- Cleans up spawned actors (ego vehicle, obstacle, sensor) and resources on exit.

Dependencies:
- tools.plotter_path_obstacle (Plotter class with obstacle support)
- tools.pygame_display (PygameDisplay class)
- tools.pid_controller (PIDController class)
"""

import carla
import numpy as np 
import time
import math
# Removed typing import

# Import helper classes using the specified format
from tools.plotter_path_obstacle import Plotter       
from tools.pygame_display import PygameDisplay 
from tools.pid_controller import PIDController 

# --- Global state for obstacle detection ---
# This flag is set to True by the obstacle sensor callback when an obstacle is detected
obstacle_detected_state = False 
# Store the location of the detected obstacle for plotting
obstacle_plot_location = None

# --- Motion Configuration ---
# Waypoints format: [x_coordinate, y_coordinate, desired_speed_kmh]
WAYPOINTS = [
    # Start position (spawn point, speed is for the first segment)
    [-64.0, 24.5, 40.0], 
    # 1. Drive towards the braking/turning point
    [0.0,   24.5, 40.0],   # Maintain 40 km/h (Obstacle placed around x=10)
    # 2. Start braking and turning/lane change (after potential obstacle)
    [27.0,  28.0, 20.0],   # Reduce speed to 20 km/h
    # 3. Slow driving after lane change
    [37.0,  32.0, 15.0],   # Drive slowly (15 km/h)
    # 4. Continue slow driving
    [40.0,  40.0, 15.0],   # Maintain 15 km/h
    # 5. Point before crosswalk
    [40.7, 53.4, 15.0],   # Maintain 15 km/h
    # 6. Point representing stop or slow down (adjust speed as needed)
    [40.7, 53.4, 15.0],   # Maintain 15 km/h (or set to 0 for a stop)
    # 7. Start moving and turning after stop/slowdown
    [38.8, 61.6, 20.0],   # Accelerate to 20 km/h for the turn
    # 8. Continue turn
    [34.6, 63.0, 20.0],   # Maintain 20 km/h
    # 9. Straighten out
    [25.0, 66.0, 30.0],   # Accelerate to 30 km/h
    # 10. Point before braking for final stop
    [-9.8, 66.1, 30.0],   # Maintain 30 km/h up to here
    # 11. Approach final stop
    [-23.0, 66.0, 20.0],  # Reduce speed to 20 km/h
    # 12. Final stop
    [-26.0, 66.0, 0.0]    # Target speed 0
]

# Index of the current target waypoint in the WAYPOINTS list
target_waypoint_id = 1 
# Allowed distance deviation (in meters) to switch to the next waypoint
WAYPOINT_THRESHOLD = 3.0 

# Simulation start time (for plotting time axis)
simulation_start_time = 0.0 

# --- Vehicle Data Functions ---
def get_location(vehicle):
   """Returns the current location of the vehicle."""
   return vehicle.get_location()

def get_speed_kmh(vehicle):
   """Returns the current speed of the vehicle in km/h."""
   velocity_vector = vehicle.get_velocity()
   speed_meters_per_second = np.linalg.norm([velocity_vector.x, velocity_vector.y, velocity_vector.z]) 
   return 3.6 * speed_meters_per_second 

def get_speed_mps(vehicle):
   """Returns the current speed of the vehicle in m/s."""
   velocity_vector = vehicle.get_velocity()
   return np.linalg.norm([velocity_vector.x, velocity_vector.y, velocity_vector.z])

# --- Simulation Management Functions ---
def remove_previous_actors(world):
    """Finds and removes actors with role 'my_car' or 'obstacle'."""
    actors_to_remove = []
    # Find vehicles with specific roles
    for actor in world.get_actors().filter('*vehicle*'):
        role = actor.attributes.get('role_name')
        if role == 'my_car' or role == 'obstacle':
            actors_to_remove.append(actor)
            
    # Find previous obstacle sensors (important to avoid multiple callbacks)
    for sensor in world.get_actors().filter('sensor.other.obstacle'):
         # Could add checks here, e.g., if parent is invalid
         actors_to_remove.append(sensor)

    count = 0
    for actor in actors_to_remove:
         print(f"  - Removing previous actor: {actor.type_id} (ID {actor.id}, Role: {actor.attributes.get('role_name', 'N/A')})")
         if actor.destroy():
             count += 1
         else:
             print(f"  - Failed to remove actor {actor.id}")
    print(f"Removed {count} previous actors/sensors.")


def update_target_waypoint(vehicle_location):
    """
    Checks if the current target waypoint is reached and switches to the next one.
    Updates the global `target_waypoint_id`.
    """
    global target_waypoint_id # Allow modification
    
    # If this is the last actual target point, do not switch further.
    if target_waypoint_id >= len(WAYPOINTS) - 1:
        return

    target_wp_data = WAYPOINTS[target_waypoint_id]
    target_loc = carla.Location(x=target_wp_data[0], y=target_wp_data[1]) 
    
    distance = vehicle_location.distance_2d(target_loc)
    
    if distance < WAYPOINT_THRESHOLD:
        reached_wp_id = target_waypoint_id
        reached_wp_coords = (target_wp_data[0], target_wp_data[1]) 
        target_waypoint_id += 1 
        
        log_message = f"Reached waypoint {reached_wp_id} (X={reached_wp_coords[0]:.1f}, Y={reached_wp_coords[1]:.1f}, dist={distance:.1f}m). "
        if target_waypoint_id < len(WAYPOINTS):
            new_target_wp_data = WAYPOINTS[target_waypoint_id]
            new_target_coords = (new_target_wp_data[0], new_target_wp_data[1])
            log_message += f"New target: {target_waypoint_id} (X={new_target_coords[0]:.1f}, Y={new_target_coords[1]:.1f})"
        else:
             log_message += "Final waypoint index reached."
        print(log_message)

# --- Spectator Setup ---
def set_initial_spectator_view(world):
    """Sets the spectator camera to a predefined fixed position and rotation."""
    spectator = world.get_spectator()
    # Define spectator transform (adjust coordinates as needed)
    spectator_location = carla.Location(x=33, y=27, z=5.61) # Adjusted slightly for better view
    spectator_rotation = carla.Rotation(pitch=-4.27, yaw=-170.21, roll=0.00)
    spectator_transform = carla.Transform(spectator_location, spectator_rotation)
    try:
        spectator.set_transform(spectator_transform)
        print(f"Initial spectator position set to: Loc=[{spectator_location.x:.2f}, {spectator_location.y:.2f}, {spectator_location.z:.2f}], Rot=[P:{spectator_rotation.pitch:.2f}, Y:{spectator_rotation.yaw:.2f}, R:{spectator_rotation.roll:.2f}]")
    except Exception as e:
        print(f"Error setting spectator transform: {e}")


# --- Controller Constants and Functions ---
MIN_SPEED_STANLEY = 1e-4 
STANLEY_K = 0.5          
MAX_STEER_DEG = 70.0     
MAX_STEER_RAD = math.radians(MAX_STEER_DEG) 

def calculate_stanley_steering_darpa(vehicle, waypoints, current_target_id):
    """Calculates the steering angle using the Stanley method (DARPA version - restored logic)."""
    
    k = STANLEY_K # Use the constant

    # Check if there is a previous waypoint
    if current_target_id < 1:
        return 0.0

    # Vehicle data
    vehicle_location = vehicle.get_location()
    vehicle_transform = vehicle.get_transform()
    vehicle_yaw_rad = math.radians(vehicle_transform.rotation.yaw)
    vehicle_speed_mps = get_speed_mps(vehicle)

    # Previous and target waypoints
    wp_prev = waypoints[current_target_id - 1]
    wp_target = waypoints[current_target_id]

    # Path segment angle
    yaw_path = np.arctan2(wp_target[1] - wp_prev[1], wp_target[0] - wp_prev[0])

    # Heading error
    yaw_diff = yaw_path - vehicle_yaw_rad
    # Normalize [-pi, pi]
    if yaw_diff > np.pi:
        yaw_diff -= 2 * np.pi
    if yaw_diff < -np.pi:
        yaw_diff += 2 * np.pi

    # Cross-track error (CTE) calculation
    # Using the line equation method from the previous version
    diff_x = wp_target[0] - wp_prev[0]
    diff_y = wp_target[1] - wp_prev[1]
    if abs(diff_x) < 1e-6: 
        crosstrack_error = vehicle_location.x - wp_prev[0]
    else:
        slope = diff_y / diff_x
        a = -slope
        b = 1.0
        c = (slope * wp_prev[0]) - wp_prev[1]
        crosstrack_error = (a * vehicle_location.x + b * vehicle_location.y + c) / np.sqrt(a ** 2 + b ** 2)

    # Determine CTE sign (as in the previous version)
    yaw_cross_track = np.arctan2(vehicle_location.y - wp_prev[1], vehicle_location.x - wp_prev[0])
    yaw_path2ct = yaw_path - yaw_cross_track
    # Normalize [-pi, pi]
    if yaw_path2ct > np.pi:
        yaw_path2ct -= 2 * np.pi
    if yaw_path2ct < -np.pi:
        yaw_path2ct += 2 * np.pi
        
    # Apply sign to CTE
    if yaw_path2ct > 0:
        crosstrack_error = abs(crosstrack_error)
    else:
        crosstrack_error = -abs(crosstrack_error)

    # Steering component from CTE
    safe_speed = max(vehicle_speed_mps, MIN_SPEED_STANLEY) 
    yaw_diff_crosstrack = np.arctan(k * crosstrack_error / safe_speed)

    # Expected final steering angle (in radians)
    steer_expect = yaw_diff + yaw_diff_crosstrack
    # Normalize [-pi, pi]
    if steer_expect > np.pi:
        steer_expect -= 2 * np.pi
    if steer_expect < -np.pi:
        steer_expect += 2 * np.pi
        
    # Limit steering angle in radians (using MAX_STEER_DEG for conversion)
    max_steer_rad = math.radians(MAX_STEER_DEG) # Convert max degree to radians
    steer_expect = min(max_steer_rad, steer_expect)
    steer_expect = max(-max_steer_rad, steer_expect)

    # Convert to CARLA control signal [-1, 1] by normalizing with max_steer_rad
    steer_carla = steer_expect / max_steer_rad
    steer_carla = np.clip(steer_carla, -1.0, 1.0) # Ensure it's within bounds

    return steer_carla
# --- End of Restored Stanley Controller Logic --


# --- Obstacle Sensor Callback ---
def obstacle_callback(event):
    """
    Handles events from the 'sensor.other.obstacle'.
    Sets the global `obstacle_detected_state` flag and stores obstacle location.
    """
    global obstacle_detected_state, obstacle_plot_location # Declare modification of global variables
    
    # Check the type of the actor the sensor detected
    actor_type = event.other_actor.type_id
    # Filter for relevant obstacles (vehicles and pedestrians)
    if 'vehicle' in actor_type or 'walker' in actor_type:
        if not obstacle_detected_state: # Print only on first detection
             print(f"!!! OBSTACLE DETECTED: {actor_type} at distance {event.distance:.2f} m !!!")
        # Set the global state flag to trigger emergency braking
        obstacle_detected_state = True 
        # Store obstacle location for plotting
        obs_loc = event.other_actor.get_location()
        obstacle_plot_location = (obs_loc.x, obs_loc.y)

def main():
    """Main execution function."""
    # Declare modification of global variables used within main
    global simulation_start_time, target_waypoint_id, obstacle_detected_state, obstacle_plot_location 
    
    client = None
    world = None
    vehicle = None         
    obstacle_vehicle = None # The static obstacle we spawn
    plotter = None 
    pygame_display = None 
    pid_controller = None 
    obstacle_sensor = None # The sensor actor attached to the ego vehicle

    try:
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0) 
        print("Connecting to CARLA server...")
        world = client.get_world()
        print(f"Connected to world: {world.get_map().name}")
        
        # Remove actors from previous runs
        remove_previous_actors(world)
        
        # --- Spawn Ego Vehicle ---
        map_spawn_points = world.get_map().get_spawn_points()
        if not map_spawn_points:
            print("Error: No spawn points found on the map!")
            return
        start_waypoint = WAYPOINTS[0]
        start_location = carla.Location(x=start_waypoint[0], y=start_waypoint[1], z=0.5) # Use z=0.5 
        spawn_point = min(map_spawn_points, key=lambda sp: sp.location.distance(start_location))
        spawn_point.location = start_location
        spawn_point.rotation = carla.Rotation(yaw=0) # Force initial yaw
        vehicle_bp_library = world.get_blueprint_library()
        vehicle_bp = vehicle_bp_library.filter('vehicle.tesla.model3')[0]
        vehicle_bp.set_attribute('role_name', 'my_car') 
        print("Attempting to spawn ego vehicle...")
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle is None:
             print(f"Error: Failed to spawn ego vehicle at {spawn_point.location}")
             return
        print(f"Ego vehicle {vehicle.type_id} (ID {vehicle.id}) spawned.")
        # -----------------------

        # --- Spawn Obstacle Vehicle ---
        try:
            obstacle_bp_library = world.get_blueprint_library()
            obstacle_bp = obstacle_bp_library.find('vehicle.audi.tt') 
            obstacle_bp.set_attribute('role_name', 'obstacle') # Assign role for easy cleanup
            # Place it on the path, e.g., on the first straight section before the turn
            obstacle_location = carla.Location(x=10.0, y=24.5, z=0.1) 
            obstacle_transform = carla.Transform(obstacle_location, carla.Rotation(yaw=0)) # Align with road
            
            print("Attempting to spawn obstacle vehicle...")
            obstacle_vehicle = world.try_spawn_actor(obstacle_bp, obstacle_transform)
            if obstacle_vehicle is not None:
                print(f"Obstacle vehicle {obstacle_vehicle.type_id} (ID {obstacle_vehicle.id}) spawned at {obstacle_location}.")
                # Keep it stationary by disabling physics simulation
                obstacle_vehicle.set_simulate_physics(False) 
            else:
                print(f"Warning: Failed to spawn obstacle vehicle at {obstacle_location}")
        except IndexError:
            print("Warning: Could not find blueprint 'vehicle.audi.tt'. Obstacle not spawned.")
        except Exception as e:
             print(f"Error spawning obstacle vehicle: {e}")
        # ----------------------------

        # --- Setup and Spawn Obstacle Sensor ---
        try:
            obs_bp_library = world.get_blueprint_library()
            obs_bp = obs_bp_library.find('sensor.other.obstacle')
            obs_bp.set_attribute("distance", "15")        # Detection distance in meters
            obs_bp.set_attribute("sensor_tick", "0.1")    # Check frequency (seconds)

            # Attach sensor relative to the vehicle's front center
            # Adjust x/z offset for desired sensor position
            sensor_transform = carla.Transform(carla.Location(x=2.5, z=0.8)) 
            obstacle_sensor = world.spawn_actor(obs_bp, sensor_transform , attach_to=vehicle)
            
            # Register the callback function to handle sensor events
            obstacle_sensor.listen(obstacle_callback) 
            print("Obstacle sensor attached and listening.")
        except IndexError:
             print("Warning: Could not find blueprint 'sensor.other.obstacle'. Obstacle detection disabled.")
             obstacle_sensor = None
        except Exception as e:
             print(f"Error setting up obstacle sensor: {e}")
             obstacle_sensor = None
        # -------------------------------------

        # Set spectator view AFTER spawning vehicle
        set_initial_spectator_view(world) 

        # --- Initialize Plotter, Pygame, and PID Controller ---
        print("Initializing Plotter...")
        # Use the plotter that supports obstacles
        plotter = Plotter(waypoints=WAYPOINTS) 
        plotter.init_plot() 
        print("Plotter initialized.")
        
        print("Initializing Pygame display...")
        pygame_display = PygameDisplay(world, vehicle) 
        print("Pygame display initialized.")
        
        print("Initializing PID controller...")
        pid_controller = PIDController() 
        print("PID controller initialized.")
        
        # Record simulation start time and reset state variables
        simulation_start_time = time.time() 
        target_waypoint_id = 1 
        obstacle_detected_state = False 
        obstacle_plot_location = None

        # --- Main Simulation Loop ---
        print("Simulation running. Following waypoints with obstacle detection. Press ESC or close window to stop.")
        while True:
            current_loop_time = time.time()

            # Check for Pygame window events (close/ESC)
            if pygame_display.parse_events():
                print("Quit requested via Pygame window.")
                break 

            # Wait for the next simulation tick
            world.wait_for_tick() 
            
            # --- Data Collection ---
            current_transform = vehicle.get_transform() 
            current_location = current_transform.location 
            current_speed_kmh = get_speed_kmh(vehicle) 
            current_sim_time_sec = current_loop_time - simulation_start_time 

            # --- Get obstacle location for plotting (if static obstacle exists) ---
            # Note: The dynamic obstacle location comes from the callback now
            # static_obstacle_loc_data = None # Removed redundant static obstacle check here
            # if obstacle_vehicle is not None and obstacle_vehicle.is_alive:
            #      try:
            #          obs_loc = obstacle_vehicle.get_location()
            #          static_obstacle_loc_data = (obs_loc.x, obs_loc.y)
            #      except Exception:
            #          print("Warning: Could not get static obstacle location.")
            #          obstacle_vehicle = None 
            
            # Use the location updated by the obstacle sensor callback for plotting
            plot_obstacle_coords = obstacle_plot_location 
            # ------------------------------------------------------

            # --- Update Visualizations ---
            # Update Plotter (with obstacle info)
            if plotter is not None and plotter.is_initialized:
                target_speed_for_plot = WAYPOINTS[target_waypoint_id][2] if 0 <= target_waypoint_id < len(WAYPOINTS) else 0.0
                try:
                    # Pass current obstacle coordinates (from callback) to plotter
                    plotter.update_plot(current_sim_time_sec, 
                                        current_location.x, 
                                        current_location.y, 
                                        current_speed_kmh, 
                                        target_speed_for_plot,
                                        obstacle_coords=plot_obstacle_coords) # Use the variable updated by the callback
                except Exception as plot_update_e:
                    print(f"Error updating plot (likely closed): {plot_update_e}")
                    plotter.cleanup_plot()
                    plotter = None 
            
            # Render Pygame display
            pygame_display.render()

            # --- Control Logic ---
            # 1. Update target waypoint based on proximity
            update_target_waypoint(current_location)
            
            # 2. Get desired speed for the current segment (base speed)
            v_desired = 0.0 
            if 0 <= target_waypoint_id < len(WAYPOINTS):
                v_desired = WAYPOINTS[target_waypoint_id][2]
            
            # 3. Calculate baseline longitudinal control (throttle/brake) using PID
            longitudinal_control = pid_controller.calculate_control(v_desired, current_speed_kmh)
            
            # 4. Calculate baseline lateral control (steering) using Stanley
            steering_angle = calculate_stanley_steering_darpa(vehicle, WAYPOINTS, target_waypoint_id) 
            
            # 5. Combine baseline control commands
            control = carla.VehicleControl()
            control.throttle = longitudinal_control.throttle
            control.brake = longitudinal_control.brake
            control.steer = steering_angle 
            control.hand_brake = False
            control.manual_gear_shift = False

            # If the obstacle sensor callback set the flag, override control
            if obstacle_detected_state: 
                print("  -> Obstacle detected! OVERRIDING control: EMERGENCY BRAKE.")
                control.throttle = 0.0
                control.brake = 1.0 # Apply full brake
            
            # 6. Apply final control to the ego vehicle
            vehicle.apply_control(control)
            # --- End Control Logic ---

    except KeyboardInterrupt:
        print("\nScript stopped by user (Ctrl+C).")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}") 
        import traceback
        traceback.print_exc()
    
    # --- Resource Cleanup Block ---
    finally:
        print("Starting resource cleanup...")
        # Reset global states explicitly here for clarity, although process exit usually handles it
        obstacle_detected_state = False 
        obstacle_plot_location = None 
        
        # Destroy obstacle sensor *before* the vehicle it's attached to
        if obstacle_sensor is not None and obstacle_sensor.is_alive:
            print("Destroying obstacle sensor...")
            try:
                if obstacle_sensor.is_listening: obstacle_sensor.stop() 
                if obstacle_sensor.destroy(): print("Obstacle sensor destroyed.")
                else: print("Obstacle sensor failed to destroy.")
            except Exception as e: print(f"Error destroying obstacle sensor: {e}")
        obstacle_sensor = None # Clear reference
        
        # Destroy Pygame display (handles its own camera cleanup)
        if pygame_display is not None:
            print("Destroying Pygame display...")
            pygame_display.destroy()
            print("Pygame display destroyed.")
            pygame_display = None

        # Cleanup plotter
        if plotter is not None:
             if plotter.is_initialized: # Check if initialized before cleaning
                 print("Cleaning up plotter...")
                 plotter.cleanup_plot()
                 print("Plotter cleaned up.")
        plotter = None # Correct indentation: assign None after checking/cleaning

        # Destroy obstacle vehicle
        if obstacle_vehicle is not None and obstacle_vehicle.is_alive:
            print(f"Destroying obstacle vehicle: {obstacle_vehicle.type_id} (ID {obstacle_vehicle.id})")
            if obstacle_vehicle.destroy(): print("Obstacle vehicle destroyed.")
            else: print("Obstacle vehicle failed to destroy.")
        obstacle_vehicle = None
                
        # Destroy ego vehicle (last, as sensor might be attached)
        if vehicle is not None and vehicle.is_alive:
             print(f"Destroying ego vehicle: {vehicle.type_id} (ID {vehicle.id})")
             # vehicle.set_simulate_physics(False) # Optional
             if vehicle.destroy(): print("Ego vehicle destroyed.")
             else: print("Ego vehicle failed to destroy.")
        vehicle = None
        
        print("Simulation finished.")


if __name__ == '__main__':
    main()
