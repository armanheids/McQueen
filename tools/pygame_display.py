"""
Pygame Display Module for CARLA Simulation Visualization (Rigid Camera)

Provides the PygameDisplay class to manage a Pygame window and render the view 
from an RGB camera sensor rigidly attached to a vehicle in CARLA.
Handles basic Pygame events like closing the window.
"""
import carla
import pygame
import numpy as np
import weakref

# Constants for camera positioning (relative offset for Rigid attachment)
# Adjust these for different fixed views relative to the vehicle
DISTANCE_BACK = 15.0 
HEIGHT_UP = 6.0      
RELATIVE_PITCH = -20.0 # Pitch angle (degrees) for the camera

class PygameDisplay:
    """
    Manages a Pygame window for displaying a CARLA RGB camera feed.

    The camera is rigidly attached to the specified vehicle actor.
    """

    def __init__(self, world, vehicle, 
                 display_size=(1280, 720), gamma=2.2):
        """
        Initializes Pygame, the display window, and the rigidly attached camera sensor.

        Args:
            world: The carla.World object.
            vehicle: The ego vehicle actor (carla.Vehicle) to attach the camera to.
            display_size (tuple): The desired (width, height) of the Pygame window.
            gamma (float): Gamma correction value for the RGB camera sensor.

        Raises:
            TypeError: If 'vehicle' is not a carla.Vehicle.
            ValueError: If 'world' is None.
            ImportError: If Pygame is not installed.
            RuntimeError: If Pygame fails to initialize.
        """
        if not isinstance(vehicle, carla.Vehicle):
             raise TypeError("vehicle argument must be a carla.Vehicle")
        if world is None:
             raise ValueError("world argument cannot be None")
             
        self.world = world
        self.vehicle = vehicle # Keep reference for attachment
        self.display_size = display_size
        self.gamma = gamma
        
        self.surface = None
        self.camera_sensor = None
        self.display = None

        # Initialize Pygame
        try:
            pygame.init()
            pygame.font.init() # Explicitly init font module if needed later
            self.display = pygame.display.set_mode(
                self.display_size,
                pygame.HWSURFACE | pygame.DOUBLEBUF
            )
            pygame.display.set_caption('CARLA Ego Vehicle View (Rigid Cam)')
            print(f"Pygame display initialized ({display_size[0]}x{display_size[1]}).")
        except pygame.error as e:
             self.destroy() # Clean up if initialization fails
             raise RuntimeError(f"Failed to initialize Pygame: {e}") from e

        self._setup_camera()

    def _setup_camera(self):
        """Creates, configures, and attaches the RGB camera sensor."""
        try:
            bp_library = self.world.get_blueprint_library()
            camera_bp = bp_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', str(self.display_size[0]))
            camera_bp.set_attribute('image_size_y', str(self.display_size[1]))
            if camera_bp.has_attribute('gamma'):
                camera_bp.set_attribute('gamma', str(self.gamma))

            # Define the fixed relative transform for Rigid attachment
            # Camera maintains this offset from the vehicle's origin
            relative_transform = carla.Transform(
                carla.Location(x=-DISTANCE_BACK, z=HEIGHT_UP),
                carla.Rotation(pitch=RELATIVE_PITCH) # Fixed pitch down
            ) 
            
            self.camera_sensor = self.world.spawn_actor(
                camera_bp,
                relative_transform, 
                attach_to=self.vehicle,
                attachment_type=carla.AttachmentType.Rigid # Use Rigid attachment
            )
            
            # Use weak reference in callback to avoid circular reference
            weak_self = weakref.ref(self)
            self.camera_sensor.listen(lambda image: PygameDisplay._parse_image(weak_self, image))
            print("RGB Camera sensor spawned with Rigid attachment and listening.")
            
        except Exception as e:
            # Clean up if camera setup fails
            self.destroy()
            print(f"Error setting up camera: {e}")
            # Re-raise maybe? Or handle based on application needs
            raise RuntimeError(f"Failed to set up CARLA camera: {e}") from e

    @staticmethod
    def _parse_image(weak_self, image):
        """
        Static method to process raw CARLA image data into a Pygame surface.
        Called asynchronously by the camera sensor listener.
        """
        self = weak_self() # Get the instance from the weak reference
        if not self or self.display is None: 
             # Instance or display might already be destroyed
             return 
        try:
            # Convert BGRA raw data to an RGB numpy array
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4)) # BGRA
            array = array[:, :, :3]  # Slice off alpha channel -> BGR
            array = array[:, :, ::-1] # Convert BGR to RGB
            # Create Pygame surface from the RGB numpy array
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        except Exception as e:
             print(f"Error parsing camera image: {e}")
             self.surface = None # Ensure surface is None on error

    def render(self):
        """Renders the current camera surface onto the Pygame display."""
        if self.surface is not None and self.display is not None: 
             try:
                 self.display.blit(self.surface, (0, 0))
                 pygame.display.flip()
             except pygame.error as e:
                 print(f"Error rendering Pygame surface: {e}") 
                 # Consider stopping simulation or handling error differently
                 self.display = None # Stop trying to render

    def parse_events(self):
        """
        Handles Pygame events (QUIT, ESC key).

        Returns:
            bool: True if a quit event was detected, False otherwise.
        """
        if self.display is None: 
            return True # Assume quit if display is gone
            
        quit_requested = False
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    quit_requested = True
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_ESCAPE:
                        quit_requested = True
            return quit_requested
        except pygame.error as e:
             print(f"Error processing Pygame events: {e}")
             return True # Assume quit on error

    def destroy(self):
        """Stops the camera listener, destroys the sensor actor, and quits Pygame."""
        print("Attempting to destroy Pygame display and camera sensor...")
        
        # Safely destroy the CARLA sensor actor
        if self.camera_sensor is not None:
            sensor = self.camera_sensor # Local variable for clarity
            self.camera_sensor = None # Prevent further use
            if sensor.is_listening:
                try:
                    sensor.stop()
                    print("Camera sensor listener stopped.")
                except Exception as e:
                     print(f"Error stopping camera sensor listener: {e}")
            
            # Check if actor is still alive in the simulation before destroying
            # Note: Requires access to the actor's world state, might need tick
            if sensor.is_alive: 
                try:
                    actor_destroyed = sensor.destroy()
                    if actor_destroyed:
                        print("Camera sensor actor destroyed successfully.")
                    else:
                        # This might happen if it was already destroyed elsewhere
                        print("Warning: Camera sensor actor destroy() returned False.") 
                except Exception as e:
                     # Catch potential RPC errors if server connection is lost
                     print(f"Error destroying camera sensor actor: {e}")
            else:
                 print("Camera sensor actor was already destroyed or invalid.")

        # Quit Pygame if it was initialized
        if pygame.get_init(): 
            try:
                pygame.quit()
                print("Pygame quit successfully.")
            except pygame.error as e:
                 print(f"Error quitting Pygame: {e}")
        
        # Clear references
        self.display = None
        self.surface = None 