"""
Matplotlib Plotter for CARLA Trajectory and Speed (plotter_path.py)

This module provides a `Plotter` class using Matplotlib to visualize 
CARLA simulation data in real-time. It displays two subplots:
- Top subplot: Shows the vehicle's X-Y trajectory and the predefined waypoint path.
- Bottom subplot: Shows the vehicle's current speed and the desired speed over time.
"""
import time
# numpy and itertools are not currently used, can be removed if not needed later
# import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
# import itertools 
# Removed typing import

class Plotter:
    """
    Handles the creation and updating of Matplotlib plots for trajectory and speed.
    """

    def __init__(self, update_interval=0.3, waypoints=None):
        """
        Initializes the Plotter.

        Args:
            update_interval (float): Minimum time in seconds between plot updates.
            waypoints (list, optional): List of waypoints [[x, y, speed], ...]. 
                                        Used to draw the reference path. Defaults to None.
        """
        self.fig = None
        self.ax = None # Will hold the array of axes [ax_trajectory, ax_speed]
        
        # Lines for the top (trajectory) plot
        self.line_trajectory = None # Vehicle's path line
        self.line_waypoints = None  # Predefined path line
        
        # Lines for the bottom (speed) plot
        self.line_speed = None       # Current speed line
        self.line_desired_speed = None # Desired speed line
        
        # Data storage for trajectory plot
        self.x_data = [] 
        self.y_data = [] 
        self.waypoints_x = [] 
        self.waypoints_y = [] 
        
        # Data storage for speed plot
        self.time_data = []
        self.speed_data = []
        self.desired_speed_data = [] 
        
        self.last_plot_time = 0.0
        self.update_interval = update_interval 
        
        self.is_initialized = False
        
        # Pre-process waypoints if provided
        if waypoints:
            try:
                # Extract only X and Y coordinates, ignoring speed or other data
                self.waypoints_x = [wp[0] for wp in waypoints]
                self.waypoints_y = [wp[1] for wp in waypoints]
            except (IndexError, TypeError) as e:
                print(f"Warning: Invalid waypoint format. Could not extract X/Y coordinates: {e}")
                self.waypoints_x = []
                self.waypoints_y = []

    def _configure_axis(self, index, title, xlabel, ylabel, 
                        xlim=None, ylim=None, 
                        xlocator_base=None, ylocator_base=None):
        """
        Helper function to configure a subplot axis.

        Args:
            index (int): The index of the axis in self.ax array (0 for trajectory, 1 for speed).
            title (str): Title for the subplot.
            xlabel (str): Label for the X-axis.
            ylabel (str): Label for the Y-axis.
            xlim (tuple, optional): X-axis limits. Defaults to None.
            ylim (tuple, optional): Y-axis limits. Defaults to None.
            xlocator_base (float, optional): Base value for X-axis major ticks. Defaults to None.
            ylocator_base (float, optional): Base value for Y-axis major ticks. Defaults to None.

        Returns:
            matplotlib.axes.Axes: The configured axis object.
            
        Raises:
            RuntimeError: If the axes array `self.ax` is not initialized before calling.
        """
        if self.ax is None:
             raise RuntimeError("Axes array self.ax is not initialized.")
        axis = self.ax[index]
        axis.set_title(title)
        axis.set_xlabel(xlabel)
        axis.set_ylabel(ylabel)
        if xlim:
            axis.set_xlim(xlim)
        if ylim:
            axis.set_ylim(ylim)
        if xlocator_base:
            axis.xaxis.set_major_locator(ticker.MultipleLocator(base=xlocator_base))
        if ylocator_base:
            axis.yaxis.set_major_locator(ticker.MultipleLocator(base=ylocator_base))
        axis.grid(True)
        # Ensure equal aspect ratio for the X-Y trajectory plot (top plot)
        if index == 0:
             axis.set_aspect('equal', adjustable='box') 
        return axis

    def init_plot(self):
        """Initializes the Matplotlib figure and axes for the plots."""
        if self.is_initialized:
            print("Plot is already initialized.")
            return

        print("Initializing plot window...")
        plt.ion() # Turn on interactive mode
        self.last_plot_time = time.time()
        
        # Create figure and axes, ensuring equal height for both subplots
        self.fig, self.ax = plt.subplots(
            nrows=2, 
            ncols=1, 
            figsize=(8, 9), # Adjusted figure size for better layout
            # Use gridspec_kw to enforce equal height ratios
            gridspec_kw={'height_ratios': [1, 1]} 
        ) 
        # Adjust vertical spacing between subplots
        plt.subplots_adjust(hspace=0.35) 

        # --- Configure Top Plot (X-Y Trajectory) ---
        # Calculate plot limits based on waypoints or use defaults
        if self.waypoints_x and self.waypoints_y:
            padding = 10.0 # Add padding around waypoints
            xlim = [min(self.waypoints_x) - padding, max(self.waypoints_x) + padding]
            ylim = [min(self.waypoints_y) - padding, max(self.waypoints_y) + padding]
            # Determine tick spacing based on the range
            x_range = xlim[1] - xlim[0]
            y_range = ylim[1] - ylim[0]
            # Calculate a reasonable base for tick marks
            locator_base = max(5.0, round(max(x_range, y_range) / 8.0)) 
        else:
            # Default limits if no waypoints are provided
            xlim = [-100.0, 100.0] 
            ylim = [0.0, 100.0]
            locator_base = 20.0

        ax0 = self._configure_axis(0, "Vehicle Trajectory", "X coordinate (m)", "Y coordinate (m)", 
                                   xlim=xlim, ylim=ylim, 
                                   xlocator_base=locator_base, ylocator_base=locator_base)
        # Draw the predefined waypoint path
        self.line_waypoints, = ax0.plot(self.waypoints_x, self.waypoints_y, '--o', color='blue', markersize=3, label='Waypoints')
        # Initialize the line for the vehicle's actual path
        self.line_trajectory, = ax0.plot([], [], color='red', linewidth=2, label='Vehicle Path')
        ax0.legend(loc='best') # Display legend

        # --- Configure Bottom Plot (Speed vs. Time) ---
        ax1 = self._configure_axis(1, "Vehicle Speed", "Time (s)", "Speed (km/h)", 
                                   xlim=[0.0, 20.0], ylim=[0.0, 80.0], # Initial time and speed limits
                                   xlocator_base=4.0, ylocator_base=10.0) # Set tick spacing
        self.line_speed, = ax1.plot([], [], color='red', linewidth=2, label='Current Speed') 
        self.line_desired_speed, = ax1.plot([], [], color='blue', linewidth=2, linestyle='--', label='Desired Speed') 
        ax1.legend(loc='best') # Display legend
        
        # Set a window title if possible
        if self.fig and self.fig.canvas.manager:
             self.fig.canvas.manager.set_window_title('CARLA Simulation Plot') 
        else:
             print("Warning: Could not get canvas manager to set window title.")
             
        plt.show(block=False) # Show plot without blocking execution
        # A small pause helps ensure the window renders before the first update call
        plt.pause(0.1) 
        
        self.is_initialized = True
        print("Plot window initialized.")

    def update_plot(self, current_sim_time_sec, x, y, speed, v_desired): 
        """
        Updates the plot data and redraws the plots efficiently.

        Args:
            current_sim_time_sec (float): Current simulation time in seconds.
            x (float): Current vehicle X coordinate.
            y (float): Current vehicle Y coordinate.
            speed (float): Current vehicle speed (e.g., in km/h).
            v_desired (float): Current desired speed (e.g., in km/h).
        """
        if not self.is_initialized or self.fig is None or self.ax is None:
            # Silently return if not initialized or plot closed
            return

        # Append new data points to lists
        self.x_data.append(x)
        self.y_data.append(y)
        self.time_data.append(current_sim_time_sec)
        self.speed_data.append(speed)
        self.desired_speed_data.append(v_desired) 

        # Throttle updates based on the defined interval
        now = time.time()
        if now - self.last_plot_time < self.update_interval:
            return # Skip update if interval hasn't passed
        self.last_plot_time = now

        try:
            # Update the data for each line object
            if self.line_trajectory: self.line_trajectory.set_data(self.x_data, self.y_data)
            if self.line_speed: self.line_speed.set_data(self.time_data, self.speed_data)
            if self.line_desired_speed: self.line_desired_speed.set_data(self.time_data, self.desired_speed_data)

            # --- Dynamic Axis Rescaling ---
            # Optional: Rescale trajectory plot axes dynamically (can impact performance)
            # self.ax[0].relim()
            # self.ax[0].autoscale_view(True, True, True)

            # Rescale time axis (X) for speed plot dynamically
            if self.time_data:
                max_time = self.time_data[-1]
                # Set x-limit to be slightly larger than max time, minimum 20s
                current_xlim_time = max(20.0, max_time + 1.0) 
                self.ax[1].set_xlim(0, current_xlim_time)
            
            # Rescale speed axis (Y) for speed plot dynamically
            if self.speed_data or self.desired_speed_data:
                all_speeds = self.speed_data + self.desired_speed_data
                min_s = 0.0 # Speed typically starts at 0
                max_s = max(all_speeds) if all_speeds else 80.0 # Default max if no data
                # Add some padding to the upper limit
                padding_s = max(5.0, (max_s - min_s) * 0.1) 
                self.ax[1].set_ylim(min_s, max_s + padding_s) 
            # -----------------------------

            # Request redraw of the canvas (efficient update)
            self.fig.canvas.draw_idle()
            
            # Crucial pause for interactive mode to process events and draw
            plt.pause(0.001) 

        except Exception as e:
            # Handle errors gracefully, e.g., if the plot window was closed manually
            print(f"Error updating plot (window might be closed): {e}") 
            self.cleanup_plot() # Attempt to clean up resources if an error occurs


    def cleanup_plot(self):
        """Closes the Matplotlib plot window and resets internal state."""
        if self.fig is not None:
            try:
                print("Closing plot window...")
                plt.close(self.fig)
            except Exception as e:
                # Log error if closing fails, but proceed with cleanup
                print(f"Error closing plot window: {e}") 
            finally:
                 # Ensure state is reset regardless of close success/failure
                 self.is_initialized = False
                 self.fig = None 
                 self.ax = None
                 # Clear references to Line2D objects
                 self.line_trajectory = None
                 self.line_waypoints = None
                 self.line_speed = None
                 self.line_desired_speed = None
                 print("Plot resources released.")
                 
        # Clear all stored data
        self.x_data.clear()
        self.y_data.clear()
        self.waypoints_x.clear()
        self.waypoints_y.clear()
        self.time_data.clear()
        self.speed_data.clear()
        self.desired_speed_data.clear() 