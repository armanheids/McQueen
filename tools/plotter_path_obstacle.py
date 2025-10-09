"""
Matplotlib Plotter for CARLA Trajectory, Speed, and Obstacles (plotter_path_obstacle.py)

This module provides a `Plotter` class using Matplotlib to visualize 
CARLA simulation data in real-time. It displays two subplots:
- Top subplot: Shows the vehicle's X-Y trajectory, the predefined waypoint path, 
               and the current position of a detected obstacle.
- Bottom subplot: Shows the vehicle's current speed and the desired speed over time.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

class Plotter:
    """
    Handles the creation and updating of Matplotlib plots for trajectory and speed,
    including an optional obstacle marker.
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
        self.line_trajectory = None 
        self.line_waypoints = None  
        self.line_obstacle = None # Line object for the obstacle marker
        
        # Lines for the bottom (speed) plot
        self.line_speed = None
        self.line_desired_speed = None 
        
        # Data storage
        self.x_data = [] 
        self.y_data = [] 
        self.waypoints_x = [] 
        self.waypoints_y = [] 
        self.obstacle_x_data = [] # Obstacle X coordinate (list with 0 or 1 element)
        self.obstacle_y_data = [] # Obstacle Y coordinate (list with 0 or 1 element)
        
        self.time_data = []
        self.speed_data = []
        self.desired_speed_data = [] 
        
        self.last_plot_time = 0.0
        self.update_interval = update_interval 
        
        self.is_initialized = False
        
        # Pre-process waypoints if provided
        if waypoints:
            try:
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
            index (int): The index of the axis in self.ax array.
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
            RuntimeError: If the axes array `self.ax` is not initialized.
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
        # Set aspect ratio for the trajectory plot
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
        # Create figure and axes with equal height ratios
        self.fig, self.ax = plt.subplots(
            nrows=2, 
            ncols=1, 
            figsize=(8, 9), # Adjust figure size as needed
            # Ensure subplots have equal height
            gridspec_kw={'height_ratios': [1, 1]} 
        ) 
        # Adjust spacing between subplots
        plt.subplots_adjust(hspace=0.35) 

        # --- Configure Top Plot (X-Y Trajectory) ---
        # Calculate appropriate limits based on waypoints, or use defaults
        if self.waypoints_x and self.waypoints_y:
             padding = 10.0 # Add some padding around the waypoints
             xlim = [min(self.waypoints_x) - padding, max(self.waypoints_x) + padding]
             ylim = [min(self.waypoints_y) - padding, max(self.waypoints_y) + padding]
             # Determine tick spacing based on the range
             x_range = xlim[1] - xlim[0]
             y_range = ylim[1] - ylim[0]
             locator_base = max(5.0, round(max(x_range, y_range) / 8.0)) # Ensure reasonable tick spacing
        else:
             # Default limits if no waypoints provided
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
        # Initialize the marker for the obstacle
        self.line_obstacle, = ax0.plot([], [], 'X', color='magenta', markersize=10, label='Obstacle') # Use 'X' marker
        ax0.legend(loc='best') # Enable legend for the trajectory plot

        # --- Configure Bottom Plot (Speed vs. Time) ---
        ax1 = self._configure_axis(1, "Vehicle Speed", "Time (s)", "Speed (km/h)", 
                                   xlim=[0.0, 20.0], ylim=[0.0, 80.0], # Initial time and speed limits
                                   xlocator_base=4.0, ylocator_base=10.0) # Adjust tick spacing
        self.line_speed, = ax1.plot([], [], color='red', linewidth=2, label='Current Speed') 
        self.line_desired_speed, = ax1.plot([], [], color='blue', linewidth=2, linestyle='--', label='Desired Speed') 
        ax1.legend(loc='best') # Enable legend for the speed plot
        
        # Set a window title
        if self.fig and self.fig.canvas.manager is not None:
             self.fig.canvas.manager.set_window_title('CARLA Simulation Plots') 
        else:
             print("Warning: Could not get canvas manager to set window title.")
             
        plt.show(block=False) # Show plot without blocking
        # A small pause helps ensure the window renders before the first update
        plt.pause(0.1) 
        
        self.is_initialized = True
        print("Plot window initialized.")

    def update_plot(self, current_sim_time_sec, x, y, 
                    speed, v_desired, obstacle_coords=None): 
        """
        Updates the plot data and redraws the plots efficiently.

        Args:
            current_sim_time_sec (float): Current simulation time in seconds.
            x (float): Current vehicle X coordinate.
            y (float): Current vehicle Y coordinate.
            speed (float): Current vehicle speed (e.g., in km/h).
            v_desired (float): Current desired speed (e.g., in km/h).
            obstacle_coords (tuple, optional): (x, y) coordinates of the obstacle, 
                                               or None if no obstacle is detected. Defaults to None.
        """
        if not self.is_initialized or self.fig is None or self.ax is None:
            # print("Debug: Plot not initialized or figure closed, skipping update.")
            return

        # Append new data points
        self.x_data.append(x)
        self.y_data.append(y)
        self.time_data.append(current_sim_time_sec)
        self.speed_data.append(speed)
        self.desired_speed_data.append(v_desired) 
        
        # Update obstacle data (replace previous point or clear)
        if obstacle_coords:
            self.obstacle_x_data = [obstacle_coords[0]] 
            self.obstacle_y_data = [obstacle_coords[1]] 
        else:
            # Clear data if no obstacle
            self.obstacle_x_data = [] 
            self.obstacle_y_data = [] 
        
        # Throttle plot updates to the specified interval
        now = time.time()
        if now - self.last_plot_time < self.update_interval:
            return
        self.last_plot_time = now

        try:
            # Update the data for each line object
            if self.line_trajectory: self.line_trajectory.set_data(self.x_data, self.y_data) 
            if self.line_speed: self.line_speed.set_data(self.time_data, self.speed_data) 
            if self.line_desired_speed: self.line_desired_speed.set_data(self.time_data, self.desired_speed_data) 
            if self.line_obstacle: self.line_obstacle.set_data(self.obstacle_x_data, self.obstacle_y_data)

            # --- Dynamic Axis Rescaling ---
            # Optional: Rescale trajectory plot axes dynamically (can impact performance)
            # self.ax[0].relim() 
            # self.ax[0].autoscale_view() 

            # Rescale time axis (X) for speed plot
            if self.time_data:
                max_time = self.time_data[-1]
                # Set x-limit to be slightly larger than max time, minimum 20s
                current_xlim_time = max(20.0, max_time + 1.0) 
                self.ax[1].set_xlim(0, current_xlim_time)
            
            # Rescale speed axis (Y) for speed plot
            if self.speed_data or self.desired_speed_data:
                all_speeds = self.speed_data + self.desired_speed_data
                min_s = 0.0 # Assume minimum speed is 0
                max_s = max(all_speeds) if all_speeds else 80.0 # Default max if no data
                # Add padding, ensure min is 0
                padding_s = max(5.0, (max_s - min_s) * 0.1) 
                self.ax[1].set_ylim(min_s, max_s + padding_s) 
            # -----------------------------

            # Request redraw of the canvas (efficient update)
            self.fig.canvas.draw_idle()
            
            # Crucial pause for interactive mode to process events and draw
            plt.pause(0.001) 

        except Exception as e:
            # Handle errors gracefully, e.g., if the plot window was closed by the user
            print(f"Error updating plot (window might be closed): {e}") 
            self.cleanup_plot() # Clean up resources if update fails


    def cleanup_plot(self):
       """Closes the Matplotlib plot window and resets state."""
       if self.fig is not None:
            try:
                print("Closing plot window...")
                plt.close(self.fig)
            except Exception as e:
                print(f"Error closing plot window: {e}") 
            finally:
                 # Ensure state is reset even if closing fails
                 self.is_initialized = False
                 self.fig = None 
                 self.ax = None
                 # Clear line references
                 self.line_trajectory = None
                 self.line_waypoints = None
                 self.line_obstacle = None
                 self.line_speed = None
                 self.line_desired_speed = None
                 print("Plot resources released.")
                 
       # Clear data lists
       self.x_data.clear()
       self.y_data.clear()
       self.time_data.clear()
       self.speed_data.clear()
       self.desired_speed_data.clear()
       self.obstacle_x_data.clear()
       self.obstacle_y_data.clear() 