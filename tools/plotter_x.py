"""
Matplotlib Plotter for CARLA X-Coordinate and Speed (plotter_x.py)

This module provides a `Plotter` class using Matplotlib to visualize 
CARLA simulation data in real-time. It displays two subplots:
- Top subplot: Shows the vehicle's X-coordinate over time.
- Bottom subplot: Shows the vehicle's current speed and the desired speed over time.
"""

import time
# numpy is not currently used, can be removed
# import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

class Plotter:
    """
    Handles the creation and updating of Matplotlib plots for X-coordinate and speed.
    """

    def __init__(self, update_interval=0.3):
        """
        Initializes the Plotter.

        Args:
            update_interval (float): Minimum time in seconds between plot updates.
        """
        self.fig = None
        self.ax = None # Will hold the array of axes [ax_x, ax_speed]
        
        # Lines for the plots
        self.line_x = None             # X-coordinate line
        self.line_speed = None         # Current speed line
        self.line_desired_speed = None # Desired speed line
        
        # Data storage
        self.time_data = []
        self.x_data = []
        self.speed_data = []
        self.desired_speed_data = [] 
        
        self.last_plot_time = 0.0
        self.update_interval = update_interval 
        
        self.is_initialized = False

    def _configure_axis(self, index, title, xlabel, ylabel, 
                        xlim=None, ylim=None, 
                        xlocator_base=None, ylocator_base=None):
        """
        Helper function to configure a subplot axis.

        Args:
            index (int): The index of the axis in self.ax array (0 for X, 1 for speed).
            title (str): Title for the subplot.
            xlabel (str): Label for the X-axis.
            ylabel (str): Label for the Y-axis.
            xlim (tuple, optional): Initial X-axis limits. Defaults to None.
            ylim (tuple, optional): Initial Y-axis limits. Defaults to None.
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
        return axis 

    def init_plot(self):
        """Initializes the Matplotlib figure and axes for the plots."""
        if self.is_initialized:
            print("Plot is already initialized.")
            return

        print("Initializing plot window...")
        plt.ion()  # Turn on interactive mode
        self.last_plot_time = time.time()
        # Create figure and 2 subplots ( vertically stacked)
        self.fig, self.ax = plt.subplots(nrows=2, ncols=1, figsize=(8, 6)) 
        # Adjust vertical spacing between subplots
        plt.subplots_adjust(hspace=0.4) 

        # --- Configure Top Plot (X coordinate vs. Time) ---
        # Define initial limits and tick spacing
        init_xlim = (0.0, 20.0)
        init_ylim_x = (-100.0, 200.0) # Example limits for X coordinate
        xlocator_base = 4.0
        ylocator_base_x = 50.0 
        ax0 = self._configure_axis(0, "Vehicle X Coordinate", "Time (s)", "X Coordinate (m)", 
                                   xlim=init_xlim, ylim=init_ylim_x, 
                                   xlocator_base=xlocator_base, ylocator_base=ylocator_base_x)
        self.line_x, = ax0.plot([], [], color='blue', linewidth=2, label='X Coordinate') 
        ax0.legend(loc='best') 

        # --- Configure Bottom Plot (Speed vs. Time) ---
        init_ylim_speed = (0.0, 80.0) # Example limits for speed
        ylocator_base_speed = 10.0
        ax1 = self._configure_axis(1, "Vehicle Speed", "Time (s)", "Speed (km/h)", 
                                   xlim=init_xlim, ylim=init_ylim_speed, 
                                   xlocator_base=xlocator_base, ylocator_base=ylocator_base_speed)
        self.line_speed, = ax1.plot([], [], color='red', linewidth=2, label='Current Speed') 
        self.line_desired_speed, = ax1.plot([], [], color='green', linewidth=2, linestyle='--', label='Desired Speed') # Changed color for distinction
        ax1.legend(loc='best') 
        
        # Set window title if possible
        if self.fig and self.fig.canvas.manager:
             self.fig.canvas.manager.set_window_title('CARLA X-Coordinate and Speed Plot') 
        else:
             print("Warning: Could not get canvas manager to set window title.")
             
        plt.show(block=False) # Show plot without blocking execution
        # A small pause helps ensure the window renders before the first update call
        plt.pause(0.1) 
        
        self.is_initialized = True
        print("Plot window initialized.")

    def update_plot(self, current_sim_time_sec, x, speed, v_desired):
        """
        Updates the plot data and redraws the plots efficiently.

        Args:
            current_sim_time_sec (float): Current simulation time in seconds.
            x (float): Current vehicle X coordinate.
            speed (float): Current vehicle speed (e.g., in km/h).
            v_desired (float): Current desired speed (e.g., in km/h).
        """
        if not self.is_initialized or self.fig is None or self.ax is None:
            # Silently return if not initialized or plot closed
            return

        # Append new data points
        self.time_data.append(current_sim_time_sec)
        self.x_data.append(x)
        self.speed_data.append(speed)
        self.desired_speed_data.append(v_desired) 

        # Throttle updates based on the defined interval
        now = time.time()
        if now - self.last_plot_time < self.update_interval:
            return # Skip update if interval hasn't passed
        self.last_plot_time = now

        try:
            # Update the data for each line object
            if self.line_x: self.line_x.set_data(self.time_data, self.x_data)
            if self.line_speed: self.line_speed.set_data(self.time_data, self.speed_data)
            if self.line_desired_speed: self.line_desired_speed.set_data(self.time_data, self.desired_speed_data)

            # --- Dynamic Axis Rescaling ---
            # Rescale time axis (X) dynamically for both plots
            if self.time_data:
                max_time = self.time_data[-1]
                # Set x-limit to be slightly larger than max time, minimum 20s
                current_xlim_time = max(20.0, max_time + 1.0) 
                self.ax[0].set_xlim(0, current_xlim_time)
                self.ax[1].set_xlim(0, current_xlim_time)

            # Rescale Y-axis for X-coordinate plot
            if self.x_data:
                min_x = min(self.x_data)
                max_x = max(self.x_data)
                padding_x = max(10.0, (max_x - min_x) * 0.1) # Add padding
                self.ax[0].set_ylim(min_x - padding_x, max_x + padding_x)

            # Rescale Y-axis for speed plot
            if self.speed_data or self.desired_speed_data:
                all_speeds = self.speed_data + self.desired_speed_data
                min_s = 0.0 # Speed usually starts at 0
                max_s = max(all_speeds) if all_speeds else 80.0 # Use default if no data
                padding_s = max(5.0, (max_s - min_s) * 0.1) # Add padding
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
                 self.line_x = None
                 self.line_speed = None
                 self.line_desired_speed = None
                 print("Plot resources released.")
                 
        # Clear all stored data
        self.time_data.clear()
        self.x_data.clear()
        self.speed_data.clear()
        self.desired_speed_data.clear() 