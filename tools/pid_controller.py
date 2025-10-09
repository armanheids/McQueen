"""
PID Controller for Longitudinal Speed Control in CARLA (pid_controller.py)

This module provides a basic PID (Proportional-Integral-Derivative) controller class
specifically designed for regulating the longitudinal speed of a CARLA vehicle.
It calculates throttle and brake commands based on the error between the desired
and current speed.
"""
import time
import carla # Needed for carla.VehicleControl

# --- Default PID Controller Gains ---
# These values can be tuned for different vehicles and scenarios
KP = 0.1  # Proportional gain
KI = 0.01 # Integral gain
KD = 0.001 # Derivative gain
# Consider adding INTEGRAL_LIMIT if anti-windup is needed
# INTEGRAL_LIMIT = 5.0 

class PIDController:
    """Implements a basic PID controller for longitudinal speed control."""

    def __init__(self, kp=KP, ki=KI, kd=KD):
        """
        Initializes the PID controller.

        Args:
            kp (float): Proportional gain. Defaults to KP constant.
            ki (float): Integral gain. Defaults to KI constant.
            kd (float): Derivative gain. Defaults to KD constant.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Optional: Anti-windup limit for the integral term
        # self.integral_limit = INTEGRAL_LIMIT 
        self.reset() # Initialize internal state

    def calculate_control(self, v_desired, v_current):
        """
        Calculates the throttle/brake control command based on PID logic.

        Args:
            v_desired (float): The desired speed (in the same units as v_current, e.g., km/h).
            v_current (float): The current speed of the vehicle.

        Returns:
            carla.VehicleControl: A control object with throttle and brake values set. 
                                   Steering is always set to 0.0.
        """
        current_time = time.time()
        delta_t = current_time - self._last_time
        
        # Avoid division by zero or large calculations if time hasn't passed
        if delta_t <= 1e-6: 
            # Return zero control if no time elapsed
            return carla.VehicleControl(steer=0.0, throttle=0.0, brake=0.0)

        # Calculate error
        error = v_desired - v_current
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term (with simple accumulation)
        self._integral = self._integral + error * delta_t 
        # Optional: Apply anti-windup clamp
        # self._integral = max(-self.integral_limit, min(self._integral, self.integral_limit))
        integral = self.ki * self._integral
        
        # Derivative term
        # Check delta_t again for safety, although already checked above
        derivative = 0.0
        if delta_t > 0:
             # Calculate derivative based on the change in error
             derivative = self.kd * (error - self._previous_error) / delta_t
        
        # Total control output (sum of terms)
        u = proportional + integral + derivative
        
        # Store current values for the next iteration
        self._previous_error = error
        self._last_time = current_time
        
        # Convert control output 'u' to throttle/brake commands
        control = carla.VehicleControl()
        control.steer = 0.0 # This controller only handles longitudinal control
        
        # Determine throttle/brake based on control output and desired speed
        if v_desired <= 0.01: 
            # Special case: If target speed is near zero, apply full brake
            control.throttle = 0.0
            control.brake = 1.0
        elif u >= 0:
            # Positive control output means accelerate/maintain speed
            # Cap throttle at 1.0
            control.throttle = min(u, 1.0) 
            control.brake = 0.0
        else:
            # Negative control output means brake
            # Cap brake at 1.0 (using absolute value of negative 'u')
            control.throttle = 0.0
            control.brake = min(abs(u), 1.0) 
            
        return control

    def reset(self):
        """Resets the internal state (integral, previous error, time) of the PID controller."""
        self._integral = 0.0
        self._previous_error = 0.0
        self._last_time = time.time()
        # Use logging instead of print for better integration potential
        # import logging
        # logging.info("PID controller reset.") 
        print("PID controller reset.") # Keep print for simplicity if logging isn't set up 