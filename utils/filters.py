"""Filters and control loop mathematical helper utilities."""

import numpy as np
from collections import deque


class PIDController:
    """Proportional-Integral-Derivative controller for visual servoing and positioning."""

    def __init__(self, kp: float, ki: float, kd: float, max_output: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        """Reset the controller internal integrator and differential memory."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error: float, current_time: float) -> float:
        """
        Calculate the PID output based on time differences.

        Args:
            error: Current error value
            current_time: Current timestamp in seconds

        Returns:
            PID correction velocity output clamped to safety limits
        """
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0

        dt = current_time - self.prev_time
        if dt <= 0:
            return 0.0

        if error == 0.0:
            self.integral = 0.0

        # Derivative term
        derivative = (error - self.prev_error) / dt

        # Provisional output for conditional integration (anti-windup)
        provisional_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Check saturation
        is_saturated_positive = provisional_output >= self.max_output
        is_saturated_negative = provisional_output <= -self.max_output
        
        # Only integrate if we are NOT saturated in the direction of the error
        if not ((is_saturated_positive and error > 0) or (is_saturated_negative and error < 0)):
            self.integral += error * dt

        # Final output
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Clamp output
        output = max(-self.max_output, min(self.max_output, output))

        # Save states
        self.prev_error = error
        self.prev_time = current_time

        return output


class MedianFilter:
    """Rolling window median filter for noise-prone distance sensors."""

    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)

    def reset(self):
        """Clear the filter history buffer."""
        self.buffer.clear()

    def update(self, value: float) -> float:
        """
        Push a new value and return the median of the current window.

        Args:
            value: The latest raw sensor reading

        Returns:
            Median filtered result
        """
        if value is None:
            if len(self.buffer) > 0:
                return float(np.median(self.buffer))
            return None

        self.buffer.append(value)
        return float(np.median(self.buffer))
