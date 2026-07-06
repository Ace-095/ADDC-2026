"""Visual servoing PID controller for precision aerial alignment over a QR code."""

import logging
import time
import numpy as np
from utils.filters import PIDController
from typing import Tuple, Optional

logger = logging.getLogger(__name__)


class AlignmentController:
    """Translate target pixel deviations into precise vehicle guided velocity vectors."""

    def __init__(self, config: dict):
        cfg = config['alignment']
        self.deadzone_px = cfg['center_deadzone_px']
        self.max_vel = cfg['max_velocity']
        self.stable_required = cfg['stable_frames_required']

        # PID constants
        self.kp = cfg.get('pid_kp', 0.5)
        self.ki = cfg.get('pid_ki', 0.08)
        self.kd = cfg.get('pid_kd', 0.2)

        # Axis inversion flags — flip to true if the vehicle drifts away from
        # the target instead of converging. Applied as -1 multiplier to the
        # metric errors before PID update. Correct value depends on camera
        # mounting orientation (image-top = drone-??). See alignment.yaml.
        self.invert_x = cfg.get('invert_x', False)
        self.invert_y = cfg.get('invert_y', False)
        
        # Camera physical properties
        self.image_width = config['camera']['width']
        self.image_height = config['camera']['height']
        
        # QR sizing config
        self.qr_size_cm = config['vision']['qr_size_cm']
        
        # Optics
        self.fov_horizontal_rad = np.radians(config['camera'].get('fov_horizontal_deg', 66.0))
        
        # PID instances
        self.pid_x = PIDController(self.kp, self.ki, self.kd, self.max_vel)
        self.pid_y = PIDController(self.kp, self.ki, self.kd, self.max_vel)
        
        self.stable_counter = 0

    def reset(self):
        """Reset the PID controller memory and stable counter."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.stable_counter = 0
        logger.info("Alignment PID controller state reset.")

    def compute(self, center: Tuple[int, int], pixel_width: Optional[int] = None, altitude_m: float = 5.0, frame_size: Optional[Tuple[int, int]] = None) -> Tuple[float, float, bool]:
        """
        Compute horizontal velocity command (vx, vy) to center the drone over the QR code.

        Args:
            center: (cx, cy) pixel coordinates of the detected QR code
            pixel_width: Bounding width of the QR code in pixels (for dynamic distance estimation)
            altitude_m: Telemetry height above ground in meters (fallback for scaling ratio)
            frame_size: Optional (width, height) tuple of the actual camera frame

        Returns:
            vx: Forward velocity command (NED local frame X, m/s)
            vy: Right velocity command (NED local frame Y, m/s)
            aligned: True if centering errors are stable within the deadzone
        """
        cx, cy = center
        current_time = time.time()

        # Dynamically support actual frame dimensions if passed
        width = frame_size[0] if frame_size is not None else self.image_width
        height = frame_size[1] if frame_size is not None else self.image_height

        # Target image center coordinates
        img_cx = width / 2.0
        img_cy = height / 2.0

        # Calculate pixel deviations
        err_x_px = cx - img_cx
        err_y_px = cy - img_cy

        # Apply deadzone filtering directly to pixel coordinates
        if abs(err_x_px) < self.deadzone_px:
            err_x_px = 0.0
        if abs(err_y_px) < self.deadzone_px:
            err_y_px = 0.0

        # Estimate distance from QR code for metric scale conversions
        if pixel_width is not None and pixel_width > 0:
            # Calculate focal length dynamically from FOV
            focal_length_px = (width / 2.0) / np.tan(self.fov_horizontal_rad / 2.0)
            real_width_m = self.qr_size_cm / 100.0
            distance_m = (real_width_m * focal_length_px) / pixel_width
        else:
            distance_m = altitude_m

        # Convert pixel error to metric distance using standard footprint formula
        px_to_m_ratio = (2.0 * distance_m * np.tan(self.fov_horizontal_rad / 2.0)) / width
        
        error_x_m = err_x_px * px_to_m_ratio
        error_y_m = err_y_px * px_to_m_ratio

        # Apply axis inversion if camera mounting differs from assumed convention.
        # Invert means: image-top ≠ drone-forward, or image-left ≠ drone-left.
        # Pixel errors (err_x_px/err_y_px) are NOT inverted — they drive the
        # deadzone gate and stable_counter which must remain sign-agnostic.
        if self.invert_x:
            error_x_m = -error_x_m
        if self.invert_y:
            error_y_m = -error_y_m

        # PID calculations:
        # Image coordinates: +X points right, +Y points down
        # NED frame: X points forward, Y points right
        # Therefore (assuming image-top = drone-nose):
        # Right error in image (+X err_x) maps to right velocity (+Y vy)
        # Down error in image (+Y err_y) maps to forward velocity (+X vx)
        vy = self.pid_x.update(error_x_m, current_time)
        vx = self.pid_y.update(error_y_m, current_time)

        # Check alignment stability
        if err_x_px == 0.0 and err_y_px == 0.0:
            self.stable_counter += 1
        else:
            self.stable_counter = 0

        aligned = self.stable_counter >= self.stable_required

        # Safety log alignment parameters
        if int(current_time) % 5 == 0:
            logger.debug(
                f"Aligning: px_err=({err_x_px:.0f},{err_y_px:.0f}) "
                f"m_err=({error_x_m:.2f},{error_y_m:.2f}) "
                f"cmd_vel=({vx:.2f},{vy:.2f}) "
                f"invert=({self.invert_x},{self.invert_y}) "
                f"stable={self.stable_counter}/{self.stable_required}"
            )

        return float(vx), float(vy), aligned
