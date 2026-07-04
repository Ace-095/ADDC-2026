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

    def compute(self, center: Tuple[int, int], pixel_width: Optional[int] = None, altitude_m: float = 5.0) -> Tuple[float, float, bool]:
        """
        Compute horizontal velocity command (vx, vy) to center the drone over the QR code.

        Args:
            center: (cx, cy) pixel coordinates of the detected QR code
            pixel_width: Bounding width of the QR code in pixels (for dynamic distance estimation)
            altitude_m: Telemetry height above ground in meters (fallback for scaling ratio)

        Returns:
            vx: Forward velocity command (NED local frame X, m/s)
            vy: Right velocity command (NED local frame Y, m/s)
            aligned: True if centering errors are stable within the deadzone
        """
        cx, cy = center
        current_time = time.time()

        # Target image center coordinates
        img_cx = self.image_width / 2.0
        img_cy = self.image_height / 2.0

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
            focal_length_px = (self.image_width / 2.0) / np.tan(self.fov_horizontal_rad / 2.0)
            real_width_m = self.qr_size_cm / 100.0
            distance_m = (real_width_m * focal_length_px) / pixel_width
        else:
            distance_m = altitude_m

        # Convert pixel error to metric distance using standard footprint formula
        px_to_m_ratio = (2.0 * distance_m * np.tan(self.fov_horizontal_rad / 2.0)) / self.image_width
        
        error_x_m = err_x_px * px_to_m_ratio
        error_y_m = err_y_px * px_to_m_ratio

        # PID calculations:
        # Image coordinates: +X points right, +Y points down
        # NED frame: X points forward, Y points right
        # Therefore:
        # Right error in image (+Y err_x) maps to right velocity (+Y vy)
        # Down error in image (+X err_y) maps to forward velocity (+X vx)
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
                f"cmd_vel=({vx:.2f},{vy:.2f}) stable={self.stable_counter}/{self.stable_required}"
            )

        return float(vx), float(vy), aligned
