"""Payload release manager with distance sensor integration, safety interlocks, and SITL compatibility."""

import os
import logging
import time
import threading
from pymavlink import mavutil
from utils.filters import MedianFilter
from typing import Optional

logger = logging.getLogger(__name__)


class PayloadControl:
    """Manages physical payload servo release commands with safety interlocks and distance sensors."""

    def __init__(self, config: dict, flight_control):
        self.cfg = config['mavlink']
        self.flight_cfg = config['flight']
        self.fc = flight_control
        self.mav = flight_control.mav

        self.channel = self.cfg['payload_servo_channel']
        self.pwm_open = self.cfg['servo_pwm_open']
        self.pwm_closed = self.cfg['servo_pwm_closed']
        self.duration = self.cfg['servo_open_duration_s']

        self.alt_min = self.flight_cfg['release_distance_min_m']
        self.alt_max = self.flight_cfg['release_distance_max_m']
        self.takeoff_threshold = self.flight_cfg['takeoff_altitude_threshold_m']

        self.use_sitl = config['system'].get('use_sitl', False)
        
        # GPIO configuration
        self.trig_pin = self.cfg.get('ultrasonic_trig_pin', 23)
        self.echo_pin = self.cfg.get('ultrasonic_echo_pin', 24)
        
        self.distance_sensor = None
        
        # Initialize distance sensor if on real hardware
        if not self.use_sitl:
            try:
                from gpiozero import DistanceSensor
                logger.info(f"Initializing ultrasonic DistanceSensor (trig={self.trig_pin}, echo={self.echo_pin})...")
                self.distance_sensor = DistanceSensor(
                    echo=self.echo_pin,
                    trigger=self.trig_pin,
                    max_distance=4.0
                )
                logger.info("DistanceSensor successfully initialized.")
            except ImportError:
                logger.warning("gpiozero library not installed. Distance sensor fallback to MAVLink altitude.")
            except Exception as e:
                logger.warning(f"Failed to initialize DistanceSensor: {e}. Fallback to MAVLink altitude.")

        # Altimeter filter
        self.alt_filter = MedianFilter(window_size=5)
        
        # State tracking
        self.takeoff_detected = False
        self.payload_released = False
        self._release_in_progress = False
        self._lock = threading.Lock()

    def check_takeoff_safety(self, current_altitude_m: float) -> bool:
        """
        Check if the drone has exceeded takeoff altitude to arm the release gate.
        
        Args:
            current_altitude_m: Current telemetry altitude in meters
        """
        if self.takeoff_detected:
            return True

        if current_altitude_m > self.takeoff_threshold:
            self.takeoff_detected = True
            self.mav.send_statustext("PAYLOAD_ARMED: Takeoff height cleared.")
            logger.info(f"Takeoff detected. Altitude {current_altitude_m:.2f}m crossed arm threshold {self.takeoff_threshold}m.")
            return True

        return False

    def get_distance_reading(self) -> float:
        """
        Read distance in meters from physical sensor, falling back to MAVLink relative altitude in SITL.
        
        Returns:
            Measured distance in meters
        """
        if self.use_sitl or self.distance_sensor is None:
            # Fall back to relative altitude from Pixhawk
            return self.mav.get_altitude()
        
        try:
            # gpiozero returns distance in meters
            dist = float(self.distance_sensor.distance)
            return dist
        except Exception as e:
            logger.warning(f"Error reading physical distance sensor: {e}")
            return self.mav.get_altitude()

    def is_in_release_window(self, raw_distance_sensor_m: float) -> bool:
        """
        Verify if the filtered height is within the safe drop window.
        
        Args:
            raw_distance_sensor_m: Raw distance reading from downward sensor
            
        Returns:
            True if filtered height is inside release limits
        """
        filtered_alt = self.alt_filter.update(raw_distance_sensor_m)
        if filtered_alt is None:
            return False

        in_window = self.alt_min <= filtered_alt <= self.alt_max
        if in_window:
            logger.debug(f"Altitude check PASSED: {filtered_alt:.3f}m is within [{self.alt_min}, {self.alt_max}]")
        else:
            logger.debug(f"Altitude check FAILED: {filtered_alt:.3f}m is outside window")
            
        return in_window

    def trigger_release(self) -> bool:
        """
        Command payload release in a non-blocking background thread.

        Returns:
            True if the release sequence was successfully initiated
        """
        with self._lock:
            if self.payload_released:
                logger.warning("Payload already released. Aborting duplicate command.")
                return False

            if self._release_in_progress:
                logger.warning("Payload release is already in progress.")
                return False

            self._release_in_progress = True
            
        threading.Thread(target=self._release_worker, daemon=True).start()
        return True

    def _release_worker(self):
        """Background thread executing the open-wait-close servo sequence."""
        try:
            logger.info("🎯 INITIATING PAYLOAD SERVO SWEEP")
            self.mav.send_statustext("PAYLOAD_DROP: Releasing Servo")

            # 1. Send servo open pulse
            success = self.mav.send_command_long(
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                param1=float(self.channel),
                param2=float(self.pwm_open)
            )
            
            if not success:
                logger.error("Failed to transmit MAV_CMD_DO_SET_SERVO open command.")
                self.mav.send_statustext("PAYLOAD_ERROR: Servo command rejected")
                self._release_in_progress = False
                return

            # 2. Wait open duration
            time.sleep(self.duration)

            # 3. Send servo close pulse
            logger.info("Closing payload servo...")
            self.mav.send_command_long(
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                param1=float(self.channel),
                param2=float(self.pwm_closed)
            )

            with self._lock:
                self.payload_released = True
                
            self.mav.send_statustext("PAYLOAD_SUCCESS: Release complete")
            logger.info("✅ Payload release sequence finished successfully.")

        except Exception as e:
            logger.error(f"Error during payload release sequence: {e}")
            try:
                self.mav.send_command_long(
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    param1=float(self.channel),
                    param2=float(self.pwm_closed)
                )
            except:
                pass
        finally:
            with self._lock:
                self._release_in_progress = False
