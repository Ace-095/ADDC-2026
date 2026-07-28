"""Flight control abstraction layer routing FSM actions to MAVLink commands."""

import logging
from pymavlink import mavutil
from typing import Optional

logger = logging.getLogger(__name__)


class FlightControl:
    """High-level autopilot command driver wrapper."""

    def __init__(self, mavlink_interface):
        self.mav = mavlink_interface

    def is_auto_mode(self) -> bool:
        """Check if ArduPilot is actively executing an AUTO mission."""
        # Reject stale HBs: if last autopilot HB is > 5s old we cannot trust custom_mode.
        # HB arrives at 1Hz; 5s = 4 missed packets before we flag it.
        hb_age = self.mav.get_autopilot_hb_age()
        if hb_age > 5.0:
            logger.warning(f"[HB-STALE] is_auto_mode: autopilot HB age={hb_age:.1f}s (>5s), returning False")
            return False
        hb = self.mav.get_autopilot_heartbeat()
        if not hb:
            return False
        # custom_mode = 3 represents AUTO mode in ArduPilot
        return hb.custom_mode == 3

    def is_guided_mode(self) -> bool:
        """Check if ArduPilot is currently in GUIDED mode.

        Used by the mid-flight restart policy: if the companion computer reboots
        and finds the Pixhawk already in GUIDED, it implies an unclean crash
        occurred during QR alignment or payload drop and an RTL should be issued.
        """
        # Reject stale HBs before trusting custom_mode == 4.
        hb_age = self.mav.get_autopilot_hb_age()
        if hb_age > 5.0:
            logger.warning(f"[HB-STALE] is_guided_mode: autopilot HB age={hb_age:.1f}s (>5s), returning False")
            return False
        hb = self.mav.get_autopilot_heartbeat()
        if not hb:
            return False
        # custom_mode = 4 represents GUIDED mode in ArduPilot
        return hb.custom_mode == 4

    def is_land_mode(self) -> bool:
        """Check if ArduPilot is currently in LAND mode.
        
        custom_mode = 9 represents LAND mode in ArduCopter.
        """
        hb_age = self.mav.get_autopilot_hb_age()
        if hb_age > 5.0:
            return False
        hb = self.mav.get_autopilot_heartbeat()
        if not hb:
            return False
        return hb.custom_mode == 9

    def is_armed(self) -> bool:
        """Check if ArduPilot motors are currently armed."""
        hb_age = self.mav.get_autopilot_hb_age()
        if hb_age > 5.0:
            return False
        hb = self.mav.get_autopilot_heartbeat()
        if not hb:
            return False
        return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

    def distance_to_wp(self) -> float:
        """
        Get the current distance to the active waypoint in meters.

        Returns:
            Distance in meters, or 999.0 if telemetry is unavailable
        """
        msg = self.mav.get_message('NAV_CONTROLLER_OUTPUT')
        if msg:
            # wp_dist is distance to active waypoint in meters
            return float(msg.wp_dist)
        
        # Alternative: calculate distance if we have waypoint coordinate details
        # Fall back to 999.0 to indicate no telemetry read
        return 999.0

    def set_guided_mode(self) -> bool:
        """Request the Pixhawk autopilot to transition into GUIDED flight mode."""
        # MAV_CMD_DO_SET_MODE = 176
        # param1 = 1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
        # param2 = 4 (ArduPilot GUIDED Custom Mode ID)
        return self.set_mode(4)

    def set_mode(self, custom_mode: int) -> bool:
        """Request an ArduCopter custom flight mode by ID.

        Centralises MAV_CMD_DO_SET_MODE so LOITER (5), LAND (9), RTL (6), etc.
        can be requested uniformly. ArduCopter custom_mode IDs:
            0 STABILIZE | 1 ACRO | 2 ALT_HOLD | 3 AUTO | 4 GUIDED
            5 LOITER    | 6 RTL  | 7 CIRCLE   | 9 LAND  | 18 BRAKE

        Args:
            custom_mode: ArduCopter custom_mode integer.
        """
        success = self.mav.send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            param1=1.0,   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=float(custom_mode),
        )
        if success:
            logger.info(f"MAV_CMD_DO_SET_MODE (custom_mode={custom_mode}) commanded.")
        return success

    def set_loiter_mode(self) -> bool:
        """Request LOITER — GPS position + altitude hold (safe hover hold)."""
        return self.set_mode(5)

    def hold_position(self) -> bool:
        """Command the drone to hover in place with zero horizontal/vertical velocity."""
        return self.mav.set_guided_velocity(0.0, 0.0, 0.0)

    def send_velocity(self, vx: float, vy: float, vz: float) -> bool:
        """
        Send horizontal and vertical velocities in the body NED frame.

        Args:
            vx: Forward velocity (m/s)
            vy: Right velocity (m/s)
            vz: Down/descend velocity (m/s)
        """
        return self.mav.set_guided_velocity(vx, vy, vz)

    def set_yaw_rate(self, yaw_rate_deg_s: float) -> bool:
        """Command a body yaw rate (deg/s) in GUIDED while holding XY + altitude.

        Used by the INITIAL_SCAN slow yaw-sweep (CHANGE 4). Positive = clockwise.
        """
        return self.mav.set_guided_yaw_rate(yaw_rate_deg_s)

    def goto_local_position(self, x: float, y: float, z: float) -> bool:
        """
        Command the drone to fly to a specific local NED coordinate.
        """
        return self.mav.set_position_target_local_ned(x, y, z)

    def set_search_speed(self, speed_m_s: float) -> bool:
        """Cap autopilot groundspeed for slow search pattern scanning.

        Sends MAV_CMD_DO_CHANGE_SPEED (178) so ArduPilot actually flies
        GUIDED position targets at the configured search speed rather than
        its internal default (WPNAV_SPEED, typically 3–5 m/s). At 3+ m/s
        the drone traverses a 4.4m camera footprint in ~1.5s — too fast
        for the QR detector to acquire a 21cm code reliably.

        Call once per state entry, NOT on every tick.

        Args:
            speed_m_s: Target groundspeed in m/s (0.3–0.5 recommended for detection)
        """
        success = self.mav.send_command_long(
            178,          # MAV_CMD_DO_CHANGE_SPEED
            param1=1.0,   # speed_type: 1 = groundspeed
            param2=float(speed_m_s),
            param3=-1.0,  # throttle: -1 = no change
            param4=0.0,   # relative: 0 = absolute speed
        )
        if success:
            logger.info(f"Search speed set to {speed_m_s:.2f} m/s via MAV_CMD_DO_CHANGE_SPEED.")
        else:
            logger.warning(f"Failed to set search speed to {speed_m_s:.2f} m/s.")
        return success

    def restore_normal_speed(self, speed_m_s: float) -> bool:
        """Restore autopilot groundspeed to normal after search completes.

        Issued once before transitioning out of RETURN_INITIAL so RTL
        is not left permanently capped at the slow search speed.

        Args:
            speed_m_s: Normal cruise groundspeed in m/s (should match WPNAV_SPEED param)
        """
        success = self.mav.send_command_long(
            178,
            param1=1.0,
            param2=float(speed_m_s),
            param3=-1.0,
            param4=0.0,
        )
        if success:
            logger.info(f"Normal speed restored to {speed_m_s:.2f} m/s via MAV_CMD_DO_CHANGE_SPEED.")
        else:
            logger.warning(f"Failed to restore normal speed to {speed_m_s:.2f} m/s.")
        return success

    # ── CHANGE 5: search navigation accel/decel tuning ────────────────────
    # ArduPilot WPNAV parameters govern GUIDED position-target tracking.
    # Tighter accel/decel + a low cruise speed cut overshoot, tracking error,
    # and motion blur — the real reasons the QR detector could not lock during
    # the search. These are written at SEARCH_SQUARE entry and restored on exit
    # so the rest of the mission keeps default WPNAV behaviour.
    _SEARCH_NAV_PARAMS = {
        'WPNAV_SPEED':    ('speed', None),     # capped via DO_CHANGE_SPEED instead
        'WPNAV_ACCEL':    ('accel', None),
        'WPNAV_DECEL':    ('decel', None),
        'WPNAV_RADIUS':   ('wp_radius', None),
    }

    def apply_search_nav_tuning(self, accel: float, decel: float, wp_radius: float) -> None:
        """Tighten ArduPilot WPNAV accel/decel/radius for the slow search pattern.

        Best-effort: each PARAM_SET is fire-and-forget (ArduPilot applies it on
        the next control cycle). Failures are logged, not fatal — the DO_CHANGE_SPEED
        groundspeed cap is the primary motion control and still applies.
        """
        self.mav.set_param('WPNAV_ACCEL', float(accel))
        self.mav.set_param('WPNAV_DECEL', float(decel))
        self.mav.set_param('WPNAV_RADIUS', float(wp_radius))
        logger.info(f"Search nav tuning applied: accel={accel} decel={decel} radius={wp_radius}")

    def restore_default_nav_tuning(self, accel: float, decel: float, wp_radius: float) -> None:
        """Restore ArduPilot WPNAV params to their mission defaults after search."""
        self.mav.set_param('WPNAV_ACCEL', float(accel))
        self.mav.set_param('WPNAV_DECEL', float(decel))
        self.mav.set_param('WPNAV_RADIUS', float(wp_radius))
        logger.info(f"Default nav tuning restored: accel={accel} decel={decel} radius={wp_radius}")

    def get_local_position(self) -> Optional[tuple]:
        """
        Get the current local NED position as (x, y, z).
        """
        return self.mav.get_local_position_ned()

    def land(self) -> bool:
        """Command the vehicle to enter precision vertical landing mode."""
        success = self.mav.send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_LAND
        )
        if success:
            logger.info("MAV_CMD_NAV_LAND commanded.")
        else:
            logger.error("Failed to send MAV_CMD_NAV_LAND command.")
        return success

    def arm(self) -> bool:
        """Arm the vehicle motors."""
        success = self.mav.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1.0  # 1 to arm, 0 to disarm
        )
        if success:
            logger.info("MAV_CMD_COMPONENT_ARM_DISARM commanded (arm).")
        else:
            logger.error("Failed to send MAV_CMD_COMPONENT_ARM_DISARM command.")
        return success

    def takeoff(self, altitude_m: float) -> bool:
        """Command the vehicle to takeoff to a specific altitude."""
        success = self.mav.send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            param7=altitude_m
        )
        if success:
            logger.info(f"MAV_CMD_NAV_TAKEOFF to {altitude_m}m commanded.")
        else:
            logger.error("Failed to send MAV_CMD_NAV_TAKEOFF command.")
        return success

    def rtl(self) -> bool:
        """Command the vehicle to return to takeoff launch coordinates."""
        success = self.mav.send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        )
        if success:
            logger.info("MAV_CMD_NAV_RETURN_TO_LAUNCH commanded.")
        return success

    def set_home_precise(self, lat_deg: float, lon_deg: float, alt_m: float) -> bool:
        """Overwrite ArduCopter's home location with a specific GPS coordinate.

        Uses MAV_CMD_DO_SET_HOME (179) via COMMAND_INT so lat/lon are carried as
        int32 degE7 — precision ~0.01 mm vs ~1 m from COMMAND_LONG's 32-bit float.

        param1 = 0  → use specified location (not current position)
        x      = lat_deg × 1e7  (int32)
        y      = lon_deg × 1e7  (int32)
        z      = alt_m MSL      (float)

        Must be called AFTER a confirmed re-arm (ArduCopter resets home on every
        arm cycle). Confirm success by polling HOME_POSITION broadcast, not just
        COMMAND_ACK.

        Args:
            lat_deg: Latitude  in decimal degrees.
            lon_deg: Longitude in decimal degrees.
            alt_m:  Altitude in metres MSL.
        """
        from pymavlink import mavutil
        lat_e7 = int(lat_deg * 1e7)
        lon_e7 = int(lon_deg * 1e7)
        success = self.mav.send_command_int(
            command=179,                           # MAV_CMD_DO_SET_HOME
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            param1=0.0,                            # 0 = use specified location
            x=lat_e7,
            y=lon_e7,
            z=float(alt_m),
        )
        if success:
            logger.info(
                f"MAV_CMD_DO_SET_HOME (COMMAND_INT) sent: "
                f"({lat_deg:.7f}\u00b0, {lon_deg:.7f}\u00b0, {alt_m:.1f}m MSL) "
                f"lat_e7={lat_e7} lon_e7={lon_e7}"
            )
        else:
            logger.error("Failed to send MAV_CMD_DO_SET_HOME via COMMAND_INT.")
        return success


    def send_qr_text(self, text: str) -> bool:
        """Send the decoded QR text payload back to GCS STATUSTEXT logs."""
        logger.info(f"Visual Scan Result: {text}")
        # MAVLink STATUSTEXT info level (6)
        return self.mav.send_statustext(f"QR: {text}", severity=6)

    def send_landing_target(self, angle_x: float, angle_y: float, distance: float) -> bool:
        """Send a precision landing target update to ArduPilot."""
        return self.mav.send_landing_target(angle_x, angle_y, distance)

    def is_landed(self) -> bool:
        """
        Verify the vehicle is fully landed using multiple telemetry signals.
        Cross-checks MAV_LANDED_STATE, throttle, and altitude.
        """
        sys_state = self.mav.get_message('EXTENDED_SYS_STATE')
        vfr = self.mav.get_message('VFR_HUD')
        alt = self.mav.get_altitude()
        
        # Check MAV_LANDED_STATE_ON_GROUND (1)
        if not sys_state or sys_state.landed_state != 1:
            return False
            
        # Check throttle idle (motors disarmed/idle)
        if not vfr or vfr.throttle > 0:
            return False
            
        # Check altitude is near zero
        if alt > 0.3:
            return False
            
        return True
