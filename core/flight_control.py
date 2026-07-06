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
        success = self.mav.send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            param1=1.0,
            param2=4.0
        )
        if success:
            logger.info("MAV_CMD_DO_SET_MODE (GUIDED) commanded.")
        return success

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
