"""Finite State Machine (FSM) governing autonomous mission execution phases."""

import logging
import math
from enum import Enum, auto
from core.payload_control import PayloadControl
from typing import Optional

logger = logging.getLogger(__name__)


class State(Enum):
    """FSM execution phases."""
    BOOT = auto()
    MONITOR_AUTO = auto()
    REQUEST_GUIDED = auto()
    GUIDED_HOLD = auto()
    INITIAL_SCAN = auto()
    SEARCH_SQUARE = auto()
    RETURN_INITIAL = auto()
    ALIGNMENT = auto()
    QR_DECODE = auto()
    LAND = auto()
    RTL = auto()


class StateMachine:
    """Synchronous FSM driving flight mode transitions and vision target centering."""

    def __init__(self, config: dict, flight_control, camera_manager,
                 qr_detector, alignment_controller, qr_decoder,
                 payload_control: PayloadControl, fallback_manager):
        self.cfg = config
        self.fc = flight_control
        self.cam = camera_manager
        self.qr_det = qr_detector
        self.align = alignment_controller
        self.qr_dec = qr_decoder
        self.payload = payload_control
        self.fallback = fallback_manager
        
        self.state = State.BOOT
        self.state_entry_tick = 0
        self.vision_fail_counter = 0
        self.guided_request_counter = 0
        self.guided_request_retries = 0  # Number of complete 5s retry cycles exhausted
        self.hold_counter = 0
        
        self.vision_fail_limit = config['vision']['fail_limit']
        self.decode_hold_ticks = config['qr_decode']['hold_ticks']
        self.search_pattern_enabled = config['vision'].get('search_pattern_enabled', True)
        
        self._last_state_log_tick = -999

    def tick(self, tick_count: int):
        """
        Execute one synchronous tick iteration of the active state.

        Args:
            tick_count: Total ticks elapsed since program start
        """
        # Periodic state log
        if tick_count - self._last_state_log_tick > 40:  # Every 2 seconds at 20Hz
            logger.info(f"FSM State: {self.state.name} | Ticks: {tick_count}")
            self._last_state_log_tick = tick_count

        if self.state == State.BOOT:
            self._tick_boot(tick_count)
        elif self.state == State.MONITOR_AUTO:
            self._tick_monitor_auto(tick_count)
        elif self.state == State.REQUEST_GUIDED:
            self._tick_request_guided(tick_count)
        elif self.state == State.GUIDED_HOLD:
            self._tick_guided_hold(tick_count)
        elif self.state == State.INITIAL_SCAN:
            self._tick_initial_scan(tick_count)
        elif self.state == State.SEARCH_SQUARE:
            self._tick_search_square(tick_count)
        elif self.state == State.RETURN_INITIAL:
            self._tick_return_initial(tick_count)
        elif self.state == State.ALIGNMENT:
            self._tick_alignment(tick_count)
        elif self.state == State.QR_DECODE:
            self._tick_qr_decode(tick_count)
        elif self.state == State.LAND:
            self._tick_land(tick_count)
        elif self.state == State.RTL:
            self._tick_rtl(tick_count)

    def _transition(self, new_state: State, tick_count: int):
        """Handle state change transitions and resets."""
        logger.info(f"FSM TRANSITION: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_entry_tick = tick_count
        
        # Reset counters on state entry
        self.guided_request_counter = 0
        self.hold_counter = 0
        self.vision_fail_counter = 0
        self.climb_initiated = False
        if new_state == State.REQUEST_GUIDED:
            self.guided_request_retries = 0
        
        # Trigger controller resets if entering tracking modes
        if new_state == State.ALIGNMENT:
            self.align.reset()
        elif new_state == State.QR_DECODE:
            self.qr_dec.reset()

    def _tick_boot(self, tick_count: int):
        """Wait for Pixhawk MAVLink connection to establish."""
        if self.fc.mav.is_connected():
            logger.info("Autopilot link connected. Monitoring flight modes...")
            self._transition(State.MONITOR_AUTO, tick_count)

    def _tick_monitor_auto(self, tick_count: int):
        """Monitor for AUTO flight mode and trigger sprayer waypoint conditions."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        # Mid-flight restart policy: if the Pixhawk is already in GUIDED when we
        # boot, it means the companion computer crashed while it was driving the
        # vehicle (QR alignment, payload drop, etc.). We cannot resume safely -
        # trigger an immediate RTL.
        if self.fc.is_guided_mode():
            logger.critical(
                "Mid-flight reboot detected: Pixhawk is in GUIDED mode on Pi boot. "
                "Cannot resume mission - commanding RTL."
            )
            self.fallback.handle_fail("Mid-flight reboot detected in GUIDED mode")
            self._transition(State.RTL, tick_count)
            return

        # Continuous arming gate check for takeoff safety
        self.payload.check_takeoff_safety(self.fc.mav.get_altitude())

        if self.fc.is_auto_mode():
            current_wp = self.fc.mav.current_waypoint

            # --- Primary trigger: autopilot STATUSTEXT announces "Sprayer" ---
            # ArduPilot broadcasts "Mission: N Sprayer" when DO_SPRAYER executes.
            # This is far more reliable than MISSION_ITEM_INT round-trips.
            if self.fc.mav.sprayer_detected:
                logger.warning(
                    f"🎯 Sprayer command confirmed via autopilot STATUSTEXT at wp {current_wp}. "
                    "Switching to GUIDED."
                )
                self.fc.mav.send_statustext("TRIGGER: Sprayer STATUSTEXT intercepted.")
                # Consume the flag so it doesn't fire again
                self.fc.mav.sprayer_detected = False
                self._transition(State.REQUEST_GUIDED, tick_count)
                return

            # --- Secondary trigger: mission item cache check (MAV_CMD_DO_SPRAYER = 223) ---
            # Fallback only - sprayer detection is now STATUSTEXT-based (primary path above).
            # Do NOT re-request mission items here; those request/response round-trips add
            # extra packets at exactly the moment timing matters (waypoint transitions).
            cmd = self.fc.mav.mission_items.get(current_wp)
            if cmd == 223:
                logger.warning(f"🎯 MAV_CMD_DO_SPRAYER (223) found in cache at waypoint {current_wp} (fallback)!")
                self.fc.mav.send_statustext("TRIGGER: DO_SPRAYER cached cmd intercepted.")
                self._transition(State.REQUEST_GUIDED, tick_count)




    def _tick_request_guided(self, tick_count: int):
        """Request GUIDED flight mode and await heartbeat confirmations."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        # Throttle SET_MODE to once per second (every 20 ticks at 20Hz).
        # Flooding the autopilot with 20 commands/sec delays its response.
        if tick_count % 20 == 0:
            self.fc.set_guided_mode()
        self.guided_request_counter += 1

        # Check mode confirmation using autopilot-specific HEARTBEAT
        # (GCS HEARTBEATs with custom_mode=0 are filtered out in get_autopilot_heartbeat)
        if self.fc.is_guided_mode():
            logger.info("GUIDED mode change confirmed by autopilot heartbeat.")
            
            # Phase 1: Capture a real position reference at GUIDED entry
            self.guided_anchor_ned = self.fc.get_local_position()
            if self.guided_anchor_ned:
                x0, y0, z0 = self.guided_anchor_ned
                logger.info(f"Captured LOCAL_POSITION_NED origin: x0={x0:.2f}, y0={y0:.2f}, z0={z0:.2f}")
            else:
                logger.warning("Could not capture LOCAL_POSITION_NED origin at GUIDED entry!")
                
            self._transition(State.GUIDED_HOLD, tick_count)
            return

        # Safety retry timeout (5 seconds per cycle, max 3 cycles = 15 seconds total)
        if self.guided_request_counter > 100:
            self.guided_request_retries += 1
            logger.warning(f"Guided mode request timeout (attempt {self.guided_request_retries}/3).")
            self.guided_request_counter = 0

            if self.guided_request_retries >= 3:
                logger.error("GUIDED mode request failed after 3 retries (15s). Aborting mission.")
                self.fallback.handle_fail("REQUEST_GUIDED: max retries exceeded")
                self._transition(State.RTL, tick_count)

    def _tick_guided_hold(self, tick_count: int):
        """Hold position briefly to damp initial mode switch swings."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        self.fc.hold_position()
        self.hold_counter += 1

        if self.hold_counter > 40:  # 2 seconds at 20Hz
            self._transition(State.INITIAL_SCAN, tick_count)

    def _tick_initial_scan(self, tick_count: int):
        """Hover scan position and look for target QR bounding boxes."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        # Maintain stable hover during scan phase
        self.fc.hold_position()
        
        # Check initial scan hover timeout (clean 20s timer)
        elapsed_ticks = tick_count - self.state_entry_tick
        timeout_ticks = int(self.cfg['search'].get('initial_scan_timeout_s', 20.0) * self.cfg['system']['tick_hz'])
        
        if elapsed_ticks >= timeout_ticks:
            logger.warning(f"INITIAL_SCAN timeout ({timeout_ticks / self.cfg['system']['tick_hz']:.1f}s). Initiating SEARCH_SQUARE.")
            self._generate_search_pattern()
            self._transition(State.SEARCH_SQUARE, tick_count)
            return

        frame = self.cam.get_latest_frame()
        if frame is None:
            # Continue holding position, do not abort early if camera frame drops
            return

        # Look for target QR
        found, bbox, center = self.qr_det.detect(frame)
        if found:
            logger.info(f"Target QR locked at pixel coordinates: {center}")
            self._transition(State.ALIGNMENT, tick_count)
            return

    def _generate_search_pattern(self):
        """Generate local NED waypoints for a bounded lawnmower search."""
        anchor = getattr(self, 'guided_anchor_ned', None)
        if not anchor:
            logger.warning("guided_anchor_ned not set! Falling back to current local position.")
            anchor = self.fc.get_local_position()
            if not anchor:
                logger.error("Could not get local position anchor! Defaulting to 0,0,0")
                anchor = (0.0, 0.0, 0.0)
            
        x0, y0, z0 = anchor
        sq_size = self.cfg['search'].get('square_size_m', 3.0)
        half_size = sq_size / 2.0
        
        alt_m = self.cfg['flight'].get('approach_altitude_m', 5.0)
        fov_rad = math.radians(self.cfg['camera'].get('fov_horizontal_deg', 66.0))
        footprint_w = 2.0 * alt_m * math.tan(fov_rad / 2.0)
        overlap = self.cfg['search'].get('lane_overlap_pct', 0.20)
        lane_width = footprint_w * (1.0 - overlap)
        
        # Guard against zero or negative lane width
        if lane_width <= 0.1:
            lane_width = 1.0
            
        num_lanes = max(1, int(math.ceil(sq_size / lane_width)))
        
        self.search_waypoints = []
        # Generate lanes starting from back-left (-half_size, -half_size)
        start_x = x0 - half_size
        start_y = y0 - half_size
        
        for i in range(num_lanes + 1):
            x = start_x + (i * lane_width)
            # Cap at the front boundary
            if x > x0 + half_size:
                x = x0 + half_size
                
            if i % 2 == 0:
                self.search_waypoints.append((x, start_y, z0))
                self.search_waypoints.append((x, y0 + half_size, z0))
            else:
                self.search_waypoints.append((x, y0 + half_size, z0))
                self.search_waypoints.append((x, start_y, z0))
                
                
        self.current_wp_idx = 0
        
        # Calculate dynamic timeout: distance / speed + margin
        total_dist = 0.0
        curr_x, curr_y = x0, y0
        for wp_x, wp_y, wp_z in self.search_waypoints:
            total_dist += math.sqrt((wp_x - curr_x)**2 + (wp_y - curr_y)**2)
            curr_x, curr_y = wp_x, wp_y
            
        speed_m_s = self.cfg['flight'].get('search_speed_m_s', 1.0)
        margin_s = 15.0 # Give it 15 seconds of slack for turns and arrival settling
        timeout_s = (total_dist / speed_m_s) + margin_s
        self.search_timeout_ticks = int(timeout_s * self.cfg['system']['tick_hz'])
        
        logger.info(f"Generated {len(self.search_waypoints)} waypoints for {sq_size}m search bounds (Lane width: {lane_width:.2f}m).")
        logger.info(f"Dynamic SEARCH_SQUARE timeout set to {timeout_s:.1f}s for {total_dist:.1f}m total path.")

    def _tick_search_square(self, tick_count: int):
        """Execute lawnmower pattern in bounded area if initial scan fails."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        frame = self.cam.get_latest_frame()
        if frame is None:
            self.fc.hold_position()
            self.vision_fail_counter += 1
            if self.vision_fail_counter >= self.vision_fail_limit:
                logger.error("BOUNDED SEARCH ABORT: Camera frame unavailable for too long.")
                self.fallback.handle_fail("SEARCH_SQUARE: Camera frame loss timeout")
                self._transition(State.RTL, tick_count)
            return

        # Look for target QR
        found, bbox, center = self.qr_det.detect(frame)
        if found:
            logger.info(f"Target QR locked at pixel coordinates: {center} during SEARCH_SQUARE")
            self._transition(State.ALIGNMENT, tick_count)
            return

        # Check bounded search dynamic timeout
        elapsed_ticks = tick_count - self.state_entry_tick
        timeout_ticks = getattr(self, 'search_timeout_ticks', 90 * self.cfg['system']['tick_hz'])
        
        if elapsed_ticks >= timeout_ticks:
            logger.warning("SEARCH_SQUARE TIMEOUT: Target not found within dynamic limit. Initiating RETURN_INITIAL...")
            self.fallback.handle_fail("SEARCH_SQUARE: dynamic timeout exceeded")
            self._transition(State.RETURN_INITIAL, tick_count)
            return
            
        # Navigate through generated local NED waypoints
        if self.current_wp_idx < len(self.search_waypoints):
            target_x, target_y, target_z = self.search_waypoints[self.current_wp_idx]
            self.fc.goto_local_position(target_x, target_y, target_z)
            
            # Check arrival tolerance
            current_pos = self.fc.get_local_position()
            if current_pos:
                cx, cy, cz = current_pos
                dist = math.sqrt((target_x - cx)**2 + (target_y - cy)**2)
                tolerance = self.cfg['search'].get('position_tolerance_m', 0.3)
                
                if dist <= tolerance:
                    logger.info(f"Search Waypoint {self.current_wp_idx} reached.")
                    self.current_wp_idx += 1
        else:
            logger.warning("SEARCH_SQUARE EXHAUSTED: Pattern finished but target not found. Initiating RETURN_INITIAL...")
            self.fallback.handle_fail("SEARCH_SQUARE: pattern finished with no detection")
            self._transition(State.RETURN_INITIAL, tick_count)

    def _tick_return_initial(self, tick_count: int):
        """Return to the GUIDED entry anchor point before RTL."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return
            
        anchor = getattr(self, 'guided_anchor_ned', None)
        if not anchor:
            logger.warning("No guided anchor found for RETURN_INITIAL. Skipping straight to RTL.")
            self._transition(State.RTL, tick_count)
            return
            
        target_x, target_y, target_z = anchor
        self.fc.goto_local_position(target_x, target_y, target_z)
        
        current_pos = self.fc.get_local_position()
        if current_pos:
            cx, cy, cz = current_pos
            dist = math.sqrt((target_x - cx)**2 + (target_y - cy)**2)
            tolerance = self.cfg['search'].get('position_tolerance_m', 0.5)
            
            if dist <= tolerance:
                logger.warning("Returned to initial GUIDED anchor point. Initiating blind LAND sequence.")
                self._transition(State.LAND, tick_count)
                return
                
        # Timeout for the return journey
        elapsed_ticks = tick_count - self.state_entry_tick
        timeout_ticks = 30.0 * self.cfg['system']['tick_hz']
        if elapsed_ticks >= timeout_ticks:
            logger.error("RETURN_INITIAL timeout. Initiating blind LAND sequence anyway.")
            self._transition(State.LAND, tick_count)

    def _tick_alignment(self, tick_count: int):
        """Compute PID centering adjustments and guide the drone over the target center."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        frame = self.cam.get_latest_frame()
        if frame is None:
            self.fc.hold_position()
            self.vision_fail_counter += 1
            if self.vision_fail_counter >= self.vision_fail_limit:
                logger.error("ALIGNMENT ABORT: Camera frame unavailable for too long.")
                self.fallback.handle_fail("ALIGNMENT: Camera frame loss timeout")
                self._transition(State.RTL, tick_count)
            return

        found, bbox, center = self.qr_det.detect(frame)
        if not found:
            self.vision_fail_counter += 1
            if self.vision_fail_counter > 40:  # Lost for 2 seconds
                logger.warning("Lost target track during alignment. Re-searching...")
                self._transition(State.GUIDED_HOLD, tick_count)
            else:
                self.fc.hold_position()
            return

        self.vision_fail_counter = 0

        # RTK GPS Cross-Check for Vision Hallucinations
        if self.fc.distance_to_wp() > 5.0:
            logger.error("RTK vs Vision drift mismatch! Target > 5m from GPS waypoint. Aborting.")
            self.fallback.handle_fail("RTK/Vision drift mismatch")
            self._transition(State.RTL, tick_count)
            return

        # Calculate pixel dimensions for scaling
        x_coords = bbox[:, 0]
        pixel_width = int(x_coords.max() - x_coords.min())
        
        # Pull telemetry alt
        altitude_m = self.fc.mav.get_altitude()
        
        # Calculate corrective velocities
        vx, vy, aligned = self.align.compute(center, pixel_width=pixel_width, altitude_m=altitude_m)
        
        # Command horizontal adjustment with slow landing descent (vz = 0.1m/s)
        self.fc.send_velocity(vx, vy, vz=0.1)

        if aligned:
            logger.info("Centering stability target reached. Initiating QR Decoding payload parse...")
            self._transition(State.QR_DECODE, tick_count)

    def _tick_qr_decode(self, tick_count: int):
        """Command hover and parse QR text contents."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        self.fc.hold_position()
        
        frame = self.cam.get_latest_frame()
        if frame is None:
            return

        # Attempt to decode payload on current frame, using tracked target box for crop
        success, text, final = self.qr_dec.decode(frame, last_bbox=self.qr_det.last_bbox)
        
        if success:
            logger.info(f"Parsed QR Payload string: {text}")
            self.fc.send_qr_text(text)
            self._transition(State.LAND, tick_count)
        elif final:
            logger.warning("Failed to decode target payload. Initiating landing sequence anyway.")
            self._transition(State.LAND, tick_count)

    def _tick_land(self, tick_count: int):
        """Execute landing and trigger distance-sensor gated payload drops."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        ticks_in_state = tick_count - self.state_entry_tick
        if ticks_in_state == 0:
            logger.info("Commanding vehicle precision landing...")
            self.fc.land()

        # Closed-loop tracking via LANDING_TARGET messages
        frame = self.cam.get_latest_frame()
        if frame is not None:
            found, bbox, center = self.qr_det.detect(frame)
            if found:
                # RTK vs Vision cross check
                if self.fc.distance_to_wp() > 5.0:
                    logger.error("RTK vs Vision drift mismatch during landing! Aborting.")
                    self.fallback.handle_fail("RTK/Vision drift mismatch")
                    self._transition(State.RTL, tick_count)
                    return
                
                h, w = frame.shape[:2]
                cx, cy = center
                img_cx, img_cy = w / 2.0, h / 2.0
                
                err_x_px = cx - img_cx
                err_y_px = cy - img_cy
                
                focal_length_px = w * 0.828
                
                # Image +Y is down (backward in standard mounting), Image +X is right
                angle_x = math.atan(err_y_px / focal_length_px)
                angle_y = math.atan(err_x_px / focal_length_px)
                
                dist_m = self.fc.mav.get_altitude()
                self.fc.send_landing_target(angle_x, angle_y, dist_m)

        # Hard landing timeout - if the release window is never reached (e.g. sensor failure,
        # horizontal drift), abort with RTL rather than hovering indefinitely.
        if ticks_in_state > 600:  # 30 seconds at 20 Hz
            logger.error("LAND TIMEOUT: Release window not reached in 30s. Triggering RTL.")
            self.fallback.handle_fail("LAND timeout: release window never hit")
            self._transition(State.RTL, tick_count)
            return

        # Altitude drop gate checks (only if payload not released yet)
        if not self.payload.payload_released:
            dist = self.payload.get_distance_reading()
            if self.payload.is_in_release_window(dist) and self.fc.is_landed():
                # Double safety arming check
                if self.payload.takeoff_detected:
                    logger.warning(f"Safe release altitude window met: {dist:.3f}m. Releasing payload...")
                    self.payload.trigger_release()
                else:
                    logger.warning(f"Drop altitude met ({dist:.3f}m) but takeoff arming safety gate is active. Aborting.")
        
        # Once payload release finished, command safe climb then RTL
        if self.payload.payload_released:
            if not getattr(self, 'climb_initiated', False):
                logger.info("Payload drop complete. Switching to GUIDED for safe climb.")
                self.fc.set_guided_mode()
                self.climb_initiated = True
                
            climb_alt = self.cfg['search'].get('post_release_climb_altitude_m', 3.0)
            current_alt = self.fc.get_altitude()
            
            if current_alt < climb_alt - 0.3:
                # Command upward velocity (-Z in NED frame)
                self.fc.send_velocity(0.0, 0.0, -0.5)
            else:
                logger.info(f"Safe climb altitude ({climb_alt}m) reached. Requesting RTL...")
                self.fc.rtl()
                self._transition(State.RTL, tick_count)

    def _tick_rtl(self, tick_count: int):
        """Maintain Return-To-Launch state loop."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        ticks_in_state = tick_count - self.state_entry_tick
        if ticks_in_state == 0:
            logger.info("Commanding vehicle Return To Launch...")
            self.fc.rtl()
            
        # Drone returns home under autopilot control
