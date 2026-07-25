"""Finite State Machine (FSM) governing autonomous mission execution phases."""

import logging
import math
import time
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
    REASSERT_HOME = auto()
    CLIMB = auto()
    RETURN_TO_ORIGIN = auto()
    RTL = auto()
    MISSION_COMPLETE = auto()


class StateMachine:
    """Synchronous FSM driving flight mode transitions and vision target centering."""

    def __init__(self, config: dict, flight_control, vision_pipeline,
                 payload_control: PayloadControl, fallback_manager):
        self.cfg = config
        self.fc = flight_control
        self.vision = vision_pipeline
        self.payload = payload_control
        self.fallback = fallback_manager
        
        self.state = State.BOOT
        self.state_entry_tick = 0
        self.vision_fail_counter = 0
        self.guided_request_counter = 0
        self.guided_request_retries = 0  # Number of complete 5s retry cycles exhausted
        self.land_request_counter = 0
        self.land_request_retries = 0
        self.land_confirmed = False
        self.takeoff_initiated = False
        self.takeoff_request_counter = 0
        self.hold_counter = 0
        self.return_land_commanded = False

        # Position reference captured in GUIDED_HOLD after telemetry is confirmed available.
        # Never set to None silently — only transitions out of GUIDED_HOLD once this is non-None.
        self.guided_anchor_ned = None

        # True home position — captured ONCE on first arm detection in MONITOR_AUTO,
        # before any waypoint navigation or re-arm can corrupt the EKF origin.
        # Contains both GPS (lat_deg, lon_deg, alt_msl_m) and NED (x, y, z) for redundancy.
        # Never overwritten after initial capture.
        self.true_home = None  # type: Optional[dict]

        # Edge-trigger guard: True when the autopilot was armed on the previous tick.
        # Used to detect the disarmed→armed edge so true_home is captured exactly once
        # (on first arm) without firing again on re-arms after landing.
        self._was_armed: bool = False
        
        self.vision_fail_limit = config['vision']['fail_limit']
        self.decode_hold_ticks = config['qr_decode']['hold_ticks']
        self.search_pattern_enabled = config['vision'].get('search_pattern_enabled', True)
        
        # Landing context: 'mission' for delivery (QR target), 'platform' for return precision landing.
        # Controls ALIGNMENT transition (skip QR_DECODE for platform) and LAND behaviour
        # (no payload release / re-climb for platform).
        self.landing_context = 'mission'

        self.home_restored = False
        # Number of DO_SET_HOME sends issued in the current REASSERT_HOME entry.
        # Reset to 0 every time REASSERT_HOME is entered via _transition().
        self.reassert_attempts: int = 0

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
        elif self.state == State.REASSERT_HOME:
            self._tick_reassert_home(tick_count)
        elif self.state == State.CLIMB:
            self._tick_climb(tick_count)
        elif self.state == State.RETURN_TO_ORIGIN:
            self._tick_return_to_origin(tick_count)
        elif self.state == State.RTL:
            self._tick_rtl(tick_count)

    def _transition(self, new_state: State, tick_count: int):
        """Handle state change transitions and resets."""
        logger.info(f"FSM TRANSITION: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_entry_tick = tick_count
        
        # Reset counters on state entry
        self.vision_fail_counter = 0
        self.hold_counter = 0

        # Enable expensive pyzbar decoding only in QR_DECODE state.
        # LAND only needs vis['found']/vis['center'] for LANDING_TARGET angles —
        # both come from qr_det.detect() which always runs regardless of request_decode.
        # Leaving decode on in LAND was running pyzbar at 50Hz for no reason.
        if self.state == State.QR_DECODE:
            self.vision.set_request_decode(True)
        else:
            self.vision.set_request_decode(False)
        self.vision_fail_counter = 0
        self.climb_initiated = False
        if new_state == State.REQUEST_GUIDED:
            self.guided_request_retries = 0
            self.guided_request_counter = 0
        if new_state == State.LAND:
            self.land_request_retries = 0
            self.land_request_counter = 0
            self.land_confirmed = False
            self.takeoff_initiated = False
            self.home_restored = False
        if new_state == State.REASSERT_HOME:
            self.reassert_attempts = 0
            self.home_restored = False
        if new_state == State.CLIMB:
            self.takeoff_initiated = False
            self.takeoff_request_counter = 0
        if new_state == State.RETURN_TO_ORIGIN:
            self.return_land_commanded = False
        # Clear stale anchor on GUIDED_HOLD entry so a prior partial capture
        # from a crashed cycle cannot be reused in a retry scenario.
        if new_state == State.GUIDED_HOLD:
            self.guided_anchor_ned = None
        
        # Trigger controller resets if entering tracking modes
        if new_state == State.ALIGNMENT:
            if self.landing_context == 'platform':
                self.vision.set_detection_mode('platform')
            self.vision.align.reset()
        elif new_state == State.QR_DECODE:
            self.vision.qr_dec.reset()

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

        # Capture true home position ONCE on the first-ever disarmed→armed edge.
        # Edge-trigger (not level-trigger) so a re-arm after QR landing cannot
        # overwrite the reference.  The `is None` guard is the primary invariant;
        # _was_armed prevents the expensive GPS/NED reads from running every tick.
        currently_armed = self.fc.is_armed()
        if self.true_home is None and currently_armed and not self._was_armed:
            gps_fix = self.fc.mav.get_gps_fix_type()
            min_fix = self.cfg.get('flight', {}).get('home_capture_min_gps_fix', 3)

            if gps_fix >= min_fix:
                gps_pos = self.fc.mav.get_global_position()
                ned_pos = self.fc.get_local_position()

                if gps_pos is not None and ned_pos is not None:
                    self.true_home = {
                        'gps': gps_pos,   # (lat_deg, lon_deg, alt_msl_m)
                        'ned': ned_pos,   # (x, y, z) in LOCAL_POSITION_NED frame
                    }
                    lat, lon, alt = gps_pos
                    x, y, z = ned_pos
                    logger.info(
                        f"TRUE HOME CAPTURED — "
                        f"GPS: ({lat:.7f}, {lon:.7f}, {alt:.1f}m MSL) | "
                        f"NED: ({x:.2f}, {y:.2f}, {z:.2f}) | "
                        f"GPS fix: {gps_fix}"
                    )
                    self.fc.mav.send_statustext(
                        f"HOME: {lat:.5f},{lon:.5f} {alt:.0f}m"
                    )
            else:
                # Warn at ~2s cadence so we can see why capture is pending in the log.
                elapsed = tick_count - self.state_entry_tick
                if elapsed % 40 == 0:
                    logger.warning(
                        f"true_home waiting — GPS fix={gps_fix} "
                        f"(need ≥{min_fix}, 3=3D 5=RTK-float 6=RTK-fixed)"
                    )
        self._was_armed = currently_armed

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
            logger.info(
                "GUIDED mode confirmed by autopilot heartbeat. "
                "Entering GUIDED_HOLD to capture LOCAL_POSITION_NED anchor."
            )
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
        """Hold position while waiting for LOCAL_POSITION_NED anchor to be confirmed.

        Retries get_local_position() on every tick.  The FSM only advances to
        INITIAL_SCAN once a non-None anchor is captured AND the minimum settle
        time has elapsed.  If the hard timeout expires with still no telemetry,
        the FSM transitions to RTL with a loud error — never silently storing None.
        """
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        self.fc.hold_position()
        self.hold_counter += 1

        # Attempt anchor capture on every tick until we get a real fix.
        if self.guided_anchor_ned is None:
            pos = self.fc.get_local_position()
            if pos is not None:
                self.guided_anchor_ned = pos
                x0, y0, z0 = pos
                logger.info(
                    f"Anchor captured in GUIDED_HOLD: "
                    f"x0={x0:.2f}, y0={y0:.2f}, z0={z0:.2f} "
                    f"(after {self.hold_counter} ticks)"
                )

        # Hard timeout: configurable, default 8 s.
        # If LOCAL_POSITION_NED never arrives we fail loudly rather than silently.
        anchor_timeout_s = self.cfg.get('flight', {}).get('anchor_capture_timeout_s', 8.0)
        anchor_timeout_ticks = int(anchor_timeout_s * self.cfg['system']['tick_hz'])

        if self.hold_counter >= anchor_timeout_ticks:
            if self.guided_anchor_ned is None:
                logger.error(
                    f"GUIDED_HOLD: LOCAL_POSITION_NED not received after "
                    f"{anchor_timeout_s:.1f}s ({self.hold_counter} ticks). "
                    "Cannot establish position reference — aborting to RTL."
                )
                self.fallback.handle_fail(
                    "GUIDED_HOLD: anchor capture timeout — no LOCAL_POSITION_NED"
                )
                self._transition(State.RTL, tick_count)
            else:
                self._transition(State.INITIAL_SCAN, tick_count)
            return

        # Minimum settle (2 s) after anchor captured, before advancing.
        # This absorbs initial mode-switch momentum before commanding positions.
        min_settle_ticks = int(2.0 * self.cfg['system']['tick_hz'])
        if self.guided_anchor_ned is not None and self.hold_counter >= min_settle_ticks:
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

        vis = self.vision.get_latest_result()
        if vis['timestamp'] == 0.0:
            return

        if vis['found']:
            latency_ms = (time.time() - vis['timestamp']) * 1000
            logger.info(f"Target QR locked at pixel coordinates: {vis['center']} (Latency: {latency_ms:.1f}ms)")
            self._transition(State.ALIGNMENT, tick_count)
            return

    def _generate_search_pattern(self):
        """Generate local NED waypoints for an expanding concentric-square search.

        Produces closed square perimeters at increasing ring sizes, all centered
        on guided_anchor_ned. Each ring is walked SW→SE→NE→NW→SW so the drone
        scans near-to-far before moving outward to the next ring.

        Ring sizes are taken from search.search_rings_m config list.
        Falls back to [1.0, 2.0, square_size_m] when the key is absent.
        """
        anchor = getattr(self, 'guided_anchor_ned', None)
        if not anchor:
            logger.warning("guided_anchor_ned not set! Falling back to current local position.")
            anchor = self.fc.get_local_position()
            if not anchor:
                logger.error("Could not get local position anchor! Defaulting to 0,0,0")
                anchor = (0.0, 0.0, 0.0)

        x0, y0, z0 = anchor

        # Ring sizes: independently tunable list, or derived from square_size_m.
        sq_size = self.cfg['search'].get('square_size_m', 3.0)
        default_rings = [1.0, 2.0, sq_size] if sq_size > 2.0 else [sq_size / 2.0, sq_size]
        ring_sizes = self.cfg['search'].get('search_rings_m', default_rings)

        self.search_waypoints = []
        for ring_size in ring_sizes:
            h = ring_size / 2.0
            # Walk the perimeter clockwise: SW → SE → NE → NW → SW (closed ring)
            self.search_waypoints.append((x0 - h, y0 - h, z0))  # SW
            self.search_waypoints.append((x0 + h, y0 - h, z0))  # SE
            self.search_waypoints.append((x0 + h, y0 + h, z0))  # NE
            self.search_waypoints.append((x0 - h, y0 + h, z0))  # NW
            self.search_waypoints.append((x0 - h, y0 - h, z0))  # SW (close)

        self.current_wp_idx = 0

        # Dynamic timeout: sequential distance sum scaled by speed + margin.
        # Same logic as before — works correctly for any waypoint list shape.
        total_dist = 0.0
        curr_x, curr_y = x0, y0
        for wp_x, wp_y, _ in self.search_waypoints:
            total_dist += math.sqrt((wp_x - curr_x)**2 + (wp_y - curr_y)**2)
            curr_x, curr_y = wp_x, wp_y

        speed_m_s = self.cfg['flight'].get('search_speed_m_s', 0.4)
        margin_s = 15.0  # Slack for per-waypoint settle time and turns
        timeout_s = (total_dist / speed_m_s) + margin_s
        self.search_timeout_ticks = int(timeout_s * self.cfg['system']['tick_hz'])

        logger.info(
            f"Generated {len(self.search_waypoints)} waypoints across "
            f"{len(ring_sizes)} concentric rings {ring_sizes} "
            f"(total path: {total_dist:.1f}m)."
        )
        logger.info(f"Dynamic SEARCH_SQUARE timeout set to {timeout_s:.1f}s.")

        # Apply configured search speed to autopilot before pattern begins.
        # Without this, ArduPilot uses its internal default (3-5 m/s), too fast
        # for the downward camera to acquire a 21cm QR reliably at 5m altitude.
        self.fc.set_search_speed(speed_m_s)

    def _tick_search_square(self, tick_count: int):
        """Execute lawnmower pattern in bounded area if initial scan fails."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        vis = self.vision.get_latest_result()
        if vis['timestamp'] == 0.0:
            self.fc.hold_position()
            self.vision_fail_counter += 1
            if self.vision_fail_counter >= self.vision_fail_limit:
                logger.error("BOUNDED SEARCH ABORT: Vision result unavailable for too long.")
                self.fallback.handle_fail("SEARCH_SQUARE: Vision timeout")
                self._transition(State.RTL, tick_count)
            return

        if vis['found']:
            latency_ms = (time.time() - vis['timestamp']) * 1000
            logger.info(f"Target QR locked at pixel coordinates: {vis['center']} during SEARCH_SQUARE (Latency: {latency_ms:.1f}ms)")
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

        # One-shot: cap speed on the first tick so the return leg is also
        # speed-limited. Must not run every tick — one MAVLink cmd per entry.
        if tick_count == self.state_entry_tick:
            return_speed = self.cfg['flight'].get('search_speed_m_s', 0.4)
            self.fc.set_search_speed(return_speed)

        target_x, target_y, target_z = anchor
        self.fc.goto_local_position(target_x, target_y, target_z)

        current_pos = self.fc.get_local_position()
        if current_pos:
            cx, cy, cz = current_pos
            dist = math.sqrt((target_x - cx)**2 + (target_y - cy)**2)
            tolerance = self.cfg['search'].get('position_tolerance_m', 0.5)

            if dist <= tolerance:
                logger.warning("Returned to initial GUIDED anchor point. Initiating blind LAND sequence.")
                normal_speed = self.cfg['flight'].get('normal_speed_m_s', 3.0)
                self.fc.restore_normal_speed(normal_speed)
                logger.info("Speed restored before LAND transition.")
                self._transition(State.LAND, tick_count)
                return

        # Timeout for the return journey
        elapsed_ticks = tick_count - self.state_entry_tick
        timeout_ticks = 30.0 * self.cfg['system']['tick_hz']
        if elapsed_ticks >= timeout_ticks:
            logger.error("RETURN_INITIAL timeout. Initiating blind LAND sequence anyway.")
            normal_speed = self.cfg['flight'].get('normal_speed_m_s', 3.0)
            self.fc.restore_normal_speed(normal_speed)
            logger.info("Speed restored before LAND transition (timeout path).")
            self._transition(State.LAND, tick_count)

    def _tick_alignment(self, tick_count: int):
        """Compute PID centering adjustments and guide the drone over the target center."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        vis = self.vision.get_latest_result()
        if vis['timestamp'] == 0.0:
            self.fc.hold_position()
            self.vision_fail_counter += 1
            if self.vision_fail_counter >= self.vision_fail_limit:
                logger.error("ALIGNMENT ABORT: Vision result unavailable for too long.")
                self.fallback.handle_fail("ALIGNMENT: Vision timeout")
                self._transition(State.RTL, tick_count)
            return

        if not vis['found']:
            self.vision_fail_counter += 1
            if self.vision_fail_counter > 40:  # Lost for 2 seconds
                logger.warning("Lost target track during alignment. Re-searching...")
                self._transition(State.GUIDED_HOLD, tick_count)
            else:
                self.fc.hold_position()
            return

        self.vision_fail_counter = 0

        # RTK GPS Cross-Check for Vision Hallucinations (mission delivery only)
        if self.landing_context == 'mission' and self.fc.distance_to_wp() > 5.0:
            logger.error("RTK vs Vision drift mismatch! Target > 5m from GPS waypoint. Aborting.")
            self.fallback.handle_fail("RTK/Vision drift mismatch")
            self._transition(State.RTL, tick_count)
            return

        # Use pre-computed velocities from vision pipeline
        vx, vy, aligned = vis['vx'], vis['vy'], vis['aligned']
        
        # Command horizontal adjustment with slow landing descent (vz = 0.1m/s)
        self.fc.send_velocity(vx, vy, vz=0.1)

        # Transition logic
        if aligned:
            if self.landing_context == 'mission':
                logger.info("Centering stability target reached. Initiating QR Decoding payload parse...")
                self._transition(State.QR_DECODE, tick_count)
            else:
                logger.info("Platform centering stable. Initiating precision landing.")
                self._transition(State.LAND, tick_count)

    def _tick_qr_decode(self, tick_count: int):
        """Command hover and parse QR text contents."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        self.fc.hold_position()
        
        vis = self.vision.get_latest_result()
        if vis['timestamp'] == 0.0:
            return

        # Check if background thread successfully decoded it
        success = vis['decode_success']
        text = vis['decode_text']
        final = vis['decode_final']
        
        if success:
            logger.info(f"Target Payload Decoded: '{text}'")
            self.fc.send_qr_text(text)
            self._transition(State.LAND, tick_count)
        elif final:
            logger.warning("Failed to decode target payload. Initiating landing sequence anyway.")
            self._transition(State.LAND, tick_count)

    def _send_landing_target(self, vis: dict):
        """Compute and send LANDING_TARGET angles from vision detection result."""
        frame_shape = vis['frame'].shape if vis['frame'] is not None else (1080, 1920)
        h, w = frame_shape[:2]
        cx, cy = vis['center']
        img_cx, img_cy = w / 2.0, h / 2.0
        err_x_px = cx - img_cx
        err_y_px = cy - img_cy
        
        fov_rad = math.radians(self.cfg['camera'].get('fov_horizontal_deg', 66.0))
        focal_length_px = (w / 2.0) / math.tan(fov_rad / 2.0)
        
        # Image +Y is down (backward in standard mounting), Image +X is right
        angle_x = math.atan(err_y_px / focal_length_px)
        angle_y = math.atan(err_x_px / focal_length_px)
        
        dist_m = self.fc.mav.get_altitude()
        self.fc.send_landing_target(angle_x, angle_y, dist_m)

    def _tick_land(self, tick_count: int):
        """Execute landing and trigger distance-sensor gated payload drops."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        ticks_in_state = tick_count - self.state_entry_tick
        
        # 1. Retry-and-confirm logic for LAND command
        if not self.land_confirmed:
            # Throttle command to once per second (every 20 ticks at 20Hz)
            if tick_count % 20 == 0:
                logger.info(f"Commanding vehicle precision landing (attempt {self.land_request_retries + 1})...")
                self.fc.land()
            
            self.land_request_counter += 1
            
            # Check mode confirmation using autopilot-specific HEARTBEAT
            if self.fc.is_land_mode():
                logger.info("LAND mode confirmed by autopilot heartbeat.")
                self.land_confirmed = True
                self.land_request_counter = 0
            else:
                # Retry timeout logic (max 3 cycles of 5 seconds = 15 seconds)
                if self.land_request_counter > 100:
                    self.land_request_retries += 1
                    logger.warning(f"LAND mode request timeout (attempt {self.land_request_retries}/3).")
                    self.land_request_counter = 0

                    if self.land_request_retries >= 3:
                        logger.error("LAND mode request failed after 3 retries (15s). Aborting to RTL.")
                        self.fallback.handle_fail("LAND: max retries exceeded")
                        self._transition(State.RTL, tick_count)
                        return
                
                # If not confirmed, do not proceed with the rest of the tick (payload checks etc)
                return

        # Log altitude every second (20 ticks) to track descent progress
        if tick_count % 20 == 0:
            current_alt = self.fc.mav.get_altitude()
            logger.info(f"LAND mode descent: Current relative altitude = {current_alt:.2f}m")

        # Closed-loop tracking via LANDING_TARGET messages
        vis = self.vision.get_latest_result()
        if vis['timestamp'] != 0.0 and vis['found']:
            # RTK vs Vision cross check (mission only)
            if self.landing_context == 'mission' and self.fc.distance_to_wp() > 5.0:
                logger.error("RTK vs Vision drift mismatch during landing! Aborting.")
                self.fallback.handle_fail("RTK/Vision drift mismatch")
                self._transition(State.RTL, tick_count)
                return
            
            self._send_landing_target(vis)

        # Hard landing timeout - only guards pre-release: if the drop window is never
        # reached (sensor failure, drift), abort. Once released, post-release climb is
        # in progress — don't kill it with this timer.
        if ticks_in_state > 600 and not self.payload.payload_released:  # 30s at 20 Hz
            logger.error("LAND TIMEOUT: Release window not reached in 30s. Triggering RTL.")
            self.fallback.handle_fail("LAND timeout: release window never hit")
            self._transition(State.RTL, tick_count)
            return

        # Platform landing: no payload release — just precision descent → disarm
        if self.landing_context == 'platform':
            if self.fc.is_landed():
                if self.state != State.MISSION_COMPLETE:
                    logger.info("PLATFORM LANDING COMPLETE. Vehicle is on the ground.")
                    self.vision.set_detection_mode('qr')  # restore default
                    self.fc.mav.send_statustext("MISSION COMPLETE: Platform landing confirmed")
                    self._transition(State.MISSION_COMPLETE, tick_count)  # new terminal state, ticks silently
            return

        # Altitude drop gate checks (only if payload not released yet)
        if not self.payload.payload_released:
            dist = self.payload.get_distance_reading()
            
            in_window = self.payload.is_in_release_window(dist)
            # SITL FIX: In SITL we don't have a real ultrasonic sensor, so dist (relative alt)
            # drops to 0.0m upon landing, missing the [0.2, 0.4] release window entirely.
            if self.payload.use_sitl and self.fc.is_landed():
                in_window = True
                
            if in_window and self.fc.is_landed():
                # Double safety arming check
                if self.payload.takeoff_detected:
                    logger.warning(f"Safe release altitude window met: {dist:.3f}m. Releasing payload...")
                    self.payload.trigger_release()
                else:
                    logger.warning(f"Drop altitude met ({dist:.3f}m) but takeoff arming safety gate is active. Aborting.")
        
        # Once payload released: switch to GUIDED → re-arm → REASSERT_HOME → CLIMB → RTL/return
        if self.payload.payload_released:
            # Step 1: Switch to GUIDED mode first
            if not self.fc.is_guided_mode():
                if tick_count % 20 == 0:
                    logger.info("Switching to GUIDED mode for post-release return sequence...")
                    self.fc.set_guided_mode()
                return  # Wait for mode confirmation

            # Step 2: Re-arm (ArduCopter auto-disarms on touchdown)
            if not self.fc.is_armed():
                if tick_count % 20 == 0:
                    logger.info("Drone disarmed after landing. Re-arming for return climb...")
                    self.fc.arm()
                return  # Wait for arm confirmation via heartbeat

            # Step 3: Re-arm confirmed — assert home BEFORE any takeoff.
            # ArduCopter just reset home to the QR pad on re-arm; REASSERT_HOME
            # will overwrite it with true_home and confirm via HOME_POSITION broadcast.
            logger.info("Re-arm confirmed. Entering REASSERT_HOME to restore launch point.")
            self._transition(State.REASSERT_HOME, tick_count)


    def _tick_reassert_home(self, tick_count: int):
        """Re-assert true home after re-arm, confirmed via HOME_POSITION broadcast.

        ArduCopter resets home to the current (QR pad) location on every arm cycle.
        This state corrects it back to the original launch point captured in
        true_home before any takeoff.

        Strategy (non-blocking, no direct socket access):
          - Sends MAV_CMD_DO_SET_HOME via COMMAND_INT (int32 lat/lon) once per second.
          - Polls HOME_POSITION from the message cache (streamed at 0.5 Hz via
            _request_targeted_streams). Tolerance: ±5 degE7 ≈ 0.5 µdeg ≈ <0.1 m.
          - After 3 failed sends without cache confirmation → FallbackManager + RTL.
            An unconfirmed home means RTL would fly to the QR pad, which is worse
            than not flying at all.
        """
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        if self.true_home is None:
            logger.error(
                "REASSERT_HOME: true_home is None — cannot restore launch point. "
                "Triggering fallback (RTL will target wrong location)."
            )
            self.fallback.handle_fail("REASSERT_HOME: true_home not captured")
            self._transition(State.RTL, tick_count)
            return

        lat_deg, lon_deg, alt_m = self.true_home['gps']
        lat_e7 = int(lat_deg * 1e7)
        lon_e7 = int(lon_deg * 1e7)

        # Send set_home_precise once per second (every 20 ticks at 20 Hz).
        if tick_count % 20 == 0:
            self.reassert_attempts += 1
            logger.info(
                f"REASSERT_HOME: sending DO_SET_HOME attempt {self.reassert_attempts}/3 "
                f"({lat_deg:.7f}\u00b0, {lon_deg:.7f}\u00b0, {alt_m:.1f}m) ..."
            )
            self.fc.set_home_precise(lat_deg, lon_deg, alt_m)

        # Poll HOME_POSITION from the message cache — never read directly from the socket.
        # ArduPilot broadcasts HOME_POSITION after every successful home change.
        hp = self.fc.mav.get_message('HOME_POSITION')
        if hp is not None:
            lat_match = abs(hp.latitude  - lat_e7) < 5   # ±5 degE7 ≈ <0.1 m
            lon_match = abs(hp.longitude - lon_e7) < 5
            if lat_match and lon_match:
                self.home_restored = True
                logger.info(
                    f"REASSERT_HOME: HOME_POSITION confirmed \u2713 "
                    f"hp.lat={hp.latitude} hp.lon={hp.longitude} "
                    f"(target lat_e7={lat_e7} lon_e7={lon_e7})"
                )
                self.fc.mav.send_statustext(
                    f"HOME OK: {lat_deg:.5f},{lon_deg:.5f} {alt_m:.0f}m"
                )
                self._transition(State.CLIMB, tick_count)
                return

        # After 3 sends (≈3 s) without a matching HOME_POSITION: abort.
        # Proceeding with the wrong home is a safety hazard — RTL would land on the QR pad.
        if self.reassert_attempts >= 3 and not self.home_restored:
            logger.error(
                "REASSERT_HOME: HOME_POSITION not confirmed after 3 attempts. "
                "RTL would target wrong location — triggering fallback."
            )
            self.fallback.handle_fail("REASSERT_HOME: HOME_POSITION confirm failed after 3 attempts")
            self._transition(State.RTL, tick_count)

    def _tick_climb(self, tick_count: int):
        """Command takeoff and climb to post-release altitude, then hand off to RETURN_TO_ORIGIN.

        Extracted from old _tick_land() Steps 3 and 4.  Entered only after
        REASSERT_HOME confirms home is correctly set to the true launch point.
        """
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        climb_alt = self.cfg['search'].get('post_release_climb_altitude_m', 3.0)
        current_alt = self.fc.mav.get_altitude()

        # Step A: command takeoff, wait for confirmed ascent
        if not self.takeoff_initiated:
            if tick_count % 20 == 0:
                logger.info(f"CLIMB: commanding takeoff to {climb_alt}m ...")
                self.fc.takeoff(climb_alt)
                self.takeoff_request_counter += 1

            # Confirm ascent: at least 3 takeoff commands sent AND altitude is rising
            if self.takeoff_request_counter > 2 and current_alt > 0.5:
                self.takeoff_initiated = True
                logger.info("CLIMB: takeoff confirmed, ascending.")
            return

        # Step B: push upward until target altitude reached, then hand off
        if current_alt < climb_alt - 0.3:
            self.fc.send_velocity(0.0, 0.0, -0.5)  # -Z = up in NED
        else:
            logger.info(
                f"CLIMB: target altitude {climb_alt}m reached "
                f"(current={current_alt:.2f}m). Transitioning to RETURN_TO_ORIGIN."
            )
            self._transition(State.RETURN_TO_ORIGIN, tick_count)

    def _tick_return_to_origin(self, tick_count: int):
        """Fly back to the initial takeoff home anchor and land there.

        Uses true_home['ned'] (captured on first arm in MONITOR_AUTO) as the target.
        This is GPS-home-agnostic, immune to any ArduCopter home location
        updates triggered during the QR landing or re-arm.
        """
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        # Once land is commanded (arrival or timeout), just hold and wait
        if self.return_land_commanded:
            if tick_count % 40 == 0:
                logger.info("RETURN_TO_ORIGIN: Final landing in progress...")
            return

        anchor = self.true_home['ned'] if self.true_home else None
        if anchor is None:
            logger.error(
                "RETURN_TO_ORIGIN: true_home is None — no home reference was captured. "
                "Commanding emergency land at current position."
            )
            self.fc.land()
            self.return_land_commanded = True
            return

        target_x, target_y, _ = anchor
        # Fly back at the climb altitude rather than descending diagonally to ground (z=0)
        climb_alt = self.cfg['search'].get('post_release_climb_altitude_m', 3.0)
        target_z = -climb_alt  # NED z is negative
        self.fc.goto_local_position(target_x, target_y, target_z)

        # Check arrival tolerance (horizontal only — altitude locked by NED setpoint)
        current_pos = self.fc.get_local_position()
        if current_pos:
            cx, cy, _ = current_pos
            dist = math.sqrt((target_x - cx) ** 2 + (target_y - cy) ** 2)
            tolerance = self.cfg['search'].get('position_tolerance_m', 0.5)
            if dist <= tolerance:
                if self.landing_context == 'mission':
                    logger.info(
                        f"Arrived at initial home anchor (dist={dist:.2f}m ≤ {tolerance}m). "
                        "Handing off to GUIDED for platform precision landing."
                    )
                    self.landing_context = 'platform'
                    self.vision.set_detection_mode('platform')
                    self._transition(State.REQUEST_GUIDED, tick_count)
                else:
                    logger.info(
                        f"Arrived at initial home anchor (dist={dist:.2f}m ≤ {tolerance}m). "
                        "Commanding final land."
                    )
                    self.fc.land()
                    self.return_land_commanded = True
                return

        # 30-second timeout fallback — land wherever we are
        elapsed_ticks = tick_count - self.state_entry_tick
        timeout_ticks = int(30.0 * self.cfg['system']['tick_hz'])
        if elapsed_ticks >= timeout_ticks:
            logger.warning(
                "RETURN_TO_ORIGIN timeout (30s). Landing at current position as fallback."
            )
            self.fc.land()
            self.return_land_commanded = True

    def _tick_rtl(self, tick_count: int):
        """Maintain Return-To-Launch state loop with optional proximity handoff."""
        if not self.fc.mav.is_connected():
            self._transition(State.BOOT, tick_count)
            return

        ticks_in_state = tick_count - self.state_entry_tick
        if ticks_in_state == 0:
            logger.info("Commanding vehicle Return To Launch...")
            self.fc.rtl()

        # If this RTL was triggered from the post-release flow (mission context),
        # monitor for home proximity and hand off to precision landing.
        if self.landing_context == 'mission' and self.true_home is not None:
            current_pos = self.fc.get_local_position()
            if current_pos:
                home_ned = self.true_home['ned']
                dx = current_pos[0] - home_ned[0]
                dy = current_pos[1] - home_ned[1]
                horiz_dist = math.sqrt(dx**2 + dy**2)
                alt = self.fc.mav.get_altitude()
                
                handoff_dist = self.cfg.get('platform', {}).get('rtl_handoff_distance_m', 5.0)
                handoff_alt = self.cfg.get('platform', {}).get('rtl_handoff_altitude_m', 8.0)
                
                if tick_count % 40 == 0:
                    logger.info(f"RTL monitoring: dist={horiz_dist:.1f}m, alt={alt:.1f}m (target dist<{handoff_dist}, alt<{handoff_alt})")
                
                if horiz_dist < handoff_dist and alt < handoff_alt:
                    logger.info(
                        f"RTL near home (dist={horiz_dist:.1f}m, alt={alt:.1f}m). "
                        "Handing off to GUIDED for platform precision landing."
                    )
                    # Switch context to platform and re-enter the alignment pipeline
                    self.landing_context = 'platform'
                    self.vision.set_detection_mode('platform')
                    self._transition(State.REQUEST_GUIDED, tick_count)
                    return

            # Safety timeout — if proximity never triggers, RTL lands normally (GPS-level)
            timeout_s = self.cfg.get('platform', {}).get('rtl_handoff_timeout_s', 60.0)
            if ticks_in_state > int(timeout_s * self.cfg['system']['tick_hz']):
                logger.warning("RTL handoff timeout. ArduCopter will complete blind RTL landing.")
