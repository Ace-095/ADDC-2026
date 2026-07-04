"""MAVLink interface handler supporting auto-reconnect, serial discovery, and SITL links."""

import logging
import threading
import time
import glob
from pathlib import Path
# pyrefly: ignore [missing-import]
from pymavlink import mavutil
from typing import Optional, Tuple, Any

logger = logging.getLogger(__name__)


class MAVLinkInterface:
    """Thread-safe connection driver for autopilot telemetry and guidance."""

    def __init__(self, baud: int = 57600, heartbeat_timeout_ticks: int = 60,
                 reconnect_delay_ticks: int = 100, use_sitl: bool = False,
                 connection_string: str = "/dev/serial0"):
        self.baud = baud
        self.heartbeat_timeout_ticks = heartbeat_timeout_ticks
        self.reconnect_delay_ticks = reconnect_delay_ticks   # Back-off ticks between reconnect attempts
        self.use_sitl = use_sitl
        self.connection_string = connection_string
        
        self._conn = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._last_heartbeat_tick = 0
        self._tick_counter = 0
        self._connected = False

        # Telemetry Cache
        self.messages = {}
        self.mission_items = {}
        self.current_waypoint = -1
        self.last_mission_item_reached = -1

        # Autopilot-specific HEARTBEAT cache (sysid=1 only).
        # Kept separate so GCS HEARTBEATs (sysid=255, custom_mode=0)
        # cannot overwrite the autopilot's mode confirmation.
        self._autopilot_hb = None
        # Phase D: track local wall-clock time of most recent autopilot HB receipt.
        # Used to detect stale HBs before trusting custom_mode.
        self._autopilot_hb_recv_time: float = 0.0

        # Set to True when autopilot STATUSTEXT announces sprayer execution.
        # Consumed (reset) by the FSM after it acts on it.
        self.sprayer_detected = False

        # Phase A: per-type latency tracking.
        # Stores {msg_type: (count, sum_lag_ms, max_lag_ms)} for debug reporting.
        self._latency_stats: dict = {}
        self._conn_start_time: float = 0.0   # wall-clock when connection was established
        self._vehicle_boot_ms_at_connect: int = 0  # first time_boot_ms seen after connect

    def start(self):
        """Start the background telemetry parsing and connection loop."""
        self._running = True
        self._thread = threading.Thread(target=self._connection_loop, daemon=True)
        self._thread.start()
        logger.info("MAVLink interface connection thread started.")

    def stop(self):
        """Stop the MAVLink thread and disconnect the serial/socket link."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        with self._lock:
            if self._conn:
                try:
                    self._conn.close()
                except:
                    pass
                self._conn = None
            self._connected = False
        logger.info("MAVLink interface connection thread stopped.")

    def is_connected(self) -> bool:
        """Check if connection is alive and receiving heartbeat ticks."""
        with self._lock:
            ticks_since_hb = self._tick_counter - self._last_heartbeat_tick
            return self._connected and ticks_since_hb < self.heartbeat_timeout_ticks

    def tick(self):
        """Increment tick counter (call from main loop at HZ frequency)."""
        with self._lock:
            self._tick_counter += 1

    def set_guided_velocity(self, vx: float, vy: float, vz: float) -> bool:
        """
        Send a velocity vector command in the vehicle body frame (NED).

        Args:
            vx: Forward velocity (m/s)
            vy: Right velocity (m/s)
            vz: Down velocity (m/s)
        """
        with self._lock:
            if not self._conn or not self._connected:
                return False
            try:
                # Type mask 0x01C7 = 455
                # bitmask: ignore pos (1|2|4), ignore acc (64|128|256)
                self._conn.mav.set_position_target_local_ned_send(
                    0, 
                    self._conn.target_system, 
                    self._conn.target_component, 
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b000001_11_000_111,  
                    0.0, 0.0, 0.0,
                    vx, vy, vz,
                    0.0, 0.0, 0.0,
                    0.0, 0.0
                )
                return True
            except Exception as e:
                logger.error(f"Failed to send velocity: {e}")
                return False

    def set_position_target_local_ned(self, x: float, y: float, z: float) -> bool:
        """
        Send a local NED position setpoint command to ArduPilot.
        
        Uses MAV_FRAME_LOCAL_NED so positions are relative to EKF origin
        and do not rotate with vehicle heading.
        """
        with self._lock:
            if not self._conn or not self._connected:
                return False
            try:
                # bitmask: ignore velocity (8|16|32), ignore acc (64|128|256), ignore yaw (1024|2048)
                # 0b000011_01_111_000 = 3576
                self._conn.mav.set_position_target_local_ned_send(
                    0, 
                    self._conn.target_system, 
                    self._conn.target_component, 
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b000011_01_111_000,  
                    x, y, z,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0
                )
                return True
            except Exception as e:
                logger.error(f"Failed to send position target: {e}")
                return False

    def get_autopilot_heartbeat(self):
        """
        Return the latest HEARTBEAT received from the autopilot (sysid=1).
        This is stored separately from the generic messages cache so that
        GCS HEARTBEATs cannot overwrite the autopilot mode confirmation.
        Returns:
            The latest autopilot HEARTBEAT message, or None if not yet received.
        """
        with self._lock:
            return self._autopilot_hb

    def get_autopilot_hb_age(self) -> float:
        """Return how many seconds ago the last autopilot HEARTBEAT was received.

        Used by the FSM (Phase D) to reject stale HB readings before trusting
        custom_mode. Returns inf if no HB has ever been received.
        """
        with self._lock:
            t = self._autopilot_hb_recv_time
        if t == 0.0:
            return float('inf')
        return time.time() - t

    def _request_targeted_streams(self) -> None:
        """Kill the MAV_DATA_STREAM_ALL flood, then request only consumed messages.

        Step 1: Explicitly stop ALL data streams (rate=0). Without this step,
        ArduPilot keeps blasting every stream group regardless of SET_MESSAGE_INTERVAL.

        Step 2: Request individual messages at rates matched to actual usage:
          GLOBAL_POSITION_INT       4 Hz  (altitude gate, GPS position)
          MISSION_CURRENT           4 Hz  (waypoint tracking)
          EXTENDED_SYS_STATE        2 Hz  (landed state check)
          VFR_HUD                   2 Hz  (throttle for landing detection)
          NAV_CONTROLLER_OUTPUT     2 Hz  (wp_dist for distance check)

        HEARTBEAT: intentionally NOT set here - ArduPilot's default 1Hz is correct
        and re-requesting it can interfere with connection watchdog timing.

        MISSION_ITEM_REACHED / STATUSTEXT: event-driven, no polling needed.
        """
        try:
            with self._lock:
                if not self._conn:
                    return
                conn = self._conn

            # Step 1: Stop all streams first so we start from a clean slate.
            # Without this, MAV_DATA_STREAM_ALL keeps running alongside our targeted requests.
            try:
                conn.mav.request_data_stream_send(
                    conn.target_system,
                    conn.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    0,   # rate = 0
                    0    # start_stop = 0 (stop)
                )
                logger.debug("MAV_DATA_STREAM_ALL stopped (rate=0).")
            except Exception as e:
                logger.debug(f"Stream stop send failed: {e}")

            # Step 2: Request only what we actually consume via SET_MESSAGE_INTERVAL.
            def _set_interval(msg_id: int, rate_hz: float) -> None:
                interval_us = int(1_000_000 / rate_hz) if rate_hz > 0 else -1
                conn.mav.command_long_send(
                    conn.target_system,
                    conn.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    float(msg_id),
                    float(interval_us),
                    0, 0, 0, 0, 0
                )

            _set_interval(33,  4.0)   # GLOBAL_POSITION_INT
            _set_interval(32,  4.0)   # LOCAL_POSITION_NED
            _set_interval(42,  4.0)   # MISSION_CURRENT
            _set_interval(245, 2.0)   # EXTENDED_SYS_STATE
            _set_interval(74,  2.0)   # VFR_HUD
            _set_interval(62,  2.0)   # NAV_CONTROLLER_OUTPUT
            logger.info("Targeted MAVLink streams configured: ALL stopped, 5 messages at low rates.")
        except Exception as e:
            logger.warning(f"Failed to configure targeted streams: {e}")

    def get_message(self, msg_type: str) -> Optional[Any]:
        """Get the latest cached telemetry packet of the given MAVLink type."""
        with self._lock:
            return self.messages.get(msg_type)

    def get_altitude(self) -> float:
        """Retrieve relative altitude above takeoff in meters."""
        msg = self.get_message('GLOBAL_POSITION_INT')
        if msg:
            return float(msg.relative_alt) / 1000.0
        return 0.0

    def get_local_position_ned(self) -> Optional[Tuple[float, float, float]]:
        """Retrieve the current local position (x, y, z) in meters."""
        msg = self.get_message('LOCAL_POSITION_NED')
        if msg:
            return (float(msg.x), float(msg.y), float(msg.z))
        return None

    def get_gps_fix_type(self) -> int:
        """Return the current GPS fix quality from GPS_RAW_INT telemetry.

        Fix type values (matches MAVLink GPS_FIX_TYPE enum):
            0 = No GPS
            1 = No Fix
            2 = 2D Fix
            3 = 3D Fix
            4 = DGPS
            5 = RTK Float
            6 = RTK Fixed

        Returns:
            int: fix_type value, or 0 if GPS_RAW_INT has not yet been received.
        """
        msg = self.get_message('GPS_RAW_INT')
        if msg:
            return int(msg.fix_type)
        return 0

    def send_command_long(self, command: int, param1: float = 0.0, param2: float = 0.0,
                          param3: float = 0.0, param4: float = 0.0, param5: float = 0.0,
                          param6: float = 0.0, param7: float = 0.0) -> bool:
        """Send a standard command long frame to the vehicle autopilot."""
        with self._lock:
            if not self._conn or not self._connected:
                return False
            try:
                self._conn.mav.command_long_send(
                    self._conn.target_system,
                    self._conn.target_component,
                    command,
                    0,  # confirmation
                    param1, param2, param3, param4, param5, param6, param7
                )
                return True
            except Exception as e:
                logger.error(f"Failed to send MAV_CMD ({command}): {e}")
                return False

    def send_statustext(self, text: str, severity: int = 6) -> bool:
        """
        Send a text notification to the GCS console via MAVLink STATUSTEXT.

        Args:
            text: Information text (up to 50 characters)
            severity: MAV_SEVERITY level (default 6 = MAV_SEVERITY_INFO)
        """
        with self._lock:
            if not self._conn or not self._connected:
                return False
            try:
                # STATUSTEXT payload is exactly 50 bytes, null-padded
                msg_truncated = text[:50]
                msg_bytes = msg_truncated.encode('utf-8')[:50].ljust(50, b'\0')
                self._conn.mav.statustext_send(severity, msg_bytes)
                logger.info(f"Sent STATUSTEXT to GCS: {msg_truncated}")
                return True
            except Exception as e:
                logger.error(f"Failed to send STATUSTEXT message: {e}")
                return False

    def send_landing_target(self, angle_x: float, angle_y: float, distance: float) -> bool:
        """Send a LANDING_TARGET message for closed-loop precision landing."""
        with self._lock:
            if not self._conn or not self._connected:
                return False
            try:
                self._conn.mav.landing_target_send(
                    0,  # time_usec
                    0,  # target_num
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    angle_x,
                    angle_y,
                    distance,
                    0,  # size_x
                    0   # size_y
                )
                return True
            except Exception as e:
                logger.error(f"Failed to send LANDING_TARGET message: {e}")
                return False

    def request_mission_item(self, seq: int) -> bool:
        """Request details for a specific mission waypoint sequence number."""
        with self._lock:
            if not self._conn or not self._connected:
                return False
            try:
                self._conn.mav.mission_request_int_send(
                    self._conn.target_system,
                    self._conn.target_component,
                    seq
                )
                return True
            except Exception as e:
                logger.debug(f"Failed to request mission item sequence {seq}: {e}")
                return False

    def _discover_port(self) -> Optional[Any]:
        """Establish MAVLink connections depending on SITL or real serial modes."""
        if self.use_sitl:
            try:
                logger.info(f"Connecting to SITL endpoint: {self.connection_string}")
                conn = mavutil.mavlink_connection(self.connection_string)
                conn.wait_heartbeat(timeout=5.0)
                logger.info("MAVLink connection established with SITL vehicle.")
                return conn
            except Exception as e:
                logger.error(f"SITL connection failed: {e}")
                return None

        # Real hardware port discovery
        candidates = []
        if Path('/dev/pixhawk').exists():
            candidates.append('/dev/pixhawk')
        
        candidates.extend(glob.glob('/dev/serial/by-id/*'))
        candidates.extend(sorted(glob.glob('/dev/ttyACM*')))
        candidates.extend(sorted(glob.glob('/dev/ttyUSB*')))
        candidates.append(self.connection_string)
        
        # Deduplicate candidates
        seen = set()
        unique_candidates = [x for x in candidates if not (x in seen or seen.add(x))]

        for port in unique_candidates:
            if not port or (port.startswith('/dev/') and not Path(port).exists()):
                continue
            try:
                logger.info(f"Trying to connect to serial port: {port} at {self.baud} baud")
                conn = mavutil.mavlink_connection(port, baud=self.baud)
                conn.wait_heartbeat(timeout=3.0)
                logger.info(f"Connected successfully to Pixhawk on {port}")
                return conn
            except Exception as e:
                logger.debug(f"Connection failed on {port}: {e}")
                continue
        
        return None

    def _connection_loop(self):
        """Maintain telemetry streams, read incoming packets, and handle reconnections."""
        reconnect_ticks = 0
        last_data_stream_req = 0
        
        while self._running:
            # Check connection state
            with self._lock:
                if self._conn is None:
                    self._connected = False
            
            # Reconnect routine
            if self._conn is None:
                if reconnect_ticks <= 0:
                    logger.info("Searching for vehicle connection...")
                    conn = self._discover_port()
                    if conn:
                        with self._lock:
                            self._conn = conn
                            self._connected = True
                            self._last_heartbeat_tick = self._tick_counter
                            # Identify as onboard computer so STATUSTEXT appears in Mission Planner
                            self._conn.mav.srcSystem = self._conn.target_system
                            self._conn.mav.srcComponent = 191  # MAV_COMP_ID_ONBOARD_COMPUTER (not 190 which is MAV_COMP_ID_MISSIONPLANNER)
                        
                        # Phase C: replace MAV_DATA_STREAM_ALL blast with targeted
                        # per-message intervals to reduce link saturation.
                        # Only request messages we actually consume.
                        self._request_targeted_streams()
                        self._conn_start_time = time.time()
                        self._vehicle_boot_ms_at_connect = 0
                        reconnect_ticks = 0
                    else:
                        logger.warning("No autopilot link established. Retrying...")
                        reconnect_ticks = self.reconnect_delay_ticks
                else:
                    reconnect_ticks -= 1
                    time.sleep(0.05)
                    continue

            # Phase C: keep-alive refresh for targeted streams (every 10 seconds).
            current_time = time.time()
            if self._connected and (current_time - last_data_stream_req > 10.0):
                self._request_targeted_streams()
                last_data_stream_req = current_time

            # Read incoming messages
            try:
                msg = None
                with self._lock:
                    if self._conn:
                        msg = self._conn.recv_match(blocking=False)
                
                if msg:
                    recv_wall = time.time()
                    msg_type = msg.get_type()
                    with self._lock:
                        self.messages[msg_type] = msg

                    # Phase A: latency instrumentation.
                    # Compare vehicle time_boot_ms with our local wall-clock estimate.
                    try:
                        tboot = getattr(msg, 'time_boot_ms', None)
                        if tboot is not None and self._conn_start_time > 0:
                            if self._vehicle_boot_ms_at_connect == 0:
                                # First message - calibrate the offset.
                                self._vehicle_boot_ms_at_connect = tboot
                                self._conn_start_time = recv_wall
                            else:
                                vehicle_elapsed_ms = tboot - self._vehicle_boot_ms_at_connect
                                local_elapsed_ms = (recv_wall - self._conn_start_time) * 1000.0
                                lag_ms = local_elapsed_ms - vehicle_elapsed_ms
                                stats = self._latency_stats.get(msg_type, (0, 0.0, 0.0))
                                n, s, mx = stats
                                n += 1
                                s += lag_ms
                                mx = max(mx, lag_ms)
                                self._latency_stats[msg_type] = (n, s, mx)
                                logger.debug(
                                    f"[LAT] {msg_type} lag={lag_ms:+.0f}ms "
                                    f"avg={s/n:.0f}ms max={mx:.0f}ms n={n}"
                                )
                    except Exception:
                        pass
                    
                    # Track HEARTBEAT ticks - store autopilot HB separately by sysid.
                    if msg_type == 'HEARTBEAT':
                        with self._lock:
                            self._last_heartbeat_tick = self._tick_counter
                            self._connected = True

                        # Determine source sysid so GCS HBs (sysid=255, custom_mode=0)
                        # don't corrupt autopilot mode detection.
                        try:
                            hdr = getattr(msg, '_header', None)
                            if hdr is not None:
                                src_id = hdr.srcSystem
                            else:
                                # pymavlink older API fallback
                                src_id = getattr(msg, 'get_srcSystem', None)
                                src_id = src_id() if callable(src_id) else 1
                        except Exception:
                            src_id = 1  # assume autopilot if we can't determine

                        logger.debug(
                            f"[HB] sysid={src_id} custom_mode={getattr(msg, 'custom_mode', '?')} "
                            f"base_mode={getattr(msg, 'base_mode', '?')}"
                        )

                        # Accept HBs from autopilot (sysid=1) or any non-GCS source
                        # (GCS is typically 255; values >200 are likely GCS/MAVProxy).
                        # We use the custom_mode from the LOWEST sysid we see,
                        # which is almost always the autopilot.
                        if src_id < 200:
                            with self._lock:
                                self._autopilot_hb = msg
                                self._autopilot_hb_recv_time = time.time()

                    # Track current waypoints
                    elif msg_type == 'MISSION_CURRENT':
                        wp = msg.seq
                        if wp != self.current_waypoint:
                            self.current_waypoint = wp
                            logger.info(f"Autopilot Waypoint updated: {wp}")
                            # Phase C: removed aggressive request_mission_item() round-trips.
                            # The sprayer trigger now uses STATUSTEXT; mission item cache
                            # is populated lazily by the FSM only when it needs to inspect.

                    # Track reached waypoints
                    elif msg_type == 'MISSION_ITEM_REACHED':
                        reached = msg.seq
                        self.last_mission_item_reached = reached
                        logger.info(f"Autopilot Mission Item reached: {reached}")
                        
                    # Receive requested mission items
                    elif msg_type == 'MISSION_ITEM_INT':
                        seq = msg.seq
                        cmd = msg.command
                        with self._lock:
                            self.mission_items[seq] = cmd
                        logger.debug(f"Waypoint {seq} command cached: {cmd}")
                    
                    # Listen for autopilot STATUSTEXT announcing sprayer execution.
                    # ArduPilot broadcasts e.g. "Mission: 4 Sprayer" when DO_SPRAYER fires.
                    # This is the most reliable trigger - no MISSION_ITEM_INT round-trip needed.
                    elif msg_type == 'STATUSTEXT':
                        try:
                            # Only react to messages from the autopilot (sysid=1), not GCS/ourselves
                            src_sysid = getattr(msg, '_header', None)
                            src_sysid = src_sysid.srcSystem if src_sysid else getattr(msg, 'get_srcSystem', lambda: 1)()
                            raw = msg.text
                            # text field may be bytes or str depending on pymavlink version
                            text = raw.decode('utf-8', errors='ignore').strip() if isinstance(raw, (bytes, bytearray)) else str(raw).strip()
                            logger.debug(f"Autopilot STATUSTEXT (sys={src_sysid}): '{text}'")
                            if src_sysid == 1 and 'sprayer' in text.lower():
                                logger.warning(f"🎯 Sprayer STATUSTEXT detected from autopilot: '{text}'")
                                with self._lock:
                                    self.sprayer_detected = True
                        except Exception as e:
                            logger.debug(f"STATUSTEXT parse error: {e}")


            except Exception as e:
                logger.error(f"MAVLink message reading loop error: {e}")
                with self._lock:
                    if self._conn:
                        try:
                            self._conn.close()
                        except:
                            pass
                        self._conn = None
                    self._connected = False
                reconnect_ticks = self.reconnect_delay_ticks

            time.sleep(0.01)  # Internal loop runs at 100Hz
