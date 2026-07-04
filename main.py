#!/usr/bin/env python3
"""Main entry point for autonomous drone companion computer."""

import sys
import signal
import logging
import time
import yaml
import cv2
import argparse
import threading
from pathlib import Path


def load_config(config_dir: str) -> dict:
    """Load and merge all .yaml files in config_dir into a single config dict.

    Each YAML file contributes its top-level keys to the merged result.
    Files are loaded in alphabetical order; later keys overwrite earlier ones
    in the (unlikely) case of duplicate top-level sections.

    Args:
        config_dir: Path to directory containing domain-specific YAML files.

    Returns:
        Merged configuration dictionary.

    Raises:
        SystemExit: If config_dir does not exist or contains no YAML files.
    """
    config_path = Path(config_dir)
    if not config_path.is_dir():
        logging.error(f"Config directory '{config_dir}' not found!")
        sys.exit(1)

    yaml_files = sorted(config_path.glob("*.yaml"))
    if not yaml_files:
        logging.error(f"No .yaml files found in config directory '{config_dir}'")
        sys.exit(1)

    merged: dict = {}
    for yaml_file in yaml_files:
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f) or {}
        merged.update(data)
        logging.debug(f"Loaded config: {yaml_file.name} ({list(data.keys())})")

    return merged

from core.mavlink_interface import MAVLinkInterface
from core.flight_control import FlightControl
from core.state_machine import StateMachine
from core.payload_control import PayloadControl
from core.fallback_manager import FallbackManager
from core.system_monitor import SystemMonitor
from vision.camera_manager import CameraManager
from vision.qr_detector import QRDetector
from vision.alignment_controller import AlignmentController
from vision.qr_decoder import QRDecoder


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('drone_pi.log')
    ]
)
logger = logging.getLogger(__name__)


class DroneSystem:
    """Main drone system coordinator."""

    def __init__(self, config_dir='config/', sitl_override=None, gui_override=None):
        # Load and merge all domain-specific YAML files from config_dir
        self.config = load_config(config_dir)
        
        # Apply command line overrides
        if sitl_override is not None:
            self.config['system']['use_sitl'] = sitl_override
        if gui_override is not None:
            self.config['system']['show_gui'] = gui_override

        self.tick_hz = self.config['system']['tick_hz']
        self.running = False
        self.tick_count = 0
        self.tick_thread = None
        
        # Initialize subsystems
        logger.info("=" * 60)
        logger.info("Initializing drone companion systems...")
        logger.info(f"SITL simulation mode: {self.config['system']['use_sitl']}")
        logger.info(f"GUI preview display: {self.config['system']['show_gui']}")
        logger.info("=" * 60)
        
        # MAVLink interface setup
        use_sitl = self.config['system']['use_sitl']
        conn_str = self.config['mavlink']['port_sitl'] if use_sitl else self.config['mavlink']['port_real']
        
        self.mav_interface = MAVLinkInterface(
            baud=self.config['mavlink']['baud'],
            heartbeat_timeout_ticks=self.config['mavlink']['heartbeat_timeout_ticks'],
            reconnect_delay_ticks=self.config['mavlink']['reconnect_delay_ticks'],
            use_sitl=use_sitl,
            connection_string=conn_str
        )
        
        self.flight_control = FlightControl(self.mav_interface)

        # Watchdog subsystems — instantiated before StateMachine so they can be passed in
        self.fallback_manager = FallbackManager(self.flight_control)
        self.system_monitor = SystemMonitor(self.config)

        self.payload_control = PayloadControl(self.config, self.flight_control)
        
        self.camera = CameraManager(
            width=self.config['camera']['width'],
            height=self.config['camera']['height'],
            fps=self.config['camera']['fps'],
            buffer_size=self.config['camera']['buffer_size'],
            iso=self.config['camera']['iso'],
            shutter_speed_us=self.config['camera']['shutter_speed_us'],
            auto_exposure_adapt=self.config['camera']['auto_exposure_adapt']
        )
        
        self.qr_detector = QRDetector(
            min_area=self.config['vision']['min_qr_area'],
            min_width_px=self.config['vision']['min_qr_pixel_width'],
            qr_size_cm=self.config['vision']['qr_size_cm'],
            fov_horizontal_deg=self.config['camera'].get('fov_horizontal_deg', 66.0)
        )
        
        self.alignment_controller = AlignmentController(self.config)
        
        self.qr_decoder = QRDecoder(
            max_attempts=self.config['qr_decode']['max_attempts']
        )
        
        self.state_machine = StateMachine(
            self.config,
            self.flight_control,
            self.camera,
            self.qr_detector,
            self.alignment_controller,
            self.qr_decoder,
            self.payload_control,
            self.fallback_manager
        )
        
        logger.info("Subsystems initialized successfully.")

    def start(self):
        """Start all companion subsystems."""
        logger.info("Starting drone companion system threads...")
        self.mav_interface.start()
        self.camera.start()
        self.running = True

    def stop(self):
        """Stop all companion subsystems."""
        if not self.running:
            return
        logger.info("Stopping drone companion systems...")
        self.running = False
        
        self.camera.stop()
        self.mav_interface.stop()
        logger.info("Drone companion systems shut down cleanly.")

    def run(self):
        """Main execution entry, spawns background ticks and main-thread display loops."""
        # 1. Start FSM tick loop in background thread
        self.tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self.tick_thread.start()
        
        # 2. Run display loop on the main thread if GUI is enabled
        if self.config['system']['show_gui']:
            self._display_loop()
        else:
            # Headless blocking wait
            try:
                while self.running:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
            self.stop()

    def _tick_loop(self):
        """FSM tick loop running in background thread."""
        logger.info(f"Starting FSM tick loop at {self.tick_hz} Hz")
        tick_period = 1.0 / self.tick_hz
        
        while self.running:
            loop_start = time.time()

            # System health watchdog — check CPU temperature before any FSM work
            if not self.system_monitor.check_health():
                logger.critical("System health check FAILED. Triggering failsafe and stopping.")
                self.fallback_manager.handle_fail("SystemMonitor: health check failed")
                self.running = False
                break

            # Increment MAVLink internal heartbeat ticks
            self.mav_interface.tick()
            
            # Tick state machine
            try:
                self.state_machine.tick(self.tick_count)
            except Exception as e:
                logger.error(f"FSM Tick Exception: {e}", exc_info=True)
                
            self.tick_count += 1
            
            # Maintain tick frequency
            elapsed = time.time() - loop_start
            sleep_time = tick_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif elapsed > tick_period * 1.5:
                logger.warning(f"FSM loop overrun: {elapsed:.3f}s (target: {tick_period:.3f}s)")

    def _display_loop(self):
        """GUI preview loop running on the main thread."""
        logger.info("Starting OpenCV display loop...")
        win_name = "Drone Companion Computer Preview"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 1280, 720)
        
        # Printable controls info
        print("\n" + "=" * 60)
        print("  GUI Preview Controls:")
        print("    'q' : Quit companion computer execution")
        print("=" * 60 + "\n")

        while self.running:
            frame = self.camera.get_latest_frame()
            if frame is not None:
                h, w = frame.shape[:2]
                
                # Draw targeting crosshair
                cv2.line(frame, (w // 2 - 30, h // 2), (w // 2 + 30, h // 2), (0, 0, 255), 2)
                cv2.line(frame, (w // 2, h // 2 - 30), (w // 2, h // 2 + 30), (0, 0, 255), 2)
                cv2.circle(frame, (w // 2, h // 2), 60, (0, 0, 255), 2)
                
                # Overlay telemetry status
                status_text = f"State: {self.state_machine.state.name}"
                cv2.putText(frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                
                alt_text = f"Alt: {self.mav_interface.get_altitude():.2f}m"
                cv2.putText(frame, alt_text, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
                mode_text = "SITL Simulator" if self.config['system']['use_sitl'] else "Pi 5 Hardware"
                cv2.putText(frame, f"Mode: {mode_text}", (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

                # Show frame
                cv2.imshow(win_name, frame)
                
            # waitKey runs events handling on main thread
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                logger.info("User requested exit from display loop.")
                self.stop()
                break

        cv2.destroyAllWindows()


def signal_handler(signum, frame):
    """Handle termination signals."""
    logger.info(f"Signal {signum} caught. Shutting down companion computer...")
    if hasattr(signal_handler, 'system'):
        signal_handler.system.stop()
    sys.exit(0)


def main():
    """Main companion script parser."""
    parser = argparse.ArgumentParser(description="Autonomous Companion Computer.")
    parser.add_argument(
        '--config_dir', type=str, default='config/',
        help='Path to directory containing domain-specific YAML config files (default: config/).'
    )

    # Mode overrides
    group_sitl = parser.add_mutually_exclusive_group()
    group_sitl.add_argument('--sitl', action='store_true', dest='sitl', default=None, help='Force SITL connection.')
    group_sitl.add_argument('--real', action='store_false', dest='sitl', default=None, help='Force Serial Port Pixhawk connection.')

    group_gui = parser.add_mutually_exclusive_group()
    group_gui.add_argument('--gui', action='store_true', dest='gui', default=None, help='Force enable preview GUI.')
    group_gui.add_argument('--headless', action='store_false', dest='gui', default=None, help='Force disable preview GUI.')

    args = parser.parse_args()

    # Instantiate companion coordinator
    system = DroneSystem(
        config_dir=args.config_dir,
        sitl_override=args.sitl,
        gui_override=args.gui
    )
    
    signal_handler.system = system
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        system.start()
        system.run()
    except Exception as e:
        logger.fatal(f"Unhandled fatal exception: {e}", exc_info=True)
        system.stop()
        sys.exit(1)


if __name__ == '__main__':
    main()
