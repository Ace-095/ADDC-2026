import sys
import time
import logging
from pathlib import Path
import threading
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("sitl_verify")

from core.mavlink_interface import MAVLinkInterface
from core.flight_control import FlightControl
from core.payload_control import PayloadControl
from core.fallback_manager import FallbackManager
from core.state_machine import StateMachine, State

class MockVisionPipeline:
    def __init__(self):
        self.detection_mode = 'qr'
        self.found = False
        self.center = (960, 540)
        self.aligned = False
        self.decode_success = False
        self.decode_text = None
        self.vx = 0.0
        self.vy = 0.0

    def get_latest_result(self):
        return {
            'found': self.found,
            'center': self.center,
            'aligned': self.aligned,
            'decode_success': self.decode_success,
            'decode_text': self.decode_text,
            'vx': self.vx,
            'vy': self.vy
        }
        
    def set_request_decode(self, enable):
        pass
        
    def set_detection_mode(self, mode):
        self.detection_mode = mode

class MockCamera:
    def start(self): pass
    def stop(self): pass

def load_config(config_dir: str) -> dict:
    import yaml
    config_path = Path(config_dir)
    yaml_files = sorted(config_path.glob("*.yaml"))
    merged = {}
    for yaml_file in yaml_files:
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f) or {}
        merged.update(data)
    return merged

def run_test():
    config = load_config('config/')

    # Initialize subsystems
    mav = MAVLinkInterface(
        baud=config['mavlink']['baud'],
        heartbeat_timeout_ticks=config['mavlink']['heartbeat_timeout_ticks'],
        use_sitl=True,
        connection_string=config['mavlink']['port_sitl']
    )
    fc = FlightControl(mav)
    payload = PayloadControl(config, fc)
    fallback = FallbackManager(fc)
    vision = MockVisionPipeline()
    
    sm = StateMachine(config, fc, vision, payload, fallback)
    
    mav.start()
    
    logger.info("Waiting for SITL connection...")
    while not mav.is_connected():
        time.sleep(0.5)
        
    logger.info("Setting parameters...")
    mav.connection.mav.param_set_send(
        mav.connection.target_system, mav.connection.target_component,
        b'PLND_ENABLED', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    mav.connection.mav.param_set_send(
        mav.connection.target_system, mav.connection.target_component,
        b'PLND_TYPE', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    
    # Wait for GPS
    logger.info("Waiting for GPS fix >= 3...")
    while mav.get_gps_fix_type() < 3:
        mav.tick()
        time.sleep(0.1)

    logger.info("Arming and taking off...")
    fc.set_mode("GUIDED")
    time.sleep(1)
    mav.connection.arducopter_arm()
    mav.connection.motors_armed_wait()
    mav.connection.mav.command_long_send(
        mav.connection.target_system, mav.connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10.0
    )
    
    while mav.get_altitude() < 9.5:
        mav.tick()
        sm.tick(1)
        time.sleep(0.1)
        
    logger.info("Takeoff complete. Simulating Sprayer Command (Phase A).")
    # Simulate DO_SPRAYER
    mav.sprayer_detected = True
    
    # Run FSM
    ticks = 0
    phase_success = {'true_home': False, 'set_home': False, 'platform_landing': False}
    
    while ticks < 300: # 30 seconds max
        mav.tick()
        sm.tick(ticks)
        
        # Inject CV state based on FSM state
        if sm.current_state == State.INITIAL_SCAN:
            vision.found = True
        elif sm.current_state == State.ALIGNMENT:
            vision.aligned = True
            
        # Check assertions
        if sm.true_home is not None and not phase_success['true_home']:
            logger.info("✅ True Home captured!")
            phase_success['true_home'] = True
            
        if sm.home_restored and not phase_success['set_home']:
            logger.info("✅ DO_SET_HOME ACK received and home restored!")
            phase_success['set_home'] = True
            
        if sm.current_state == State.RTL and sm.landing_context == 'qr':
            # Force close to home to trigger platform landing
            if sm.true_home:
                # Mock location to be near true home
                pass
                
        if sm.current_state == State.LANDING and sm.landing_context == 'platform':
            logger.info("✅ Transitioned to platform landing!")
            phase_success['platform_landing'] = True
            break
            
        time.sleep(0.1)
        ticks += 1
        
    mav.stop()
    print("TEST RESULTS:", phase_success)

if __name__ == '__main__':
    run_test()
