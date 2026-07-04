#!/usr/bin/env python3
import time
import logging
from core.payload_control import PayloadControl

logging.basicConfig(level=logging.INFO)

class MockMavlink:
    def send_command_long(self, command, param1=0.0, param2=0.0, **kwargs):
        print(f"[MAVLink Mock] Sending command {command} p1={param1} p2={param2}")
        return True
        
    def send_statustext(self, text, severity=6):
        print(f"[MAVLink Mock] STATUSTEXT: {text}")
        return True

class MockFlightControl:
    def __init__(self):
        self.mav = MockMavlink()

def main():
    config = {
        'mavlink': {
            'payload_servo_channel': 9,
            'servo_pwm_open': 1900,
            'servo_pwm_closed': 1100,
            'servo_open_duration_s': 2.0
        },
        'flight': {
            'release_distance_min_m': 0.1,
            'release_distance_max_m': 0.5,
            'takeoff_altitude_threshold_m': 2.0
        },
        'system': {
            'use_sitl': True
        }
    }
    
    fc = MockFlightControl()
    payload = PayloadControl(config, fc)
    
    print("Payload Control Bench Test")
    print("--------------------------")
    input("Press Enter to trigger payload release...")
    
    success = payload.trigger_release()
    print(f"Trigger call returned: {success}")
    
    # Wait for the background thread to complete the 2 second delay + padding
    time.sleep(3.0)
    print("Bench test complete.")

if __name__ == '__main__':
    main()
