#!/usr/bin/env python3
"""Bench test script for QR detector and alignment controller."""

import sys
import os
import time
import logging
import yaml
import cv2
import numpy as np
from pathlib import Path

# Add current directory to path
sys.path.insert(0, str(Path(__file__).parent.resolve()))

from vision.camera_manager import CameraManager
from vision.qr_detector import QRDetector
from vision.alignment_controller import AlignmentController

def load_config(config_dir: str) -> dict:
    config_path = Path(config_dir)
    if not config_path.is_dir():
        print(f"Error: Config directory '{config_dir}' not found!")
        sys.exit(1)
    yaml_files = sorted(config_path.glob("*.yaml"))
    merged = {}
    for yaml_file in yaml_files:
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f) or {}
        merged.update(data)
    return merged

def main():
    # Setup logging to console with debug level so we can see Alignment logs
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    
    logger = logging.getLogger("bench_test_alignment")
    logger.info("Starting Alignment Bench Test...")
    
    config = load_config('config/')
    # Force GUI on
    config['system']['show_gui'] = True
    
    # Initialize components
    camera = CameraManager(
        width=config['camera']['width'],
        height=config['camera']['height'],
        fps=config['camera']['fps'],
        buffer_size=config['camera']['buffer_size'],
        iso=config['camera']['iso'],
        shutter_speed_us=config['camera']['shutter_speed_us'],
        auto_exposure_adapt=config['camera']['auto_exposure_adapt']
    )
    
    qr_detector = QRDetector(
        min_area=config['vision']['min_qr_area'],
        min_width_px=config['vision']['min_qr_pixel_width'],
        qr_size_cm=config['vision']['qr_size_cm'],
        fov_horizontal_deg=config['camera'].get('fov_horizontal_deg', 66.0)
    )
    
    alignment_controller = AlignmentController(config)
    
    # Start camera
    if not camera.start():
        logger.error("Failed to start camera manager!")
        sys.exit(1)
        
    win_name = "Alignment Bench Test Preview"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)
    
    print("\n" + "=" * 60)
    print("  Alignment Bench Test Controls:")
    print("    'q' : Quit bench test")
    print("    'r' : Reset PID controller state")
    print("=" * 60 + "\n")
    
    try:
        last_tick_time = time.time()
        tick_period = 1.0 / config['system']['tick_hz']
        
        while True:
            current_time = time.time()
            # Loop at desired frequency
            if current_time - last_tick_time < tick_period:
                time.sleep(0.005)
                # Still handle window events to keep GUI responsive
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    alignment_controller.reset()
                    print("PID controller state reset by user.")
                continue
                
            last_tick_time = current_time
            
            frame = camera.get_latest_frame()
            if frame is not None:
                h, w = frame.shape[:2]
                
                # Dynamically align controller's camera model with actual frame size (e.g. webcam fallbacks)
                alignment_controller.image_width = w
                alignment_controller.image_height = h
                
                # Detect QR
                found, bbox, center = qr_detector.detect(frame)
                
                # Default values for overlay
                vx, vy, aligned = 0.0, 0.0, False
                err_x_px, err_y_px = 0.0, 0.0
                error_x_m, error_y_m = 0.0, 0.0
                
                if found and center is not None:
                    # Calculate bbox width
                    if bbox is not None:
                        bbox_width = int(np.max(bbox[:, 0]) - np.min(bbox[:, 0]))
                    else:
                        bbox_width = None
                        
                    # Compute PID alignment outputs
                    vx, vy, aligned = alignment_controller.compute(
                        center, 
                        pixel_width=bbox_width, 
                        altitude_m=1.0  # Simulated low altitude for bench test
                    )
                    
                    # Compute pixel error relative to center
                    img_cx = w / 2.0
                    img_cy = h / 2.0
                    err_x_px = center[0] - img_cx
                    err_y_px = center[1] - img_cy
                    
                    # Convert to metric for visualization
                    fov_rad = np.radians(config['camera'].get('fov_horizontal_deg', 66.0))
                    px_to_m_ratio = (2.0 * 1.0 * np.tan(fov_rad / 2.0)) / w
                    error_x_m = err_x_px * px_to_m_ratio
                    error_y_m = err_y_px * px_to_m_ratio
                    
                    # Draw QR bounding box and center
                    if bbox is not None:
                        cv2.polylines(frame, [bbox], True, (0, 255, 0), 3)
                    cv2.circle(frame, center, 8, (255, 0, 0), -1)
                    
                    # Draw a line from center of image to QR center
                    cv2.line(frame, (w // 2, h // 2), center, (255, 255, 0), 2)
                    
                    # Draw velocity vector arrow (scaled for visualization)
                    # Note: vx is forward (maps to vertical screen axis -Y), vy is right (maps to horizontal screen axis +X)
                    # We invert vx on screen because image-Y points down, drone-forward (+X) is up in the image.
                    arrow_scale = 300.0  # multiplier to make velocity visible
                    target_x = int(w // 2 + vy * arrow_scale)
                    target_y = int(h // 2 - vx * arrow_scale)
                    cv2.arrowedLine(frame, (w // 2, h // 2), (target_x, target_y), (0, 255, 255), 3, tipLength=0.2)
                    
                    # Print the required log line format directly to console on every successful detection
                    print(
                        f"Aligning: px_err=({err_x_px:.0f},{err_y_px:.0f}) "
                        f"m_err=({error_x_m:.2f},{error_y_m:.2f}) "
                        f"cmd_vel=({vx:.2f},{vy:.2f}) "
                        f"invert=({alignment_controller.invert_x},{alignment_controller.invert_y}) "
                        f"stable={alignment_controller.stable_counter}/{alignment_controller.stable_required}"
                    )
                
                # Draw targeting crosshair at image center
                cv2.line(frame, (w // 2 - 30, h // 2), (w // 2 + 30, h // 2), (0, 0, 255), 2)
                cv2.line(frame, (w // 2, h // 2 - 30), (w // 2, h // 2 + 30), (0, 0, 255), 2)
                cv2.circle(frame, (w // 2, h // 2), 60, (0, 0, 255), 2)
                
                # Overlay status text
                cv2.putText(frame, f"QR Found: {found}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if found else (0, 0, 255), 2)
                cv2.putText(frame, f"vx (Fwd): {vx:.3f} m/s", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(frame, f"vy (Rgt): {vy:.3f} m/s", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(frame, f"Errors (px): X={err_x_px:.1f}, Y={err_y_px:.1f}", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(frame, f"Invert X/Y: {alignment_controller.invert_x}/{alignment_controller.invert_y}", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 255, 128), 2)
                cv2.putText(frame, f"Stable Frames: {alignment_controller.stable_counter}/{alignment_controller.stable_required}", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255) if aligned else (255, 255, 255), 2)
                if aligned:
                    cv2.putText(frame, "ALIGNED!", (w // 2 - 100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 4)
                
                cv2.imshow(win_name, frame)
                
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                alignment_controller.reset()
                print("PID controller state reset by user.")
                
    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        logger.info("Bench test stopped cleanly.")

if __name__ == '__main__':
    main()
