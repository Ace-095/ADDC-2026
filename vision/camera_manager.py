"""Camera interface with Picamera2 native support and OpenCV fallbacks for SITL."""

import logging
import cv2
import numpy as np
import time
import threading
from collections import deque
from typing import Optional, Tuple, Dict, Any

logger = logging.getLogger(__name__)


class CameraManager:
    """Manage native Pi Camera or OpenCV fallbacks for frame capture."""

    def __init__(self, source: str = "hardware", width: int = 1920, height: int = 1080, fps: int = 20, buffer_size: int = 2, iso: int = 100, shutter_speed_us: int = 600, auto_exposure_adapt: bool = True):
        self.source = source
        self.width = width
        self.height = height
        self.fps = fps
        self.buffer_size = buffer_size
        self.iso = iso
        self.shutter_speed_us = shutter_speed_us
        self.auto_exposure_adapt = auto_exposure_adapt

        self.camera = None
        self.cap = None
        self.use_picamera = False
        self.camera_type = "None"
        
        self._thread = None
        self._running = False
        self._frame_buffer = deque(maxlen=buffer_size)
        self._lock = threading.Lock()

    def start(self) -> bool:
        """Initialize camera backend and start the capture thread."""
        logger.info("Initializing camera systems...")
        
        # 1. Attempt Gazebo GStreamer if source is gazebo
        if self.source == "gazebo":
            if self._init_gazebo_gstreamer():
                self._running = True
                self._thread = threading.Thread(target=self._capture_loop, daemon=True)
                self._thread.start()
                logger.info(f"Camera manager capture thread started using backend: {self.camera_type}")
                return True
            else:
                logger.error("Gazebo GStreamer initialization failed!")
                return False

        # 2. Attempt Picamera2 initialization
        if self._init_picamera2():
            self.use_picamera = True
        else:
            logger.warning("Picamera2 not available or failed. Falling back to OpenCV capture...")
            # 3. Attempt OpenCV fallbacks
            if not self._init_opencv():
                logger.error("All camera initialization attempts failed!")
                return False

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        logger.info(f"Camera manager capture thread started using backend: {self.camera_type}")
        return True

    def stop(self):
        """Stop camera capture thread and release hardware resources."""
        logger.info("Stopping camera manager...")
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        
        with self._lock:
            if self.use_picamera and self.camera:
                try:
                    self.camera.stop()
                    self.camera.close()
                except Exception as e:
                    logger.warning(f"Error closing Picamera2: {e}")
                self.camera = None
            elif self.cap:
                try:
                    self.cap.release()
                except Exception as e:
                    logger.warning(f"Error releasing OpenCV VideoCapture: {e}")
                self.cap = None
        logger.info("Camera manager stopped")

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Get the most recent frame (non-blocking, thread-safe)."""
        with self._lock:
            if len(self._frame_buffer) > 0:
                return self._frame_buffer[-1].copy()
            return None

    def update_settings(self, iso: Optional[int] = None, exposure_us: Optional[int] = None):
        """
        Dynamically update exposure and ISO controls on active camera hardware.

        Args:
            iso: Analog gain control (100 - 3200)
            exposure_us: Exposure time in microseconds
        """
        with self._lock:
            if self.use_picamera and self.camera:
                controls = {}
                if iso is not None:
                    self.iso = iso
                    controls["AnalogueGain"] = float(iso) / 100.0
                if exposure_us is not None:
                    self.shutter_speed_us = exposure_us
                    controls["ExposureTime"] = int(exposure_us)
                
                if controls:
                    try:
                        self.camera.set_controls(controls)
                        logger.debug(f"Picamera2 controls updated: {controls}")
                    except Exception as e:
                        logger.warning(f"Failed to update Picamera2 controls: {e}")
            elif self.cap:
                # OpenCV hardware controls vary widely by driver/OS
                try:
                    if iso is not None:
                        self.iso = iso
                        self.cap.set(cv2.CAP_PROP_ISO_SPEED, iso)
                    if exposure_us is not None:
                        self.shutter_speed_us = exposure_us
                        self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure_us)
                except Exception as e:
                    logger.warning(f"Failed to update OpenCV camera controls: {e}")

    def _init_picamera2(self) -> bool:
        """Try to import and configure Picamera2."""
        try:
            from picamera2 import Picamera2
            self.camera = Picamera2()
            
            # Setup preview configuration
            config = self.camera.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"},
                buffer_count=2
            )
            self.camera.configure(config)
            self.camera.start()
            
            # Allow camera sensor to warm up/auto-expose initially
            time.sleep(1.5)
            
            # Apply initial manual settings optimized to suppress motion blur
            controls = {
                "AeEnable": False,        # Disable automatic exposure
                "AwbEnable": True,         # Retain auto white balance for color contrast
                "AnalogueGain": float(self.iso) / 100.0,
                "ExposureTime": int(self.shutter_speed_us)
            }
            
            # Adaptive initial exposure
            if self.auto_exposure_adapt:
                test_array = self.camera.capture_array("main")
                avg_brightness = np.mean(test_array)
                if avg_brightness > 180:
                    controls["ExposureTime"] = int(self.shutter_speed_us * 0.5)
                elif avg_brightness < 80:
                    controls["ExposureTime"] = int(self.shutter_speed_us * 1.5)
            
            self.camera.set_controls(controls)
            time.sleep(0.5)
            
            # Test frame fetch
            test_frame = self.camera.capture_array("main")
            if test_frame is not None and test_frame.size > 0:
                self.camera_type = "Picamera2 (Native)"
                logger.info(f"Picamera2 successfully initialized at {self.width}x{self.height}")
                return True
            
            self.camera.stop()
            self.camera.close()
            self.camera = None
            return False
            
        except ImportError:
            logger.debug("picamera2 python package not installed.")
            return False
        except Exception as e:
            logger.debug(f"Failed to initialize Picamera2: {e}")
            if self.camera:
                try:
                    self.camera.close()
                except:
                    pass
                self.camera = None
            return False

    def _init_gazebo_gstreamer(self) -> bool:
        """Initialize GStreamer pipeline to receive Gazebo video stream."""
        try:
            # Standard Gazebo SITL gstreamer UDP pipeline
            pipeline = (
                "udpsrc port=5600 timeout=3000000000 ! application/x-rtp, payload=96 ! "
                "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw, format=BGR ! appsink sync=false async=false drop=true"
            )
            logger.debug(f"Attempting Gazebo GStreamer capture with pipeline: {pipeline}")
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if cap.isOpened():
                # Avoid reading a test frame here as it might block the main thread 
                # indefinitely if Gazebo is not streaming video yet.
                self.cap = cap
                self.camera_type = "Gazebo GStreamer"
                logger.info("Camera initialized: Gazebo GStreamer")
                return True
            return False
        except Exception as e:
            logger.error(f"Gazebo GStreamer init failed: {e}")
            return False

    def _init_opencv(self) -> bool:
        """Try various OpenCV camera captures."""
        # Check standard video index candidates
        for idx in [0, 1, 2]:
            for backend in [cv2.CAP_V4L2, cv2.CAP_ANY]:
                try:
                    logger.debug(f"Trying OpenCV capture device {idx} with backend {backend}")
                    cap = cv2.VideoCapture(idx, backend)
                    if cap.isOpened():
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                        cap.set(cv2.CAP_PROP_FPS, self.fps)
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        
                        # Set MJPEG if possible to maximize resolution throughput
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                        
                        time.sleep(1.0)
                        ret, test_frame = cap.read()
                        if ret and test_frame is not None and test_frame.size > 0:
                            self.cap = cap
                            self.camera_type = f"OpenCV index {idx} ({backend})"
                            logger.info(f"OpenCV camera successfully initialized at {test_frame.shape[1]}x{test_frame.shape[0]}")
                            return True
                        cap.release()
                except Exception as e:
                    logger.debug(f"OpenCV init failed on device {idx} backend {backend}: {e}")
        return False

    def _capture_loop(self):
        """Continuously extract frames from camera and update the thread-safe buffer."""
        delay = 1.0 / self.fps
        
        while self._running:
            loop_start = time.time()
            frame = None
            try:
                if self.use_picamera and self.camera:
                    array = self.camera.capture_array("main")
                    # Picamera2 returns RGB888 arrays; convert to OpenCV BGR color format
                    frame = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
                elif self.cap and self.cap.isOpened():
                    ret, raw_frame = self.cap.read()
                    if ret:
                        frame = raw_frame
                    
                if frame is not None:
                    with self._lock:
                        self._frame_buffer.append(frame)
                else:
                    time.sleep(0.01)
                    
            except Exception as e:
                logger.error(f"Error in Camera Capture loop: {e}")
                time.sleep(0.5)

            elapsed = time.time() - loop_start
            time.sleep(max(0, delay - elapsed))
