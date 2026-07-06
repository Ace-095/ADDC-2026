import logging
import time
import threading
from typing import Optional, Tuple, Any
import numpy as np

logger = logging.getLogger(__name__)


class VisionPipeline:
    """Background thread that continuously processes camera frames for CV tasks.
    
    Offloads heavy synchronous work (detection, alignment math, decoding) from
    the high-frequency FSM loop.
    """

    def __init__(self, camera, qr_detector, qr_decoder, alignment_controller, flight_control):
        self.cam = camera
        self.qr_det = qr_detector
        self.qr_dec = qr_decoder
        self.align = alignment_controller
        self.fc = flight_control

        self._running = False
        self._thread = None
        self._lock = threading.Lock()

        # Thread-safe cache of the latest CV results
        self._latest_result = {
            'found': False,
            'bbox': None,
            'center': None,
            'frame': None,
            'timestamp': 0.0,
            'aligned': False,
            'vx': 0.0,
            'vy': 0.0,
            'decode_success': False,
            'decode_text': None,
            'decode_final': False
        }

        self.request_decode = False

    def start(self):
        """Start the background vision processing loop."""
        self._running = True
        self._thread = threading.Thread(target=self._vision_loop, daemon=True)
        self._thread.start()
        logger.info("VisionPipeline background thread started.")

    def stop(self):
        """Stop the vision processing loop."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        logger.info("VisionPipeline stopped.")

    def get_latest_result(self) -> dict:
        """Fetch the most recent detection/alignment/decode result (non-blocking)."""
        with self._lock:
            return self._latest_result.copy()

    def set_request_decode(self, enable: bool):
        """Enable or disable active QR decoding (computationally expensive)."""
        with self._lock:
            self.request_decode = enable
            if not enable:
                # Clear stale decode results when turning it off
                self._latest_result['decode_success'] = False
                self._latest_result['decode_text'] = None
                self._latest_result['decode_final'] = False

    def _vision_loop(self):
        """Continuously process the latest camera frame."""
        while self._running:
            loop_start = time.time()
            
            frame = self.cam.get_latest_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            
            # 1. Detection
            found, bbox, center = self.qr_det.detect(frame)
            
            aligned = False
            vx, vy = 0.0, 0.0
            decode_success = False
            decode_text = None
            decode_final = False

            # 2. Alignment & Decode (only if found)
            if found:
                # Alignment
                h, w = frame.shape[:2]
                x_coords = bbox[:, 0]
                pixel_width = int(x_coords.max() - x_coords.min())
                altitude_m = self.fc.mav.get_altitude()
                vx, vy, aligned = self.align.compute(center, pixel_width=pixel_width, altitude_m=altitude_m, frame_size=(w, h))
                
                # Decoding (only if requested)
                with self._lock:
                    should_decode = self.request_decode
                
                if should_decode:
                    success, text, final = self.qr_dec.decode(frame, last_bbox=bbox)
                    if success:
                        decode_success = True
                        decode_text = text
                    decode_final = final

            # 3. Update Cache
            with self._lock:
                # Preserve existing decode text if we found it previously but missed a frame
                if not decode_success and self.request_decode:
                    decode_success = self._latest_result['decode_success']
                    decode_text = self._latest_result['decode_text']
                    decode_final = self._latest_result['decode_final']

                self._latest_result.update({
                    'found': found,
                    'bbox': bbox,
                    'center': center,
                    'frame': frame,
                    'timestamp': time.time(),
                    'aligned': aligned,
                    'vx': vx,
                    'vy': vy,
                    'decode_success': decode_success,
                    'decode_text': decode_text,
                    'decode_final': decode_final
                })
            
            # Yield CPU to ensure other threads run (max ~50Hz processing)
            elapsed = time.time() - loop_start
            sleep_time = max(0.01, 0.02 - elapsed)
            time.sleep(sleep_time)
