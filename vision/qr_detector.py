"""High-Performance Aerial QR Code Detector using pyzbar and adaptive ROI zoom."""

import logging
import cv2
import numpy as np
from pyzbar import pyzbar
from collections import deque
from typing import Tuple, Optional, List, Any

logger = logging.getLogger(__name__)


class QRDetector:
    """Detect and locate QR codes in video frames with flight-optimized preprocessing."""

    def __init__(self, min_area: int = 1600, min_width_px: int = 80, qr_size_cm: float = 21.0, fov_horizontal_deg: float = 66.0):
        self.min_area = min_area
        self.min_width_px = min_width_px
        self.qr_size_cm = qr_size_cm
        self.fov_horizontal_deg = fov_horizontal_deg

        # Adaptive lighting history
        self.brightness_history = deque(maxlen=20)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        
        # Detection history for confidence filtering
        self.detection_history = deque(maxlen=5)
        
        # Track last successfully matched coordinates
        self.last_center = None
        self.last_bbox = None

    def detect(self, frame: np.ndarray) -> Tuple[bool, Optional[np.ndarray], Optional[Tuple[int, int]]]:
        """
        Scan frame for QR codes using multi-scale search and candidate ROI zooming.

        Args:
            frame: Input image frame (BGR format)

        Returns:
            found: Boolean indicating successful detection
            bbox: (4, 2) NumPy array of bounding box points or None
            center: (cx, cy) center coordinate tuple in pixels or None
        """
        if frame is None or frame.size == 0:
            return False, None, None

        h, w = frame.shape[:2]
        
        # 1. Apply adaptive lighting pre-processing
        gray = self._adapt_preprocessing(frame)

        # 2. Stage 1: Quick Multi-scale direct scan (full frame & downscaled for performance)
        for scale in [1.0, 0.75, 0.5]:
            scaled_gray = gray
            if scale != 1.0:
                scaled_gray = cv2.resize(gray, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)

            barcodes = pyzbar.decode(scaled_gray)
            if barcodes:
                for barcode in barcodes:
                    found, bbox, center = self._process_barcode_result(barcode, scale)
                    if found:
                        self.detection_history.append(True)
                        return True, bbox, center

        # 3. Stage 2: ROI Candidate Detection (Zoom & Sharpen for distant targets)
        candidate_rois = self._find_candidate_regions(gray)
        for rx, ry, rw, rh in candidate_rois:
            roi = gray[ry:ry+rh, rx:rx+rw]
            if roi.size == 0:
                continue

            # Digital zoom (bicubic resize) and sharpening enhancement
            zoom_factor = 800.0 / max(rw, rh)
            if zoom_factor > 1.0:
                zoomed_roi = cv2.resize(roi, (int(rw * zoom_factor), int(rh * zoom_factor)), interpolation=cv2.INTER_CUBIC)
            else:
                zoomed_roi = roi
                zoom_factor = 1.0

            # Apply unsharp mask sharpening filter
            blur = cv2.GaussianBlur(zoomed_roi, (0, 0), 1.0)
            sharpened_roi = cv2.addWeighted(zoomed_roi, 2.0, blur, -1.0, 0)
            
            # Run scan on enhanced ROI
            barcodes = pyzbar.decode(sharpened_roi)
            if barcodes:
                for barcode in barcodes:
                    # Convert barcode box coordinates back to full frame
                    bx = rx + int(barcode.rect.left / zoom_factor)
                    by = ry + int(barcode.rect.top / zoom_factor)
                    bw = int(barcode.rect.width / zoom_factor)
                    bh = int(barcode.rect.height / zoom_factor)
                    
                    if bw >= self.min_width_px:
                        cx = bx + bw // 2
                        cy = by + bh // 2
                        bbox_pts = np.array([
                            [bx, by],
                            [bx + bw, by],
                            [bx + bw, by + bh],
                            [bx, by + bh]
                        ], dtype=np.int32)
                        
                        logger.debug(f"Target found in ROI: {bw}x{bh}px, center: {cx}, {cy}")
                        self.detection_history.append(True)
                        return True, bbox_pts, (cx, cy)

        # 4. Stage 3: Direct fallbacks to alternative adaptive thresholds (comprehensive scan)
        for thresh_method in [cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.ADAPTIVE_THRESH_MEAN_C]:
            thresh = cv2.adaptiveThreshold(gray, 255, thresh_method, cv2.THRESH_BINARY, 21, 10)
            barcodes = pyzbar.decode(thresh)
            if barcodes:
                for barcode in barcodes:
                    found, bbox, center = self._process_barcode_result(barcode, 1.0)
                    if found:
                        self.detection_history.append(True)
                        return True, bbox, center

        # No QR detected
        self.detection_history.append(False)
        return False, None, None

    def estimate_distance(self, pixel_width: int, total_width_px: int) -> float:
        """
        Calculate distance from camera to QR code based on focal lengths.
        
        Args:
            pixel_width: Bounding box width of QR code in pixels
            total_width_px: Full frame resolution width in pixels
            
        Returns:
            Estimated distance in meters
        """
        # Calculate focal length dynamically from FOV
        fov_horizontal_rad = np.radians(self.fov_horizontal_deg)
        focal_length_px = (total_width_px / 2.0) / np.tan(fov_horizontal_rad / 2.0)
        
        real_width_m = self.qr_size_cm / 100.0
        distance = (real_width_m * focal_length_px) / pixel_width
        return float(distance)

    def _adapt_preprocessing(self, frame: np.ndarray) -> np.ndarray:
        """Adapt frame preprocessing dynamically based on average scene brightness."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        avg_brightness = np.mean(gray)
        self.brightness_history.append(avg_brightness)

        if len(self.brightness_history) >= 10:
            mean_brightness = np.mean(self.brightness_history)
            if mean_brightness > 180:
                # Bright sunlight: apply soft blur to eliminate glares after CLAHE
                enhanced = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
                enhanced = cv2.GaussianBlur(enhanced, (3, 3), 0)
            elif mean_brightness < 80:
                # Low light: apply high contrast CLAHE and sharpening kernel
                enhanced = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8)).apply(gray)
                kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
                enhanced = cv2.filter2D(enhanced, -1, kernel)
            else:
                enhanced = self.clahe.apply(gray)
        else:
            enhanced = self.clahe.apply(gray)
            
        return enhanced

    def _find_candidate_regions(self, gray: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """Identify candidate rectangular high-contrast contours (potential QR blocks)."""
        h, w = gray.shape
        # Edge combine
        edges1 = cv2.Canny(gray, 50, 150)
        edges2 = cv2.Canny(gray, 30, 100)
        edges = cv2.bitwise_or(edges1, edges2)

        # Close and dilate gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        dilated = cv2.dilate(closed, kernel, iterations=1)

        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1000 or area > (w * h * 0.7):
                continue
            
            rx, ry, rw, rh = cv2.boundingRect(cnt)
            aspect_ratio = float(rw) / rh if rh > 0 else 0
            
            # QR targets are square-ish
            if 0.5 < aspect_ratio < 2.0:
                # Calculate standard deviation inside contour bounds for high contrast confirmation
                roi = gray[ry:ry+rh, rx:rx+rw]
                if roi.size > 0 and np.std(roi) > 40:
                    # Pad region for extraction
                    pad = 20
                    bx = max(0, rx - pad)
                    by = max(0, ry - pad)
                    bw = min(w - bx, rw + 2*pad)
                    bh = min(h - by, rh + 2*pad)
                    candidates.append((bx, by, bw, bh))
                    
        return candidates[:5]

    def _process_barcode_result(self, barcode: Any, scale: float) -> Tuple[bool, Optional[np.ndarray], Optional[Tuple[int, int]]]:
        """Convert a pyzbar barcode result to standardized bounding box and center offsets."""
        rx = int(barcode.rect.left / scale)
        ry = int(barcode.rect.top / scale)
        rw = int(barcode.rect.width / scale)
        rh = int(barcode.rect.height / scale)

        # Enforce minimum size rule to avoid triggering on glitched signals
        if rw < self.min_width_px:
            logger.warning(f"Detected QR width {rw}px is below minimum width safety gate ({self.min_width_px}px). Drone too high.")
            return False, None, None

        if rw * rh < self.min_area:
            return False, None, None

        cx = rx + rw // 2
        cy = ry + rh // 2

        # Format points as a polygon box
        bbox = np.array([
            [rx, ry],
            [rx + rw, ry],
            [rx + rw, ry + rh],
            [rx, ry + rh]
        ], dtype=np.int32)

        return True, bbox, (cx, cy)
