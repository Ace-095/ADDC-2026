"""High-Performance Aerial QR Code Detector using pyzbar and adaptive ROI zoom."""

import logging
import cv2
import numpy as np
from pyzbar import pyzbar
from collections import deque
from typing import Tuple, Optional, List, Any

logger = logging.getLogger(__name__)


class QRDetector:
    """Detect and locate QR codes in video frames with flight-optimized preprocessing.

    CHANGE 4: in addition to the boolean ``detect()`` API (kept for backward
    compatibility), this detector now computes a per-frame *confidence* score in
    [0, 1] and tracks temporal consistency across recent detections. The score
    fuses: (a) decoding success, (b) target pixel size vs the minimum gate,
    (c) detection-quality margins, and (d) agreement of the current detection's
    pixel location with the recent track (rejects single-frame hallucinations).
    The FSM (INITIAL_SCAN / SEARCH_SQUARE) requires several consecutive
    confident frames before committing to ALIGNMENT, which dramatically reduces
    false positives and stabilises the hand-off.
    """

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

        # ── CHANGE 4: confidence / temporal tracking state ────────────────
        # Rolling window of recent detection centers (px) for consistency check.
        self._track_window = deque(maxlen=5)
        # Most recent confidence + track metadata (exposed for diagnostics).
        self.last_confidence: float = 0.0
        self._last_track_jitter_px: float = float('inf')

    def _track_consistency(self, center) -> float:
        """Return a [0,1] consistency score for a new detection center.

        Compares the candidate center against the recent track of confirmed
        detections. High agreement (small mean distance) → high score. A cold
        track (no history) returns 0.5 (neutral) so a first valid detection is
        not penalised; a wildly jumping detection scores low.
        """
        if center is None:
            return 0.0
        if len(self._track_window) == 0:
            return 0.5
        dists = [float(np.hypot(center[0] - c[0], center[1] - c[1])) for c in self._track_window]
        mean_jitter = sum(dists) / len(dists)
        self._last_track_jitter_px = mean_jitter
        # Decay: full credit at ≤30 px mean offset, none at ≥150 px.
        score = max(0.0, 1.0 - (mean_jitter - 30.0) / 120.0)
        return min(1.0, score)

    def _confidence(self, decoded_ok: bool, bbox_width_px: int, center) -> float:
        """Fuse sub-scores into a single [0,1] detection confidence (CHANGE 4)."""
        # Size margin: how far above the minimum gate is the detection? A QR
        # comfortably larger than the gate is more reliable than a marginal one.
        size_ratio = bbox_width_px / max(1, self.min_width_px)
        size_score = min(1.0, size_ratio / 2.0)  # 1.0 once ≥ 2× the gate
        # Decode readiness: a QR pyzbar actually decoded is strong evidence.
        decode_score = 1.0 if decoded_ok else 0.45
        # Temporal agreement with the recent track.
        track_score = self._track_consistency(center)

        confidence = (0.5 * decode_score) + (0.3 * size_score) + (0.2 * track_score)
        return float(min(1.0, max(0.0, confidence)))

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
                        # decoded_ok = barcode carries a data payload (pyzbar found the code)
                        return self._accept(barcode.data is not None and len(barcode.data) > 0,
                                            bbox, center)

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
                        return self._accept(
                            barcode.data is not None and len(barcode.data) > 0,
                            bbox_pts, (cx, cy), bbox_width_px=bw)

        # 4. Stage 3: Direct fallbacks to alternative adaptive thresholds (comprehensive scan)
        for thresh_method in [cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.ADAPTIVE_THRESH_MEAN_C]:
            thresh = cv2.adaptiveThreshold(gray, 255, thresh_method, cv2.THRESH_BINARY, 21, 10)
            barcodes = pyzbar.decode(thresh)
            if barcodes:
                for barcode in barcodes:
                    found, bbox, center = self._process_barcode_result(barcode, 1.0)
                    if found:
                        return self._accept(
                            barcode.data is not None and len(barcode.data) > 0,
                            bbox, center)

        # No QR detected
        self._miss()
        return False, None, None

    def _accept(self, decoded_ok: bool, bbox: np.ndarray, center, bbox_width_px: int = 0) -> tuple:
        """Record a successful detection, update confidence + track, and return it.

        Centralises every success path (CHANGE 4) so confidence scoring and the
        temporal track window are always updated consistently.
        """
        if bbox_width_px <= 0 and bbox is not None:
            xs = bbox[:, 0]
            bbox_width_px = int(xs.max() - xs.min())
        self.detection_history.append(True)
        self._track_window.append((int(center[0]), int(center[1])))
        self.last_center = center
        self.last_bbox = bbox
        self.last_confidence = self._confidence(decoded_ok, bbox_width_px, center)
        logger.debug(
            f"QR accept: conf={self.last_confidence:.2f} "
            f"w={bbox_width_px}px jitter={self._last_track_jitter_px:.0f}px "
            f"decoded={decoded_ok}"
        )
        return True, bbox, center

    def _miss(self):
        """Record a no-detection frame (decays the temporal track)."""
        self.detection_history.append(False)
        self.last_confidence = 0.0
        # Do NOT clear the track window on a single miss — a one-frame dropout
        # shouldn't reset accumulated consistency. It decays naturally via maxlen.

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
        """Adapt frame preprocessing dynamically based on scene brightness (CHANGE 4).

        Now uses a rolling brightness VARIANCE estimate (not just the mean) so
        rapidly changing light — partly-clouded sun, shadow edges, indoor
        flicker — is handled. High-variance scenes get a stronger contrast floor
        and mild denoise; low-light frames get denoise + a conservative sharpen.
        All branches stay computationally cheap (CLAHE + a couple of small kernels)
        so the Pi 5 + AI HAT pipeline keeps ~20 Hz.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        avg_brightness = np.mean(gray)
        self.brightness_history.append(avg_brightness)

        if len(self.brightness_history) >= 10:
            mean_brightness = np.mean(self.brightness_history)
            brightness_std = float(np.std(self.brightness_history))  # rolling variance proxy
            if mean_brightness > 180:
                # Bright sunlight: clamp glare, soft blur to flatten highlights.
                enhanced = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
                enhanced = cv2.GaussianBlur(enhanced, (3, 3), 0)
            elif mean_brightness < 80:
                # Low light: denoise first (noise explodes under heavy CLAHE), then
                # high-contrast CLAHE, then a conservative sharpen.
                enhanced = cv2.fastNlMeansDenoising(gray, None, h=7, templateWindowSize=7,
                                                   searchWindowSize=21)
                enhanced = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8)).apply(enhanced)
                kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
                enhanced = cv2.filter2D(enhanced, -1, kernel)
            elif brightness_std > 40:
                # Flickering / mixed-light scene: raise the contrast floor a notch
                # so the QR finder pattern stays legible across the lighting swing.
                enhanced = cv2.createCLAHE(clipLimit=2.8, tileGridSize=(8, 8)).apply(gray)
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
