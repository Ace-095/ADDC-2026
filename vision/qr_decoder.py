"""QR code decoding engine utilizing pyzbar and advanced preprocessing."""

import logging
import cv2
import numpy as np
from pyzbar import pyzbar
import string
from typing import Tuple, Optional

logger = logging.getLogger(__name__)


class QRDecoder:
    """Extract string payload from QR code targets with retry counters and image filters."""

    def __init__(self, max_attempts: int = 120):
        self.max_attempts = max_attempts
        self.attempt_counter = 0
        self.decoded_text = None
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

    def reset(self):
        """Reset decoder attempts and text cache state."""
        self.attempt_counter = 0
        self.decoded_text = None

    def _is_valid_payload(self, text: str) -> bool:
        """Basic validation to reject pyzbar hallucinations."""
        if not text or len(text.strip()) < 3:
            return False
        # Ensure all characters are printable ASCII
        return all(c in string.printable for c in text)

    def decode(self, frame: np.ndarray, last_bbox: Optional[np.ndarray] = None) -> Tuple[bool, Optional[str], bool]:
        """
        Attempt to extract and decode a QR code from the given frame.

        Args:
            frame: Raw BGR camera image frame

        Returns:
            success: Decoded text was retrieved successfully
            text: The decoded QR string payload
            final: Processing cycle completed (success or max retries exceeded)
        """
        if frame is None or frame.size == 0:
            return False, None, False

        self.attempt_counter += 1
        
        try:
            # Preprocess to optimize contrast for decoding
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Apply adaptive histogram equalization
            enhanced = self.clahe.apply(gray)
            
            # Form list of image variations to try (raw, enhanced, sharpened, thresholds)
            # Denoise while keeping edges sharp
            denoised = cv2.bilateralFilter(enhanced, 9, 75, 75)
            
            # Sharpening kernel
            blur = cv2.GaussianBlur(enhanced, (0, 0), 1.0)
            sharpened = cv2.addWeighted(enhanced, 2.0, blur, -1.0, 0)
            
            variants = [enhanced, denoised, sharpened, gray]
            
            # Try adaptive thresholding variations
            for method in [cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.ADAPTIVE_THRESH_MEAN_C]:
                thresh = cv2.adaptiveThreshold(enhanced, 255, method, cv2.THRESH_BINARY, 21, 10)
                variants.append(thresh)

            # First, if we have a last_bbox, try to decode cropped variants
            if last_bbox is not None:
                h, w = frame.shape[:2]
                rx = max(0, np.min(last_bbox[:, 0]) - 20)
                ry = max(0, np.min(last_bbox[:, 1]) - 20)
                rx2 = min(w, np.max(last_bbox[:, 0]) + 20)
                ry2 = min(h, np.max(last_bbox[:, 1]) + 20)
                
                for img in variants:
                    crop = img[ry:ry2, rx:rx2]
                    if crop.size == 0: continue
                    barcodes = pyzbar.decode(crop)
                    if barcodes:
                        for barcode in barcodes:
                            data = barcode.data.decode('utf-8', errors='ignore')
                            if self._is_valid_payload(data):
                                self.decoded_text = data
                                logger.info(f"QR payload decoded successfully from crop: {data}")
                                return True, data, True

            # Fall back to decoding the full-frame versions
            for img in variants:
                barcodes = pyzbar.decode(img)
                if barcodes:
                    for barcode in barcodes:
                        data = barcode.data.decode('utf-8', errors='ignore')
                        if self._is_valid_payload(data):
                            self.decoded_text = data
                            logger.info(f"QR payload decoded successfully: {data}")
                            return True, data, True

        except Exception as e:
            logger.debug(f"Error during pyzbar QR decode attempt: {e}")

        # Check if retry safety limits are breached
        if self.attempt_counter >= self.max_attempts:
            logger.warning(f"QR decode failed. Max attempts reached: {self.attempt_counter}/{self.max_attempts}")
            return False, None, True

        return False, None, False
