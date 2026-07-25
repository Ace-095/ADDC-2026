"""ArUco fiducial marker detector for platform precision landing.

Provides the same (found, bbox, center) return interface as QRDetector so the
AlignmentController and LANDING_TARGET angle math work without modification.
Uses OpenCV's built-in cv2.aruco — no additional dependencies beyond OpenCV.
"""

import logging
import cv2
import numpy as np
from typing import Tuple, Optional

logger = logging.getLogger(__name__)


class PlatformDetector:
    """Detect a specific ArUco marker for visual homing on the launch platform."""

    def __init__(self, marker_id: int = 0, marker_size_cm: float = 30.0,
                 aruco_dict_name: str = 'DICT_4X4_50',
                 min_width_px: int = 40):
        """
        Args:
            marker_id:       The ArUco marker ID expected on the platform.
            marker_size_cm:  Physical edge length of the printed marker (cm).
            aruco_dict_name: OpenCV ArUco dictionary enum name (e.g. 'DICT_4X4_50').
            min_width_px:    Minimum detected marker width in pixels — rejects
                             distant/noisy detections that are too small to align on.
        """
        self.target_id = marker_id
        self.marker_size_cm = marker_size_cm
        self.min_width_px = min_width_px

        # Resolve the OpenCV enum from the string name
        dict_enum = getattr(cv2.aruco, aruco_dict_name, None)
        if dict_enum is None:
            logger.error(
                f"Unknown ArUco dictionary '{aruco_dict_name}'. "
                f"Falling back to DICT_4X4_50."
            )
            dict_enum = cv2.aruco.DICT_4X4_50
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_enum)
        # BUGFIX: In OpenCV 4.6.0, cv2.aruco.DetectorParameters() returns a struct
        # that segfaults when passed to a background thread (VisionPipeline).
        # We MUST use DetectorParameters_create() which returns a thread-safe smart pointer.
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        logger.info(
            f"PlatformDetector initialised: dict={aruco_dict_name}, "
            f"target_id={marker_id}, size={marker_size_cm}cm, "
            f"min_width={min_width_px}px"
        )

    def detect(self, frame: np.ndarray) -> Tuple[bool, Optional[np.ndarray], Optional[Tuple[int, int]]]:
        """Scan frame for the target ArUco marker.

        Returns the same (found, bbox, center) triple as QRDetector.detect()
        so downstream alignment and LANDING_TARGET math can be reused unchanged.

        Args:
            frame: Input BGR image from the downward camera.

        Returns:
            found:  True if the target marker ID is visible and large enough.
            bbox:   (4, 2) int32 ndarray of corner points, or None.
            center: (cx, cy) pixel centre of the marker, or None.
        """
        if frame is None or frame.size == 0:
            return False, None, None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is None:
            return False, None, None

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id != self.target_id:
                continue

            # corners[i] is shape (1, 4, 2) — squeeze to (4, 2)
            pts = corners[i][0].astype(np.int32)
            width = int(np.max(pts[:, 0]) - np.min(pts[:, 0]))

            if width < self.min_width_px:
                logger.debug(
                    f"ArUco ID {marker_id} detected but too small "
                    f"({width}px < {self.min_width_px}px min)."
                )
                continue

            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
            logger.debug(
                f"Platform marker ID {marker_id} detected: "
                f"width={width}px, center=({cx}, {cy})"
            )
            return True, pts, (cx, cy)

        return False, None, None
