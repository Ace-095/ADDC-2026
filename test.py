import cv2
import numpy as np
import threading
from vision.platform_detector import PlatformDetector

det = PlatformDetector()
det.aruco_params = cv2.aruco.DetectorParameters_create()
def run():
    det.detect(np.zeros((100, 100, 3), dtype=np.uint8))
    
t = threading.Thread(target=run)
t.start()
t.join()
print("Done")
