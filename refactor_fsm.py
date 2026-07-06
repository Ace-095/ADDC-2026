import re

with open('core/state_machine.py', 'r') as f:
    content = f.read()

# 1. Update __init__
content = content.replace(
    "def __init__(self, config: dict, flight_control, camera_manager,\n                 qr_detector, alignment_controller, qr_decoder,\n                 payload_control: PayloadControl, fallback_manager):",
    "def __init__(self, config: dict, flight_control, vision_pipeline,\n                 payload_control: PayloadControl, fallback_manager):"
)
content = content.replace("self.cam = camera_manager", "self.vision = vision_pipeline")
content = content.replace("self.qr_det = qr_detector\n        self.align = alignment_controller\n        self.qr_dec = qr_decoder\n", "")

# 2. Update _transition
trans_old = """        # Reset counters on state entry
        self.vision_fail_counter = 0
        self.hold_counter = 0"""
trans_new = """        # Reset counters on state entry
        self.vision_fail_counter = 0
        self.hold_counter = 0
        
        # Enable expensive decoding only when necessary
        if self.state in (State.QR_DECODE, State.LAND):
            self.vision.set_request_decode(True)
        else:
            self.vision.set_request_decode(False)"""
content = content.replace(trans_old, trans_new)

# 3. Update INITIAL_SCAN
init_old = """        frame = self.cam.get_latest_frame()
        if frame is None:
            # Continue holding position, do not abort early if camera frame drops
            return

        # Look for target QR
        found, bbox, center = self.qr_det.detect(frame)
        if found:
            logger.info(f"Target QR locked at pixel coordinates: {center}")
            self._transition(State.ALIGNMENT, tick_count)
            return"""
init_new = """        vis = self.vision.get_latest_result()
        if vis['timestamp'] == 0.0:
            return

        if vis['found']:
            logger.info(f"Target QR locked at pixel coordinates: {vis['center']}")
            self._transition(State.ALIGNMENT, tick_count)
            return"""
content = content.replace(init_old, init_new)

# 4. Update SEARCH_SQUARE
search_old = """        frame = self.cam.get_latest_frame()
        if frame is None:
            self.fc.hold_position()
            self.vision_fail_counter += 1
            if self.vision_fail_counter >= self.vision_fail_limit:
                logger.error("BOUNDED SEARCH ABORT: Camera frame unavailable for too long.")
                self.fallback.handle_fail("SEARCH_SQUARE: Camera frame loss timeout")
                self._transition(State.RTL, tick_count)
            return

        # Look for target QR
        found, bbox, center = self.qr_det.detect(frame)
        if found:
            logger.info(f"Target QR locked at pixel coordinates: {center} during SEARCH_SQUARE")
            self._transition(State.ALIGNMENT, tick_count)
            return"""
search_new = """        vis = self.vision.get_latest_result()
        if vis['timestamp'] == 0.0:
            self.fc.hold_position()
            self.vision_fail_counter += 1
            if self.vision_fail_counter >= self.vision_fail_limit:
                logger.error("BOUNDED SEARCH ABORT: Vision result unavailable for too long.")
                self.fallback.handle_fail("SEARCH_SQUARE: Vision timeout")
                self._transition(State.RTL, tick_count)
            return

        if vis['found']:
            logger.info(f"Target QR locked at pixel coordinates: {vis['center']} during SEARCH_SQUARE")
            self._transition(State.ALIGNMENT, tick_count)
            return"""
content = content.replace(search_old, search_new)

# 5. Update ALIGNMENT
align_old = """        frame = self.cam.get_latest_frame()
        if frame is None:
            self.fc.hold_position()
            self.vision_fail_counter += 1
            if self.vision_fail_counter >= self.vision_fail_limit:
                logger.error("ALIGNMENT ABORT: Camera frame unavailable for too long.")
                self.fallback.handle_fail("ALIGNMENT: Camera frame loss timeout")
                self._transition(State.RTL, tick_count)
            return

        found, bbox, center = self.qr_det.detect(frame)
        if not found:
            self.vision_fail_counter += 1
            if self.vision_fail_counter > 40:  # Lost for 2 seconds
                logger.warning("Lost target track during alignment. Re-searching...")
                self._transition(State.GUIDED_HOLD, tick_count)
            else:
                self.fc.hold_position()
            return

        self.vision_fail_counter = 0

        # RTK GPS Cross-Check for Vision Hallucinations
        if self.fc.distance_to_wp() > 5.0:
            logger.error("RTK vs Vision drift mismatch! Target > 5m from GPS waypoint. Aborting.")
            self.fallback.handle_fail("RTK/Vision drift mismatch")
            self._transition(State.RTL, tick_count)
            return

        # Calculate pixel dimensions for scaling
        x_coords = bbox[:, 0]
        pixel_width = int(x_coords.max() - x_coords.min())
        
        # Pull telemetry alt
        altitude_m = self.fc.mav.get_altitude()
        
        # Calculate corrective velocities
        h, w = frame.shape[:2]
        vx, vy, aligned = self.align.compute(center, pixel_width=pixel_width, altitude_m=altitude_m, frame_size=(w, h))
        
        # Command horizontal adjustment with slow landing descent (vz = 0.1m/s)
        self.fc.send_velocity(vx, vy, vz=0.1)

        # Transition logic
        if aligned:"""
align_new = """        vis = self.vision.get_latest_result()
        if vis['timestamp'] == 0.0:
            self.fc.hold_position()
            self.vision_fail_counter += 1
            if self.vision_fail_counter >= self.vision_fail_limit:
                logger.error("ALIGNMENT ABORT: Vision result unavailable for too long.")
                self.fallback.handle_fail("ALIGNMENT: Vision timeout")
                self._transition(State.RTL, tick_count)
            return

        if not vis['found']:
            self.vision_fail_counter += 1
            if self.vision_fail_counter > 40:  # Lost for 2 seconds
                logger.warning("Lost target track during alignment. Re-searching...")
                self._transition(State.GUIDED_HOLD, tick_count)
            else:
                self.fc.hold_position()
            return

        self.vision_fail_counter = 0

        # RTK GPS Cross-Check for Vision Hallucinations
        if self.fc.distance_to_wp() > 5.0:
            logger.error("RTK vs Vision drift mismatch! Target > 5m from GPS waypoint. Aborting.")
            self.fallback.handle_fail("RTK/Vision drift mismatch")
            self._transition(State.RTL, tick_count)
            return

        # Use pre-computed velocities from vision pipeline
        vx, vy, aligned = vis['vx'], vis['vy'], vis['aligned']
        
        # Command horizontal adjustment with slow landing descent (vz = 0.1m/s)
        self.fc.send_velocity(vx, vy, vz=0.1)

        # Transition logic
        if aligned:"""
content = content.replace(align_old, align_new)

# 6. Update QR_DECODE
qr_dec_old = """        frame = self.cam.get_latest_frame()
        if frame is None:
            return

        # Attempt to decode payload on current frame, using tracked target box for crop
        success, text, final = self.qr_dec.decode(frame, last_bbox=self.qr_det.last_bbox)
        
        if success:
            logger.info(f"Target Payload Decoded: '{text}'")"""
qr_dec_new = """        vis = self.vision.get_latest_result()
        if vis['timestamp'] == 0.0:
            return

        # Check if background thread successfully decoded it
        success = vis['decode_success']
        text = vis['decode_text']
        
        if success:
            logger.info(f"Target Payload Decoded: '{text}'")"""
content = content.replace(qr_dec_old, qr_dec_new)

# 7. Update LAND (vision abort log check)
land_old = """            frame = self.cam.get_latest_frame()
            if frame is None:
                self.vision_fail_counter += 1
                if self.vision_fail_counter > 10:  # Allow 0.5s of dropped frames
                    logger.warning("Camera frames lost during LAND sequence.")
            else:
                found, bbox, center = self.qr_det.detect(frame)
                if not found:
                    self.vision_fail_counter += 1
                    if self.vision_fail_counter > 20:  # Allow 1.0s of lost tracking
                        logger.warning("Target tracking lost during LAND sequence.")"""
land_new = """            vis = self.vision.get_latest_result()
            if vis['timestamp'] == 0.0:
                self.vision_fail_counter += 1
                if self.vision_fail_counter > 10:  # Allow 0.5s of dropped frames
                    logger.warning("Vision results lost during LAND sequence.")
            else:
                if not vis['found']:
                    self.vision_fail_counter += 1
                    if self.vision_fail_counter > 20:  # Allow 1.0s of lost tracking
                        logger.warning("Target tracking lost during LAND sequence.")"""
content = content.replace(land_old, land_new)

with open('core/state_machine.py', 'w') as f:
    f.write(content)
