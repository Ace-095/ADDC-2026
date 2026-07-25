import logging
import time

logging.basicConfig(level=logging.INFO)

from core.state_machine import StateMachine, State

# ---------------------------------------------------------------------------
# Shared mock infrastructure
# ---------------------------------------------------------------------------

class MockConfig:
    def __init__(self):
        import yaml
        from pathlib import Path
        config_path = Path('config')
        self.cfg = {}
        for yf in sorted(config_path.glob("*.yaml")):
            with open(yf) as f:
                self.cfg.update(yaml.safe_load(f) or {})
    def get(self, key, default=None):
        return self.cfg.get(key, default)
    def __getitem__(self, key):
        return self.cfg[key]


class MockMav:
    """MAVLinkInterface stub aligned to the current API surface."""
    def __init__(self):
        self.connected = True
        self.armed = False
        self.mode = "MANUAL"
        self.fix = 3
        self.sprayer_detected = False
        self.pos = (-35.363262, 149.165237, 584.0)   # Canberra-ish
        self.ned = (0.0, 0.0, -10.0)
        self.msg = []
        self.current_waypoint = 1
        # Simulated message cache (mirrors MAVLinkInterface.messages dict)
        self._messages = {}

    def is_connected(self):       return self.connected
    def get_gps_fix_type(self):   return self.fix
    def get_global_position(self): return self.pos
    def get_altitude(self):        return abs(self.ned[2])
    def get_local_position_ned(self): return self.ned
    def send_statustext(self, txt, severity=6): self.msg.append(txt)
    def get_message(self, msg_type): return self._messages.get(msg_type)

    def inject_home_position(self, lat_deg, lon_deg, offset_e7=0):
        """Inject a synthetic HOME_POSITION message into the cache."""
        class _HP:
            def __init__(self, lat, lon):
                self.latitude  = lat
                self.longitude = lon
        self._messages['HOME_POSITION'] = _HP(
            int(lat_deg * 1e7) + offset_e7,
            int(lon_deg * 1e7) + offset_e7,
        )


class MockFC:
    """FlightControl stub aligned to the current API surface."""
    def __init__(self):
        self.mav = MockMav()
        self._home_set_calls = 0

    def is_armed(self):          return self.mav.armed
    def is_auto_mode(self):      return self.mav.mode == "AUTO"
    def is_guided_mode(self):    return self.mav.mode == "GUIDED"
    def is_land_mode(self):      return self.mav.mode == "LAND"
    def is_landed(self):         return False
    def get_local_position(self): return self.mav.ned
    def distance_to_wp(self):    return 0.0

    def set_guided_mode(self):   self.mav.mode = "GUIDED"; return True
    def arm(self):               self.mav.armed = True; return True
    def land(self):              self.mav.mode = "LAND"; return True
    def rtl(self):               return True
    def takeoff(self, alt):      return True
    def hold_position(self):     return True
    def send_velocity(self, vx, vy, vz): return True
    def goto_local_position(self, x, y, z): return True
    def send_landing_target(self, ax, ay, d): return True
    def set_search_speed(self, s):   return True
    def restore_normal_speed(self, s): return True
    def send_qr_text(self, t):   return True

    def set_home_precise(self, lat_deg, lon_deg, alt_m):
        self._home_set_calls += 1
        return True


class MockPayload:
    def __init__(self):
        self.payload_released = False
        self.takeoff_detected = True
        self.use_sitl = True
    def check_takeoff_safety(self, alt): pass
    def get_distance_reading(self):      return 0.0
    def is_in_release_window(self, d):  return True
    def trigger_release(self):           self.payload_released = True


class MockFallback:
    def __init__(self):
        self.fail_reasons = []
    def handle_fail(self, reason):
        self.fail_reasons.append(reason)
    def blind_land(self): pass


class MockAlign:
    def reset(self): pass
    def compute(self, c, pw, alt, fs, mode): return (0, 0, True)


class MockQRDec:
    def reset(self): pass
    def decode(self, frame, last_bbox): return (True, "DELIVERY_OK", True)


class MockVision:
    def __init__(self):
        self.mode = 'qr'
        self.align = MockAlign()
        self.qr_dec = MockQRDec()
    def set_detection_mode(self, mode): self.mode = mode
    def set_request_decode(self, req):  pass
    def get_latest_result(self):
        return {
            'found': True, 'center': (100, 100), 'aligned': True,
            'decode_success': True, 'decode_text': "DELIVERY_OK",
            'vx': 0, 'vy': 0, 'decode_final': True,
            'bbox': None, 'frame': None,
            'timestamp': time.time(),
        }


def _make_sm():
    """Return (sm, fc, payload, fallback) with fresh mocks."""
    config   = MockConfig()
    fc       = MockFC()
    payload  = MockPayload()
    fallback = MockFallback()
    vision   = MockVision()
    sm = StateMachine(config, fc, vision, payload, fallback)
    return sm, fc, payload, fallback


# ---------------------------------------------------------------------------
# Bench test 1 — true_home not overwritten on re-arm
# ---------------------------------------------------------------------------

def test_true_home_not_overwritten_on_rearm():
    """
    Invariant: true_home must be set exactly once (first arm edge) and never
    overwritten by a subsequent disarm->rearm cycle.
    """
    sm, fc, _, _ = _make_sm()

    # Boot -> MONITOR_AUTO
    sm.tick(1)
    assert sm.state == State.MONITOR_AUTO

    # Unarmed tick — true_home must stay None
    sm.tick(2)
    assert sm.true_home is None, "true_home should be None before first arm"

    # First arm edge -> capture true_home
    fc.mav.armed = True
    sm.tick(3)
    assert sm.true_home is not None, "true_home not captured on first arm"
    captured = sm.true_home.copy()
    print("  1a: true_home captured on first arm")

    # Disarm -> true_home unchanged, _was_armed resets
    fc.mav.armed = False
    sm.tick(4)
    assert sm.true_home == captured, "true_home should not clear on disarm"
    assert sm._was_armed is False,   "_was_armed should be False after disarm tick"

    # Re-arm -> edge fires again, but `is None` guard prevents overwrite
    fc.mav.pos = (-35.400000, 149.200000, 590.0)   # different location
    fc.mav.armed = True
    sm.tick(5)
    assert sm.true_home == captured, "true_home must not be overwritten on re-arm"
    print("  1b: true_home not overwritten on re-arm")
    print("PASS: test_true_home_not_overwritten_on_rearm")


# ---------------------------------------------------------------------------
# Bench test 2 — REASSERT_HOME confirms on 3rd attempt
# ---------------------------------------------------------------------------

def test_reassert_home_confirms_on_3rd_attempt():
    """
    HOME_POSITION cache mismatches on first two sends, matches on the 3rd.
    FSM must reach CLIMB (not RTL or stuck in REASSERT_HOME).
    """
    sm, fc, _, _ = _make_sm()

    lat, lon, alt = -35.363262, 149.165237, 584.0
    sm.true_home = {'gps': (lat, lon, alt), 'ned': (0.0, 0.0, -10.0)}
    sm._transition(State.REASSERT_HOME, 0)
    assert sm.state == State.REASSERT_HOME

    # Inject mismatched HOME_POSITION (offset by 1000 degE7 units)
    fc.mav.inject_home_position(lat, lon, offset_e7=1000)

    sm.tick(20)   # attempt 1 sent; cache still wrong
    assert sm.state == State.REASSERT_HOME, "Should still wait after attempt 1"
    assert sm.reassert_attempts == 1

    sm.tick(40)   # attempt 2 sent; cache still wrong
    assert sm.state == State.REASSERT_HOME, "Should still wait after attempt 2"
    assert sm.reassert_attempts == 2

    # Correct HOME_POSITION arrives before 3rd poll
    fc.mav.inject_home_position(lat, lon, offset_e7=0)
    sm.tick(60)   # attempt 3 sent; cache now matches
    assert sm.state == State.CLIMB,      f"Expected CLIMB, got {sm.state}"
    assert sm.home_restored is True,     "home_restored must be True"
    print("PASS: test_reassert_home_confirms_on_3rd_attempt")


# ---------------------------------------------------------------------------
# Bench test 3 — REASSERT_HOME failure -> FallbackManager, transition to RTL
# ---------------------------------------------------------------------------

def test_reassert_home_fallback_on_all_fail():
    """
    All 3 HOME_POSITION responses mismatch -> FallbackManager.handle_fail()
    is called and FSM transitions to RTL rather than silently taking off.
    """
    sm, fc, _, fallback = _make_sm()

    lat, lon, alt = -35.363262, 149.165237, 584.0
    sm.true_home = {'gps': (lat, lon, alt), 'ned': (0.0, 0.0, -10.0)}
    sm._transition(State.REASSERT_HOME, 0)

    # Always-wrong HOME_POSITION in cache throughout
    fc.mav.inject_home_position(lat, lon, offset_e7=9999)

    sm.tick(20)   # attempt 1
    sm.tick(40)   # attempt 2
    sm.tick(60)   # attempt 3 -> should abort

    assert sm.state == State.RTL,          f"Expected RTL, got {sm.state}"
    assert len(fallback.fail_reasons) > 0, "FallbackManager.handle_fail not called"
    assert "REASSERT_HOME" in fallback.fail_reasons[-1], \
        f"Expected 'REASSERT_HOME' in reason, got: {fallback.fail_reasons[-1]}"
    assert sm.home_restored is False,      "home_restored must stay False on failure"
    print("PASS: test_reassert_home_fallback_on_all_fail")


# ---------------------------------------------------------------------------
# Legacy integration smoke-test (original run())
# ---------------------------------------------------------------------------

def run():
    """Original integration smoke-test (kept for regression reference)."""
    config   = MockConfig()
    fc       = MockFC()
    payload  = MockPayload()
    fallback = MockFallback()
    vision   = MockVision()

    sm = StateMachine(config, fc, vision, payload, fallback)

    sm.tick(1)
    sm.tick(2)
    assert sm.true_home is None

    fc.mav.armed = True
    sm.tick(3)
    assert sm.true_home is not None
    print("TRUE HOME CAPTURED")

    fc.mav.mode = "AUTO"
    fc.mav.sprayer_detected = True

    ticks = 4
    for _ in range(100):
        sm.tick(ticks)
        state = sm.state

        if state == State.REQUEST_GUIDED:
            fc.mav.mode = "GUIDED"

        if sm.home_restored and not getattr(sm, '_home_ack_printed', False):
            print("DO_SET_HOME ACKNOWLEDGED")
            sm._home_ack_printed = True

        ticks += 1
        time.sleep(0.01)


if __name__ == '__main__':
    print("\n--- Running REASSERT_HOME bench tests ---")
    test_true_home_not_overwritten_on_rearm()
    test_reassert_home_confirms_on_3rd_attempt()
    test_reassert_home_fallback_on_all_fail()
    print("\nAll bench tests passed.\n")
    print("--- Running legacy smoke-test ---")
    run()
