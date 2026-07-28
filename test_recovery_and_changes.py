#!/usr/bin/env python3
"""Unit tests for CHANGE 1-6 additions: crash recovery, timeout-drop, initial-
scan confirmation, search dwell, and QR-not-found forced release.

Run with the project venv:
    drone_venv/bin/python3 test_recovery_and_changes.py
"""

import logging
import os
import tempfile
import time
import yaml
from pathlib import Path

logging.basicConfig(level=logging.WARNING)

from core.state_machine import StateMachine, State, _RESUMABLE_STATES, _RECOVERY_DEMOTION
from core.mission_store import MissionStore


# ---------------------------------------------------------------------------
# Mock infrastructure (extends test_fsm.py mocks with the new API surface)
# ---------------------------------------------------------------------------

def _load_cfg():
    cfg = {}
    for yf in sorted(Path('config').glob("*.yaml")):
        with open(yf) as f:
            cfg.update(yaml.safe_load(f) or {})
    return cfg


class MockMav:
    def __init__(self):
        self.connected = True
        self.armed = False
        self.mode = "MANUAL"          # one of MANUAL/AUTO/GUIDED/LOITER/LAND
        self.fix = 3
        self.sprayer_detected = False
        self.pos = (-35.363262, 149.165237, 584.0)
        self.ned = (0.0, 0.0, -5.0)
        self.msg = []
        self.current_waypoint = 1
        self._messages = {}
        self._params_set = {}         # param_id -> value (CHANGE 5)

    def is_connected(self):            return self.connected
    def get_gps_fix_type(self):        return self.fix
    def get_global_position(self):     return self.pos
    def get_altitude(self):            return abs(self.ned[2])
    def get_local_position_ned(self):  return self.ned
    def send_statustext(self, txt, severity=6): self.msg.append(txt)
    def get_message(self, t):          return self._messages.get(t)
    def set_param(self, pid, val, ptype=9):  self._params_set[pid] = val; return True  # CHANGE 5
    def set_guided_yaw_rate(self, r):  return True

    def inject_home_position(self, lat_deg, lon_deg, offset_e7=0):
        class _HP:
            def __init__(self, lat, lon):
                self.latitude = lat; self.longitude = lon
        self._messages['HOME_POSITION'] = _HP(int(lat_deg * 1e7) + offset_e7,
                                              int(lon_deg * 1e7) + offset_e7)


class MockFC:
    def __init__(self):
        self.mav = MockMav()
        self._modes_set = []
        self._yaw_calls = []

    def is_armed(self):          return self.mav.armed
    def is_auto_mode(self):      return self.mav.mode == "AUTO"
    def is_guided_mode(self):    return self.mav.mode == "GUIDED"
    def is_land_mode(self):      return self.mav.mode == "LAND"
    def is_landed(self):         return False
    def get_local_position(self): return self.mav.ned
    def distance_to_wp(self):    return 0.0

    def set_guided_mode(self):   self.mav.mode = "GUIDED"; self._modes_set.append(4); return True
    def set_mode(self, cm):
        self._modes_set.append(cm)
        self.mav.mode = {4: "GUIDED", 5: "LOITER", 9: "LAND", 6: "RTL"}.get(cm, self.mav.mode)
        return True
    def set_loiter_mode(self):   return self.set_mode(5)
    def set_yaw_rate(self, r):   self._yaw_calls.append(r); return True
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
    def apply_search_nav_tuning(self, accel, decel, wp_radius): return True
    def restore_default_nav_tuning(self, accel, decel, wp_radius): return True
    def set_home_precise(self, lat, lon, alt):
        self._home_set_calls = getattr(self, '_home_set_calls', 0) + 1
        return True


class MockPayload:
    def __init__(self):
        self.payload_released = False
        self.takeoff_detected = True
        self.use_sitl = True
    def check_takeoff_safety(self, alt): pass
    def get_distance_reading(self):      return 0.0
    def is_in_release_window(self, d):  return True
    def trigger_release(self):
        # Simulate the async servo sweep completing the release immediately.
        self.payload_released = True


class MockFallback:
    def __init__(self):
        self.fail_reasons = []
    def handle_fail(self, reason): self.fail_reasons.append(reason)
    def blind_land(self): pass


class MockVision:
    """Controllable vision stub — can toggle found/confidence per test."""
    def __init__(self):
        self.mode = 'qr'
        self.align = type('A', (), {'reset': lambda self: None})()
        self.qr_dec = type('D', (), {'reset': lambda self: None})()
        self._found = False
        self._confidence = 0.0
        self._decode_success = False
        self._decode_text = None
    def set_detection_mode(self, m): self.mode = m
    def set_request_decode(self, r): pass
    def set_result(self, found, confidence=0.0, decode_success=False, decode_text=None):
        self._found = found; self._confidence = confidence
        self._decode_success = decode_success; self._decode_text = decode_text
    def get_latest_result(self):
        return {
            'found': self._found, 'center': (100, 100), 'aligned': True,
            'decode_success': self._decode_success, 'decode_text': self._decode_text,
            'vx': 0, 'vy': 0, 'decode_final': True, 'confidence': self._confidence,
            'bbox': None, 'frame': None, 'timestamp': time.time(),
        }


def _make_sm(store=None):
    cfg = _load_cfg()
    fc = MockFC(); payload = MockPayload(); fallback = MockFallback(); vision = MockVision()
    if store is None:
        d = tempfile.mkdtemp()
        store = MissionStore(os.path.join(d, 'mission_state.json'))
    sm = StateMachine(cfg, fc, vision, payload, fallback, mission_store=store)
    return sm, fc, payload, fallback, vision, store


# ---------------------------------------------------------------------------
# CHANGE 1 — MissionStore atomicity + tolerance
# ---------------------------------------------------------------------------

def test_mission_store_atomic_and_tolerant():
    d = tempfile.mkdtemp(); p = os.path.join(d, 'mission_state.json')
    ms = MissionStore(p)
    assert ms.load() is None, "fresh store should load None"
    ck = {'state': 'SEARCH_SQUARE', 'guided_anchor_ned': [1.0, 2.0, -5.0],
          'payload_released': False, 'search_wp_idx': 3}
    assert ms.save(ck) is True
    loaded = ms.load()
    assert loaded['state'] == 'SEARCH_SQUARE' and loaded['search_wp_idx'] == 3
    assert loaded['schema_version'] == 1 and 'saved_at' in loaded
    # no leftover temp files
    assert not any(f.startswith('.mission_state.') for f in os.listdir(d))
    # corrupt file -> None (no exception)
    with open(p, 'w') as f: f.write('{not valid')
    assert ms.load() is None
    # old schema -> None
    import json
    with open(p, 'w') as f: json.dump({'schema_version': 0}, f)
    assert ms.load() is None
    # clear
    assert ms.clear() is True and not os.path.exists(p)
    print("PASS: test_mission_store_atomic_and_tolerant")


# ---------------------------------------------------------------------------
# CHANGE 1 — recovery: fresh start when grounded / no checkpoint
# ---------------------------------------------------------------------------

def test_recovery_fresh_start_when_grounded():
    sm, fc, _, _, _, store = _make_sm()
    # checkpoint present but vehicle grounded -> must NOT resume
    store.save({'state': 'SEARCH_SQUARE', 'schema_version_placeholder': True}
               | {'guided_anchor_ned': [1, 2, -5]})
    fc.mav.armed = False
    fc.mav.ned = (0.0, 0.0, 0.0)
    sm.tick(1)  # BOOT -> connected, probes recovery
    assert sm.state == State.MONITOR_AUTO, f"grounded vehicle must fresh-start, got {sm.state}"
    print("PASS: test_recovery_fresh_start_when_grounded")


def test_recovery_resume_when_airborne_with_checkpoint():
    sm, fc, _, _, _, store = _make_sm()
    # armed + airborne + valid checkpoint of a resumable state -> RECOVER -> resume
    fc.mav.armed = True
    fc.mav.ned = (0.0, 0.0, -5.0)        # 5m up
    fc.mav.fix = 3
    anchor = [1.0, 2.0, -5.0]
    store.save({
        'state': 'SEARCH_SQUARE', 'schema_version': 1, 'saved_at': time.time(),
        'guided_anchor_ned': anchor, 'true_home': {'gps': [-35.3, 149.1, 584.0], 'ned': [0, 0, -5]},
        'payload_released': False, 'payload_armed': True, 'qr_text': None,
        'search_wp_idx': 2, 'search_timeout_ticks': 1200, 'landing_context': 'mission',
    })
    sm.tick(1)  # BOOT probes -> RECOVER
    assert sm.state == State.RECOVER, f"expected RECOVER, got {sm.state}"
    sm.tick(2)  # RECOVER validates + resumes
    assert sm.state == State.SEARCH_SQUARE, f"expected resume to SEARCH_SQUARE, got {sm.state}"
    assert sm.guided_anchor_ned == (1.0, 2.0, -5.0), "anchor must be restored"
    assert sm.true_home is not None and sm.current_wp_idx == 2, "search progress restored"
    print("PASS: test_recovery_resume_when_airborne_with_checkpoint")


def test_recovery_demotes_active_state_to_safe_parent():
    sm, fc, _, _, _, store = _make_sm()
    fc.mav.armed = True; fc.mav.ned = (0, 0, -5); fc.mav.fix = 3
    store.save({
        'state': 'ALIGNMENT', 'schema_version': 1, 'saved_at': time.time(),
        'guided_anchor_ned': [1, 2, -5], 'true_home': {'gps': [-35.3, 149.1, 584], 'ned': [0, 0, -5]},
        'payload_released': False, 'landing_context': 'mission',
    })
    sm.tick(1)
    assert sm.state == State.RECOVER
    sm.tick(2)
    # ALIGNMENT is active-control -> demote to GUIDED_HOLD
    assert sm.state == State.GUIDED_HOLD, f"expected demotion to GUIDED_HOLD, got {sm.state}"
    print("PASS: test_recovery_demotes_active_state_to_safe_parent")


def test_recovery_refutes_released_but_pre_release_state():
    sm, fc, _, _, _, store = _make_sm()
    fc.mav.armed = True; fc.mav.ned = (0, 0, -5); fc.mav.fix = 3
    # contradictory: payload released but state claims SEARCH_SQUARE
    store.save({
        'state': 'SEARCH_SQUARE', 'schema_version': 1, 'saved_at': time.time(),
        'payload_released': True, 'guided_anchor_ned': [1, 2, -5],
        'landing_context': 'mission',
    })
    sm.tick(1); sm.tick(2)
    assert sm.state == State.RTL, f"contradiction must -> RTL, got {sm.state}"
    print("PASS: test_recovery_refutes_released_but_pre_release_state")


def test_recovery_resumes_post_release_state():
    sm, fc, payload, _, _, store = _make_sm()
    fc.mav.armed = True; fc.mav.ned = (0, 0, -5); fc.mav.fix = 3
    payload.payload_released = True
    store.save({
        'state': 'CLIMB', 'schema_version': 1, 'saved_at': time.time(),
        'payload_released': True, 'payload_armed': True,
        'true_home': {'gps': [-35.3, 149.1, 584], 'ned': [0, 0, -5]},
        'landing_context': 'mission',
    })
    sm.tick(1); sm.tick(2)
    assert sm.state == State.CLIMB, f"post-release state must resume, got {sm.state}"
    assert payload.payload_released is True, "release flag must stay set"
    print("PASS: test_recovery_resumes_post_release_state")


# ---------------------------------------------------------------------------
# CHANGES 2/3 — shared timeout-drop sequence
# ---------------------------------------------------------------------------

def test_request_guided_timeout_runs_drop_sequence():
    sm, fc, payload, fallback, vision, _ = _make_sm()
    # Reach REQUEST_GUIDED: armed, AUTO, sprayer fired
    fc.mav.armed = True; fc.mav.mode = "AUTO"; fc.mav.sprayer_detected = True
    sm.tick(1)  # BOOT -> MONITOR_AUTO
    sm.tick(2)  # sprayer -> REQUEST_GUIDED
    assert sm.state == State.REQUEST_GUIDED
    # Make GUIDED never confirm: set_guided_mode becomes a no-op so the autopilot
    # heartbeat never reports GUIDED (forces the timeout path).
    fc.set_guided_mode = lambda: True
    # 3 retry cycles × ~100 ticks each (5s at 20Hz) = the hard timeout.
    t = 3
    for _ in range(3 * 101):
        sm.tick(t); t += 1
    assert sm._drop_phase == 'hold', f"timeout should arm drop sequence at 'hold', got {sm._drop_phase}"
    assert any("REQUEST_GUIDED" in r for r in fallback.fail_reasons), "fallback reason logged"
    # Drive the sequencer: hold(1.5s) -> descend -> stabilize -> drop -> RTL
    tick_hz = sm.cfg['system']['tick_hz']
    for _ in range(int(1.5 * tick_hz) + 3):
        sm.tick(t); t += 1
    assert sm._drop_phase == 'descend', f"after hold should descend, got {sm._drop_phase}"
    # make altitude already ~1m so descend completes quickly
    fc.mav.ned = (0, 0, -1.0)
    # Drive stabilize (~1s) then drop. The mock's trigger_release completes the
    # release synchronously, so once the 'drop' phase fires the payload flag is
    # set immediately and the next drop-phase tick transitions to RTL. Keep
    # ticking until we either land in RTL or exhaust a generous budget.
    released_and_rtl = False
    for _ in range(int(2.0 * tick_hz) + 30):
        sm.tick(t); t += 1
        if sm.state == State.RTL:
            released_and_rtl = True
            break
    assert released_and_rtl, f"drop sequence should complete to RTL, got state={sm.state} phase={sm._drop_phase}"
    assert payload.payload_released is True, "payload must be released in drop sequence"
    print("PASS: test_request_guided_timeout_runs_drop_sequence")


def test_guided_hold_timeout_runs_same_drop_sequence():
    sm, fc, payload, fallback, vision, _ = _make_sm()
    # Put the FSM directly into GUIDED_HOLD with no anchor (forces hard timeout).
    fc.mav.armed = True; fc.mav.mode = "GUIDED"
    sm._transition(State.GUIDED_HOLD, 0)
    # Tick past the anchor timeout (8s = 160 ticks) with get_local_position returning None.
    fc.get_local_position = lambda: None
    fc.mav.get_local_position_ned = lambda: None
    for i in range(165):
        sm.tick(1 + i)
    assert sm._drop_phase == 'hold', f"GUIDED_HOLD timeout should arm drop sequence, got {sm._drop_phase}"
    assert any("GUIDED_HOLD" in r for r in fallback.fail_reasons), "GUIDED_HOLD fallback reason logged"
    print("PASS: test_guided_hold_timeout_runs_same_drop_sequence")


# ---------------------------------------------------------------------------
# CHANGE 4 — initial-scan confirmation gate
# ---------------------------------------------------------------------------

def test_initial_scan_requires_consecutive_confident_frames():
    sm, fc, _, _, vision, _ = _make_sm()
    fc.mav.armed = True; fc.mav.mode = "GUIDED"
    # Force a valid anchor so GUIDED_HOLD advances quickly.
    sm._transition(State.INITIAL_SCAN, 0)
    # Sub-threshold single detection must NOT transition (confidence 0.3 < 0.6)
    vision.set_result(True, confidence=0.3)
    sm.tick(1); sm.tick(2)
    assert sm.state == State.INITIAL_SCAN, "sub-threshold detection must not trigger"
    assert sm._confirm_counter == 0, "sub-threshold should reset counter"
    # One confident frame is not enough (need 3)
    vision.set_result(True, confidence=0.9)
    sm.tick(3)
    assert sm.state == State.INITIAL_SCAN and sm._confirm_counter == 1
    # A miss resets the streak
    vision.set_result(False, confidence=0.0)
    sm.tick(4)
    assert sm._confirm_counter == 0
    # Three consecutive confident frames -> ALIGNMENT
    vision.set_result(True, confidence=0.9)
    sm.tick(5); sm.tick(6); sm.tick(7)
    assert sm.state == State.ALIGNMENT, f"3 confident frames should -> ALIGNMENT, got {sm.state}"
    print("PASS: test_initial_scan_requires_consecutive_confident_frames")


# ---------------------------------------------------------------------------
# CHANGE 5 — search waypoint dwell
# ---------------------------------------------------------------------------

def test_search_waypoint_dwell_holds_before_advancing():
    sm, fc, _, _, vision, _ = _make_sm()
    fc.mav.armed = True; fc.mav.mode = "GUIDED"
    vision.set_result(False, confidence=0.0)
    sm.guided_anchor_ned = (0.0, 0.0, -5.0)
    sm._generate_search_pattern()
    sm._transition(State.SEARCH_SQUARE, 0)
    # current position = anchor -> first waypoint (SW corner) is ~half-ring away.
    # Force arrival: make current pos equal to the first waypoint.
    wp0 = sm.search_waypoints[0]
    fc.mav.ned = wp0
    fc.get_local_position = lambda: wp0
    fc.mav.get_local_position_ned = lambda: wp0
    sm.tick(1)
    # On arrival, a dwell gate should be armed (not yet advanced)
    assert sm._search_dwell_until_tick > 1, "dwell gate should arm on arrival"
    idx_before = sm.current_wp_idx
    # Within dwell window -> still same waypoint
    sm.tick(2)
    assert sm.current_wp_idx == idx_before, "must hold during dwell"
    # After dwell expires -> advance
    for i in range(3, int(sm._search_dwell_until_tick) + 2):
        sm.tick(i)
    assert sm.current_wp_idx == idx_before + 1, "must advance after dwell"
    print("PASS: test_search_waypoint_dwell_holds_before_advancing")


# ---------------------------------------------------------------------------
# CHANGE 6 — QR-not-found forced release on blind landing
# ---------------------------------------------------------------------------

def test_qr_not_found_blind_landing_forces_release():
    sm, fc, payload, _, vision, _ = _make_sm()
    fc.mav.armed = True; fc.mav.mode = "GUIDED"
    vision.set_result(False, confidence=0.0)
    sm.guided_anchor_ned = (0.0, 0.0, -5.0)
    # Drive the real QR-not-found path: RETURN_INITIAL arrival -> LAND, which sets
    # _blind_landing AFTER the transition (matching the state machine's real flow).
    sm._transition(State.RETURN_INITIAL, 0)
    # Simulate arrival at the anchor: current pos == anchor.
    anchor = sm.guided_anchor_ned
    fc.mav.ned = anchor
    fc.get_local_position = lambda: anchor
    fc.mav.get_local_position_ned = lambda: anchor
    sm.tick(1)  # RETURN_INITIAL detects arrival -> LAND + sets _blind_landing
    assert sm.state == State.LAND, f"arrival should -> LAND, got {sm.state}"
    assert sm._blind_landing is True, "RETURN_INITIAL arrival must mark blind landing"
    # Confirm LAND mode so the release gate runs.
    fc.mav.mode = "LAND"
    fc.is_landed = lambda: True
    # distance reading 0.0 -> not in [0.2,0.4] window normally, but blind+landed forces it
    payload.is_in_release_window = lambda d: (0.2 <= d <= 0.4)
    payload.get_distance_reading = lambda: 0.0
    sm.tick(2)
    assert payload.payload_released is True, "blind landing must force release once landed"
    print("PASS: test_qr_not_found_blind_landing_forces_release")


def test_blind_landing_marker_reset_on_normal_land_entry():
    sm, fc, _, _, _, _ = _make_sm()
    fc.mav.armed = True; fc.mav.mode = "GUIDED"
    sm._blind_landing = True
    sm._transition(State.LAND, 0)  # normal LAND entry (not from RETURN_INITIAL)
    assert sm._blind_landing is False, "non-blind LAND entry must clear the marker"
    print("PASS: test_blind_landing_marker_reset_on_normal_land_entry")


# ---------------------------------------------------------------------------
# Resumable-state set sanity
# ---------------------------------------------------------------------------

def test_resumable_set_excludes_active_control():
    assert State.ALIGNMENT not in _RESUMABLE_STATES
    assert State.QR_DECODE not in _RESUMABLE_STATES
    assert State.LAND not in _RESUMABLE_STATES
    assert State.REQUEST_GUIDED not in _RESUMABLE_STATES
    assert State.GUIDED_HOLD in _RESUMABLE_STATES
    assert State.SEARCH_SQUARE in _RESUMABLE_STATES
    assert _RECOVERY_DEMOTION[State.LAND] == State.RTL
    assert _RECOVERY_DEMOTION[State.ALIGNMENT] == State.GUIDED_HOLD
    print("PASS: test_resumable_set_excludes_active_control")


if __name__ == '__main__':
    print("\n--- CHANGE 1-6 unit tests ---")
    test_mission_store_atomic_and_tolerant()
    test_recovery_fresh_start_when_grounded()
    test_recovery_resume_when_airborne_with_checkpoint()
    test_recovery_demotes_active_state_to_safe_parent()
    test_recovery_refutes_released_but_pre_release_state()
    test_recovery_resumes_post_release_state()
    test_request_guided_timeout_runs_drop_sequence()
    test_guided_hold_timeout_runs_same_drop_sequence()
    test_initial_scan_requires_consecutive_confident_frames()
    test_search_waypoint_dwell_holds_before_advancing()
    test_qr_not_found_blind_landing_forces_release()
    test_blind_landing_marker_reset_on_normal_land_entry()
    test_resumable_set_excludes_active_control()
    print("\nAll CHANGE 1-6 unit tests passed.\n")
