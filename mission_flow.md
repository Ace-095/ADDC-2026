# ADDC-2026 — Full Mission Flow & FSM Reference

Verified fixes (re-checked against current repo):
- **Infinite post-landing loop — FIXED.** `LAND` now guards with `if self.state != State.MISSION_COMPLETE` and transitions to a real terminal `MISSION_COMPLETE` state, which has no `tick()` dispatch branch — it idles silently (only the routine 2s heartbeat log fires).
- **`qr_size_cm` — FIXED.** Back to `21.0` in `config/vision.yaml`, matching the physical target and `AlignmentController`'s distance math.
- QR gate (`min_qr_pixel_width: 38`) and REASSERT_HOME/CLIMB — confirmed still correct, unchanged.

15 states total (16 with the crash-recovery `RECOVER` entry state added in the persistence work):
`BOOT → [RECOVER] → MONITOR_AUTO → REQUEST_GUIDED → GUIDED_HOLD → INITIAL_SCAN → SEARCH_SQUARE → RETURN_INITIAL → ALIGNMENT → QR_DECODE → LAND → REASSERT_HOME → CLIMB → RETURN_TO_ORIGIN → RTL → MISSION_COMPLETE`

## 0. RECOVER — crash-recovery entry (CHANGE 1)
Entered only at boot, when a valid on-disk checkpoint exists **and** the vehicle is armed + airborne (detected in BOOT). Validates the recovered context against live telemetry (armed/airborne, GPS fix ≥ min, no released/state contradiction), restores the persisted variables (anchor, true_home, payload status, QR text, search progress, landing context), and re-enters the last safe state. Resumable hold/navigation states resume directly; active closed-loop states demote to a safe parent (`REQUEST_GUIDED/ALIGNMENT/QR_DECODE → GUIDED_HOLD`, `LAND → RTL`). Any validation failure → documented reason + RTL. A grounded vehicle or absent/stale checkpoint → fresh start (MONITOR_AUTO), never a blind mid-air resume.

A checkpoint (`mission_state.json`) is written atomically after **every** FSM transition and after key variable changes (anchor, true_home, payload status, QR text, search waypoint index). `MISSION_COMPLETE` clears it so a later boot does not resume a finished mission.

---

## 1. BOOT
Waits for MAVLink connection (`fc.mav.is_connected()`). No branching — once connected → `MONITOR_AUTO`.

## 2. MONITOR_AUTO
Passive monitor while ArduCopter runs its own AUTO mission.
- **True home capture (once only):** on the first disarmed→armed edge, if GPS fix ≥ configured minimum (default 3), captures `true_home` = {GPS lat/lon/alt, NED x/y/z}. Edge-triggered so a later re-arm (post-drop) never overwrites it.
- **Mid-flight-reboot safety:** if the Pi boots and finds the FC already in GUIDED, assumes a companion-computer crash mid-mission → immediate `RTL` (never resumes blind).
- **Continuous takeoff safety check:** polls telemetry altitude each tick to arm the payload `takeoff_safety` gate once the threshold is crossed, unlocking the release logic for later.
- **Sprayer trigger (mission start):**
  - Primary: `STATUSTEXT` "Mission: N Sprayer" from ArduPilot → consume flag → `REQUEST_GUIDED`.
  - Fallback: cached mission item `MAV_CMD_DO_SPRAYER (223)` at current waypoint → same transition.
- No trigger yet → stays in AUTO, ArduCopter keeps flying its own waypoints.

## 3. REQUEST_GUIDED
Sends `SET_MODE(GUIDED)` once/sec. Confirmed via autopilot heartbeat → `GUIDED_HOLD`.
- **Timeout (CHANGE 2):** 3 retry cycles × 5s (15s total) unconfirmed → no longer a bare RTL. Instead the shared timeout-drop sequence runs: **LOITER hold (shed residual velocity, ~1.5s) → GUIDED descend to ~1 m AGL → LOITER stabilize (~1s) → drop payload → confirm release (bounded wait) → RTL.** (LOITER is the MAVLink-clean realization of "AltHold + hold XY": ArduPilot LOITER holds GPS position AND altitude, whereas ALT_HOLD does not hold XY and cannot accept companion velocity descent.)

## 4. GUIDED_HOLD
Holds position while capturing a `LOCAL_POSITION_NED` anchor (retried every tick until non-None).
- Anchor + min 2s settle → `INITIAL_SCAN`.
- **Hard timeout (CHANGE 3):** default 8s with still no telemetry → the **same** shared timeout-drop sequence as REQUEST_GUIDED (LOITER hold → descend ~1 m AGL → drop → confirm → RTL), never silently proceeding with `None`. Both timeouts behave identically by construction (shared sequencer).

## 5. INITIAL_SCAN — first QR look
Hovers at the anchor, checks the vision result each tick.
- **QR found (CHANGE 4):** a detection must clear the confidence threshold (`initial_scan_min_confidence`, default 0.6) for `initial_scan_confirm_frames` (default 3) **consecutive** frames before → `ALIGNMENT`. This kills single-frame false positives that previously oscillated the FSM in/out of ALIGNMENT. An optional slow in-place yaw sweep (`initial_scan_yaw_sweep`, default OFF) widens the camera footprint without translating XY.
- **QR not found, timeout (20s default)** → generates concentric-square search waypoints (rings from config, default [1.0, 2.0, square_size_m]) → `SEARCH_SQUARE`.

## 6. SEARCH_SQUARE — concentric-square search (CHANGE 5)
Flies the generated ring waypoints at a **reduced cruise speed (0.25 m/s)** with **tightened WPNAV accel/decel (0.5 m/s²)** and a **small waypoint radius (0.2 m)**, all applied at entry and restored on exit. Each waypoint is followed by a **hover dwell (`search_waypoint_dwell_s`, default 1.5 s)** so the camera stabilises (no motion blur) and the detector gets a clean, stationary look before the next transit. Waypoints advance only after dwell + a fresh vision frame is examined.
- **QR found (confidence-gated, same gate as INITIAL_SCAN)** → `ALIGNMENT`.
- **Vision result stale too long** (`vision_fail_limit` ticks) → fallback → `RTL`.
- **Dynamic timeout exceeded** (path-length/speed based) → `RETURN_INITIAL`.
- **Pattern fully walked, still nothing found** → `RETURN_INITIAL`.

## 7. RETURN_INITIAL — QR never found
Flies back to the `GUIDED_HOLD` anchor (not `true_home` — this is the pre-search hover point).
- **Arrived (within tolerance)** → speed + default WPNAV restored → `LAND` marked as a **blind landing** (`_blind_landing=True`).
- **30s timeout** → same blind `LAND` anyway.
- No `guided_anchor_ned` at all → straight to `RTL`.
- **Net effect of QR-not-found path (CHANGE 6):** the drone lands at the anchor, **releases the payload** (forced on a confirmed blind landing so the ultrasonic [0.2,0.4]m window gap on flat ground does not strand the drop), confirms release, then runs the full post-landing return-home sequence (re-arm → `REASSERT_HOME` → `CLIMB` → `RETURN_TO_ORIGIN` → platform landing → `MISSION_COMPLETE`). The payload is delivered even when no QR was ever locked.

## 8. ALIGNMENT — QR (or platform marker) found, centering
Runs every tick on the live vision result.
- **Vision result stale too long** → fallback → `RTL`.
- **Target lost** (not found): counts up to 40 ticks (2s) holding position; if still lost → back to `GUIDED_HOLD` (re-search from scratch, not `RTL` — recoverable).
- **RTK/vision cross-check** (mission context only): if GPS distance-to-waypoint > 5m while vision claims a lock → treated as a vision hallucination → fallback → `RTL`.
- **Aligned (PID converged + stable-frame count met):**
  - context = `mission` → `QR_DECODE`.
  - context = `platform` → straight to `LAND` (no decode step for the return pad).

## 9. QR_DECODE — mission context only
Holds position, waits for background pyzbar decode.
- **Decoded successfully** → sends text via `send_qr_text` → `LAND`.
- **Decode exhausted retries without success (`final=True`)** → logs failure, lands anyway → `LAND`.
- (Platform context skips this state entirely — see ALIGNMENT above.)

## 10. LAND — shared by both mission and platform landings
1. Retry-and-confirm `LAND` mode (3×5s max) → fallback `RTL` if never confirmed.
2. While descending: sends `LANDING_TARGET` messages from live vision detection (if found) for precision correction.
3. **RTK/vision cross-check** (mission only) during descent → fallback → `RTL`.
4. **30s hard timeout** if payload never released → fallback → `RTL`.
5. **Branch — `landing_context == 'platform'`:**
   - On touchdown (`is_landed()`), logs completion once, restores vision to `qr` mode, sends `MISSION COMPLETE` STATUSTEXT, transitions to `MISSION_COMPLETE` (terminal). **Mission ends here for the return-landing path.**
6. **Branch — `landing_context == 'mission'`:** payload release gate —
   - Real ultrasonic in `[0.2, 0.4]m` window + landed + takeoff-arm-safety passed → `trigger_release()`.
   - SITL: no real sensor, altitude reads `0.0` on touchdown → forced `in_window=True` so release still fires.
   - Arming-safety gate active but altitude met → logs warning, does **not** release (double safety).
   - **After release:** switches to GUIDED → re-arms (ArduCopter auto-disarms on touchdown) → once armed, → `REASSERT_HOME`.

## 11. REASSERT_HOME — mission context only, after payload drop
ArduCopter reset home to the QR pad on re-arm. Sends `MAV_CMD_DO_SET_HOME` (COMMAND_INT, once/sec) targeting `true_home`, confirms via `HOME_POSITION` broadcast (±5 degE7 tolerance, non-blocking cache read).
- **Confirmed** → `CLIMB`.
- **3 attempts (~3s) without confirmation** → fallback → `RTL` (deliberately refuses to proceed with unconfirmed home — wrong home would make RTL land back on the QR pad).
- `true_home` somehow `None` → immediate fallback → `RTL`.

## 12. CLIMB
Commands takeoff to `post_release_climb_altitude_m` (default 3.0m), confirms ascent (3 takeoff cmds + alt > 0.5m), then pushes up via velocity until target altitude reached.
- No branching — always → `RETURN_TO_ORIGIN` once altitude met.

## 13. RETURN_TO_ORIGIN
Flies to `true_home['ned']` at climb altitude (GPS-home-agnostic — uses NED so it's immune to ArduCopter's home resets).
- **Arrived, `landing_context == 'mission'`:** switches context to `platform`, sets vision to ArUco mode, → `REQUEST_GUIDED` (re-enters the whole GUIDED→ALIGNMENT→LAND pipeline, this time hunting the platform marker instead of a QR).
- **Arrived, already `platform` context** (i.e. reached via the RTL-handoff path instead): commands final land directly → `return_land_commanded=True`, holds.
- **30s timeout** → emergency land wherever it is.
- **No `true_home` captured at all** → emergency land immediately at current position.

## 14. RTL — fallback path only (never a "normal" step)
Any fallback trigger throughout the mission routes here. Commands ArduCopter's native RTL.
- **Mission context, near home** (< `rtl_handoff_distance_m`, default 5m, and < `rtl_handoff_altitude_m`, default 8m): hands off to `platform` context → `REQUEST_GUIDED` for a precision platform landing instead of a blind ArduCopter RTL touchdown.
- **Handoff never triggers within 60s** → ArduCopter completes its own blind RTL landing (no precision, no payload-safe handling).

## 15. MISSION_COMPLETE — terminal
No tick logic. Idles. Only the periodic state-name log line fires every 2s. Confirmed fix — no more per-tick spam/re-triggering.

---

## Full QR-detection outcome matrix

| Scenario | Path |
|---|---|
| QR found in INITIAL_SCAN | → ALIGNMENT → QR_DECODE → LAND (release) → REASSERT_HOME → CLIMB → RETURN_TO_ORIGIN → platform ALIGNMENT → LAND → MISSION_COMPLETE |
| QR found during SEARCH_SQUARE | same as above |
| QR never found (search exhausted/timeout) — CHANGE 6 | → RETURN_INITIAL → blind LAND → **release forced once landed** → confirm → GUIDED→re-arm→REASSERT_HOME→CLIMB→RETURN_TO_ORIGIN→platform landing → MISSION_COMPLETE |
| QR found then lost mid-ALIGNMENT (<2s) | holds, keeps trying same lock |
| QR found then lost mid-ALIGNMENT (≥2s) | → GUIDED_HOLD → re-search from INITIAL_SCAN |
| QR decode fails after max attempts | lands anyway without payload text confirmation |
| Vision hallucination (GPS/vision >5m mismatch) | abort → RTL |
| Platform marker never found on return | same SEARCH_SQUARE/RETURN_INITIAL logic, but landing_context stays 'platform' → blind LAND still transitions cleanly to MISSION_COMPLETE |
| Any fallback trigger, near home | RTL → auto-handoff to precision platform landing if within range/altitude |
| Any fallback trigger, far from home | RTL → ArduCopter blind landing (60s handoff timeout expired) |

## All fallback triggers (→ mission-safety net)
- **→ shared timeout-drop + RTL (CHANGES 2/3):** REQUEST_GUIDED timeout · GUIDED_HOLD anchor timeout. Both run LOITER-hold → descend ~1 m AGL → drop payload → confirm → RTL (never a bare RTL with the payload aboard).
- **→ bare RTL (safety net):** Mid-flight reboot in GUIDED **with no valid checkpoint** (with a checkpoint it resumes via RECOVER) · ALIGNMENT vision timeout · ALIGNMENT RTK/vision mismatch · SEARCH_SQUARE vision timeout · LAND mode timeout · LAND RTK/vision mismatch · LAND 30s release timeout · REASSERT_HOME unconfirmed after 3 attempts · RECOVER validation failure.
