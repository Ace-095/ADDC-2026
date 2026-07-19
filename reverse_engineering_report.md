# COMPLETE SOFTWARE REVERSE ENGINEERING REPORT

## Executive Summary
The `drone_pi` repository is an autonomous companion computer system designed for drone precision delivery and platform landing operations. It interfaces with an ArduPilot-based flight controller (e.g., Pixhawk) via MAVLink over serial or TCP/UDP links. The system takes over flight control from the autopilot when an automated "Sprayer" mission item is reached, engaging a finite state machine (FSM) that leverages downward-facing cameras and OpenCV/pyzbar for target acquisition (QR codes and ArUco markers). It autonomously controls the drone's position via visual servoing to hover directly over a target, drops a physical payload using a GPIO/MAVLink-controlled servo mechanism governed by an ultrasonic sensor, and subsequently returns the drone precisely to its launch platform using ArUco-based visual homing.

## Project Purpose
This software solves the problem of "last-meter" precision in drone delivery. Standard GPS/RTK navigation can bring a drone to a general waypoint, but dropping a payload accurately requires visual confirmation and closed-loop control. The companion computer software achieves:
1. Identifying a drop zone via QR codes.
2. Descending precisely over the drop zone.
3. Automatically releasing a payload when the distance sensor confirms a safe drop height.
4. Reading the QR payload text for mission validation.
5. Ascending and flying back to the exact initial launch pad using an ArUco fiducial marker for a centimeter-accurate return landing.

## Technology Stack
- **Language**: Python 3
- **Autopilot Interface**: `pymavlink` (MAVLink protocol)
- **Computer Vision**: `OpenCV` (image processing, ArUco detection), `pyzbar` (QR detection & decoding)
- **Camera Interface**: `picamera2` (native Raspberry Pi cameras), `GStreamer` (Gazebo SITL streams), `OpenCV VideoCapture` (USB fallbacks)
- **Hardware Integration**: `gpiozero` (ultrasonic distance sensor)
- **Configuration Management**: `PyYAML`
- **Execution Environment**: Designed for Raspberry Pi 5 / ARM SBCs running Linux.
- **Simulation Environment**: Gazebo SITL with ArduPilot.

## High-Level Architecture
The system uses a highly concurrent, modular architecture separating high-frequency flight control loops from expensive computer vision tasks:

1. **Main Thread (Display & Coordination)**: Handles GUI rendering (OpenCV `imshow`) and graceful shutdown.
2. **FSM Tick Thread**: Runs at 20Hz (configurable). Owns the `StateMachine`, queries the flight controller, reads the vision cache, computes PID alignment, and dispatches MAVLink commands.
3. **MAVLink Thread (`MAVLinkInterface`)**: Runs continuously, reading telemetry packets asynchronously, tracking vehicle mode, and caching waypoints/heartbeats.
4. **Camera Thread (`CameraManager`)**: Continuously grabs frames from the hardware or simulation pipeline into a small rolling buffer to minimize latency.
5. **Vision Pipeline Thread (`VisionPipeline`)**: Polls the latest camera frame at up to 50Hz, runs the selected detector (QR or ArUco), computes metric position errors via `AlignmentController`, and stores results in a thread-safe dictionary for the FSM to consume.

This multi-threaded decoupled architecture ensures that the MAVLink heartbeat and flight control loops never block while waiting for a heavy pyzbar QR decoding cycle to finish.

## Folder Structure Analysis
- `/core`: The brain of the system. Contains the `StateMachine`, `MAVLinkInterface`, `FlightControl` abstraction, `PayloadControl` (servo/distance), `SystemMonitor` (thermal), and `FallbackManager`. Responsible for all flight logic and vehicle communication.
- `/vision`: The perception system. Contains the threaded `VisionPipeline`, `CameraManager` (multi-backend camera driver), `QRDetector` (optimized pyzbar wrapper), `QRDecoder` (text extraction), `PlatformDetector` (ArUco marker detection for return landing), and `AlignmentController` (visual servoing PID math).
- `/config`: Configuration layer. Contains YAML files (`alignment.yaml`, `camera.yaml`, `flight.yaml`, `mavlink.yaml`, `platform.yaml`, `system.yaml`, `vision.yaml`). Parsed and merged into a single dictionary on boot.
- `/utils`: Helper utilities. Contains `filters.py` (MedianFilter for ultrasonic sensor, PIDController for alignment). `logger.py` and `timers.py` exist but are currently empty.
- `/ardupilot`: (Excluded from deep analysis) The full ArduPilot flight controller source tree used for building the SITL environment.
- Root scripts (`main.py`, `test_fsm.py`, `verify_sitl.py`): Entry points, bench tests, and SITL launch wrappers.

## File-by-File Analysis
- `main.py`: Entry point. Parses CLI args, loads YAML configs, initializes all subsystems, spawns the threads, and runs the display and FSM tick loops. Handles graceful shutdown.
- `core/state_machine.py`: The heart of the application logic. A synchronous Finite State Machine (FSM). Manages the transition from BOOT to GUIDED flight modes, handles search patterns, alignment, payload decoding, landing, and return-to-launch (RTL) logic.
- `core/mavlink_interface.py`: Thread-safe driver for autopilot telemetry. Handles discovery of serial or TCP/UDP ports, auto-reconnection, sending guided velocity commands, parsing incoming packets asynchronously (caching for the FSM), and tracking latency.
- `core/flight_control.py`: Abstraction layer sitting above `mavlink_interface.py`. Provides clean methods like `is_armed()`, `goto_local_position()`, `takeoff()`, and `land()` for the FSM.
- `core/payload_control.py`: Manages the physical drop mechanism (servo) and ultrasonic distance sensor (`gpiozero`). Contains safety interlocks (`check_takeoff_safety`) to prevent accidental ground deployment and handles the asynchronous release sequence.
- `core/system_monitor.py`: Health check module. Reads `/sys/class/thermal/thermal_zone0/temp` to ensure the Pi 5 doesn't overheat (>85°C), triggering a fallback RTL if it does.
- `core/fallback_manager.py`: Centralized error handler. Currently commands a Return-To-Launch (RTL) upon any fail-safe trigger (e.g., vision timeout, thermal overload, stale heartbeat).
- `vision/vision_pipeline.py`: The background worker for CV. Pulls frames, invokes the active detector (QR or Platform), passes bounding boxes to the `AlignmentController`, and conditionally decodes the QR. Writes results to a thread-safe dict.
- `vision/camera_manager.py`: Manages image acquisition. Tries Gazebo GStreamer pipelines first, then falls back to `picamera2` native API, and finally standard OpenCV `VideoCapture` (USB webcams). Maintains a fast 2-frame ring buffer.
- `vision/qr_detector.py`: Scans frames for QR codes using `pyzbar`. Implements a multi-scale search, adaptive histogram equalization (CLAHE), and unsharp masking to enhance distant codes. Returns a bounding box.
- `vision/qr_decoder.py`: Dedicated to extracting the text string from the QR code. Tries multiple image filtering techniques (bilateral filtering, adaptive thresholds) if initial decodes fail.
- `vision/platform_detector.py`: Scans for ArUco fiducial markers (using `cv2.aruco`) for return precision landing. Operates as a drop-in replacement for `QRDetector` in the pipeline.
- `vision/alignment_controller.py`: Visual servoing logic. Translates pixel deviations from the camera center into real-world velocity vectors using PID controllers, accounting for altitude and field-of-view scaling.
- `utils/filters.py`: Contains a `PIDController` for alignment math with anti-windup, and a `MedianFilter` to smooth out noisy ultrasonic distance readings.
- `config/*.yaml`: Stores tunable parameters (PIDs, speeds, deadzones, camera settings).

## Runtime Execution Flow
1. **Application Startup**: `main.py` is invoked. Configs are merged from the `config/` dir. Subsystems are instantiated (`MAVLinkInterface`, `CameraManager`, `VisionPipeline`, `StateMachine`).
2. **Subsystem Threading**: `mav_interface`, `camera`, and `vision_pipeline` start their background threads.
3. **FSM Tick Loop**: The main thread spawns a 20Hz tick loop that continuously updates the `StateMachine`.
4. **Boot & Monitor**: The FSM waits for a Pixhawk connection (`BOOT`), then monitors (`MONITOR_AUTO`) for the drone to be armed and running an AUTO mission. It captures the `true_home` GPS/NED coordinates immediately upon arming.
5. **Mode Intercept**: The FSM detects a "Sprayer" waypoint (either via STATUSTEXT or MISSION_ITEM cache). It requests `GUIDED` mode (`REQUEST_GUIDED`).
6. **Guided Hold**: Once GUIDED is confirmed, it holds position (`GUIDED_HOLD`) to capture an origin anchor point in the `LOCAL_POSITION_NED` frame.
7. **Search Phase**: It hovers (`INITIAL_SCAN`) looking for the QR. If timed out, it generates a concentric square waypoint path and flies it (`SEARCH_SQUARE`). If still not found, it returns to the anchor point (`RETURN_INITIAL`).
8. **Alignment**: Once a QR is detected, it computes PID velocity vectors (`ALIGNMENT`) to center the drone over the target while slowly descending.
9. **Decoding**: Once stable, it requests the `VisionPipeline` to extract the QR text (`QR_DECODE`).
10. **Landing & Payload Release**: The FSM commands `LAND`. It monitors the ultrasonic sensor. When the drone passes below 0.4m, the servo opens, dropping the package.
11. **Return sequence**: After drop, it switches back to `GUIDED`, re-arms, takes off to 3m, and flies back to the original `true_home` coordinates (`RETURN_TO_ORIGIN`).
12. **Platform Homing**: Near home, it triggers `RTL`, which hands off to another `ALIGNMENT` phase, this time using the ArUco marker (`PlatformDetector`). It lands directly on the pad.
13. **Shutdown**: The GUI loop captures `q` keypress, sets `running = False`, joins all threads, and exits cleanly.

## Data Flow
- **Input (Telemetry)**: MAVLink packets arrive via TCP/Serial -> `MAVLinkInterface._connection_loop` -> parsed into a thread-safe dictionary (`self.messages`).
- **Input (Vision)**: Camera sensor -> `CameraManager` buffer -> `VisionPipeline` pops frame -> passes to Detector -> gets bbox -> passes to `AlignmentController` -> gets velocity vectors -> saves to `_latest_result` dict.
- **Processing (FSM)**: The 20Hz tick loop polls `FlightControl`/`MAVLinkInterface` for state (mode, altitude, position) and `VisionPipeline` for perception (aligned?, found?). It applies state transition logic.
- **Output (Commands)**: FSM calls `FlightControl.send_velocity` -> `MAVLinkInterface` packs a `SET_POSITION_TARGET_LOCAL_NED` MAVLink message -> transmitted to Pixhawk. FSM calls `PayloadControl` -> sends `DO_SET_SERVO` to actuate the drop mechanism.
- **Output (Logs/UI)**: `logger` outputs state changes to `drone_pi.log` and stdout. `main.py`'s `_display_loop` reads frames and overlays text for the GUI.

## Error Handling & Fallback Mechanisms
- **Stale Heartbeats**: `FlightControl` checks the age of the autopilot's last heartbeat. If >5s, mode checks (like `is_auto_mode`) return `False`, preventing stale state from causing errant transitions.
- **Mid-Flight Reboot Policy**: If the companion computer boots and finds the Pixhawk is already in `GUIDED` mode, it assumes a crash occurred mid-mission. It immediately commands `RTL` instead of resuming unsafely.
- **Vision Loss**: If tracking is lost during `ALIGNMENT` or `SEARCH_SQUARE`, a counter increments. If the target is lost for >2 seconds during alignment, it reverts to `GUIDED_HOLD`. If lost completely (e.g., 50s), `FallbackManager` issues `RTL`.
- **Hardware Failures**: The `SystemMonitor` checks CPU thermal load on every tick. If it exceeds 85°C, the FSM aborts immediately via `FallbackManager`.
- **Sensor Loss**: If the physical distance sensor (ultrasonic) fails, `PayloadControl` falls back to using the MAVLink relative altitude (`GLOBAL_POSITION_INT`) for the drop gate.
- **Command Retries**: Modes like `GUIDED` and `LAND` are commanded with a retry-and-confirm logic. If they fail to be acknowledged after 15 seconds, the mission aborts to `RTL`.
- **Boundary / Timeout limits**: Bounded searches (`SEARCH_SQUARE`) calculate a dynamic timeout based on path length and speed. If exceeded, it falls back to `RETURN_INITIAL`.

## Architecture Deep Dive
- **Design Patterns**: 
  - **Finite State Machine (FSM)**: The core logic uses a highly explicit, synchronous FSM pattern. Every state has an entry, tick loop, and transition exit.
  - **Observer/Event-Driven**: The FSM reacts to events from the threaded `MAVLinkInterface` and `VisionPipeline`.
  - **Strategy/Adapter**: `CameraManager` acts as an adapter, trying multiple backends (Gazebo GStreamer, picamera2, OpenCV fallback) to provide a unified `get_latest_frame()` interface.
  - **Thread-safe Single-writer/Multiple-reader Cache**: `MAVLinkInterface.messages` and `VisionPipeline._latest_result` are dictionaries wrapped by `threading.Lock()` providing decoupling between the high-frequency FSM and slow hardware/network reads.
- **State Management**: Handled explicitly in `state_machine.py`. State transitions clear specific tracking variables (like `guided_anchor_ned` or counters) and enforce preconditions (e.g. `true_home` capture).
- **Concurrency**:
  - Main Thread: GUI blocking `cv2.waitKey` and startup.
  - FSM Tick Thread: 20Hz synchronous loop.
  - MAVLink Thread: Asynchronous `recv_match` block.
  - Vision Pipeline Thread: Asynchronous compute loop (running at ~50Hz max) to prevent pyzbar decoding from halting the 20Hz flight control loop.
  - Camera Thread: Asynchronous image capture.
  - Payload Worker Thread: The servo release sequence is non-blocking to allow the FSM to continue monitoring altitude and heartbeat.

## Security Review
- **Secrets Management**: No explicit secrets (API keys) are used in the codebase, as it relies on local hardware and serial MAVLink links.
- **Authorization/Authentication**: MAVLink 1.0/2.0 is used without signing. This is standard for companion-to-autopilot serial links, but risky over open TCP/UDP (SITL).
- **Input Validation**: `QRDecoder` implements `_is_valid_payload` to reject non-printable ASCII noise generated by pyzbar hallucinations.
- **Access Control**: There is no OS-level access control on the web streams or GUI preview.
- **Vulnerabilities**: MAVLink injection is possible if the UDP/TCP ports are exposed to an untrusted network. The system currently blind-trusts any MAVLink packet with `sysid=1` as the autopilot.

## Performance Analysis
- **Bottlenecks**: 
  - `pyzbar` decoding is extremely CPU intensive. This was mitigated by moving it to the `VisionPipeline` background thread, and gated by the `request_decode` flag which is ONLY enabled during the `QR_DECODE` FSM state.
  - Unsharp masking and CLAHE in `QRDetector` add latency.
- **Optimization Opportunities**:
  - `MAVLinkInterface._request_targeted_streams()` successfully kills the default `MAV_DATA_STREAM_ALL` flood and replaces it with low-frequency targeted streams (4Hz position, 2Hz HUD), significantly reducing serial overhead and latency.
  - OpenCV fallback (MJPG stream) could be optimized for lower CPU usage by avoiding unnecessary BGR conversions where grayscale is sufficient.
  - Memory usage is tightly bound using `deque(maxlen=...)` for camera buffers and sensor histories.

## Dependency Mapping
- `main.py` -> `config/*.yaml`
- `main.py` -> `core.*`, `vision.*`
- `core.state_machine` -> `core.flight_control`, `vision.vision_pipeline`, `core.payload_control`, `core.fallback_manager`
- `core.flight_control` -> `core.mavlink_interface`
- `vision.vision_pipeline` -> `vision.camera_manager`, `vision.qr_detector`, `vision.qr_decoder`, `vision.alignment_controller`, `vision.platform_detector`
- `vision.alignment_controller` -> `utils.filters.PIDController`
- `core.payload_control` -> `utils.filters.MedianFilter`, `gpiozero`, `core.flight_control`
- `core.mavlink_interface` -> `pymavlink`

## Hidden Knowledge
- **SITL vs Real Hardware Assumptions**: The codebase is littered with SITL (Software In The Loop) specific workarounds. For instance, in `payload_control.py`, SITL lacks a real ultrasonic sensor, so the code falls back to the MAVLink relative altitude which artificially drops to `0.0m` instantly on touchdown, bypassing the `[0.2, 0.4]` safe release window. A specific `if self.payload.use_sitl and self.fc.is_landed():` hack forces the window to evaluate true.
- **Camera Orientation**: The `alignment.yaml` configuration hides a critical implicit assumption: the axis inversion flags (`invert_x`, `invert_y`) must match the physical mounting of the camera on the drone. If the camera is rotated 90 degrees, the drone will fly away from the target instead of towards it.
- **Home Position Overwrite**: ArduCopter automatically resets the "home" location whenever the drone is armed. Because the drone re-arms on the drop pad, it loses its original launch point. The code implements a highly specific workaround (`Step 2.5` in `_tick_land`) to force the home coordinate back to the original `true_home` using `MAV_CMD_DO_SET_HOME`.
- **Target Tracking Modes**: The FSM switches the vision pipeline between `qr` and `platform` modes dynamically.
- **Sprayer Command**: The script hijacks the `DO_SPRAYER` mission item as a signal to transition from `AUTO` to `GUIDED`. It primarily relies on parsing the `STATUSTEXT` from the autopilot rather than polling mission items, a clever latency optimization.

## Improvement Opportunities
- **MAVLink TCP latency**: The system uses a TCP bridge (`tcp:127.0.0.1:5764`) for SITL, which is known to introduce up to 71ms of latency due to Nagle's algorithm and OS buffering. Switching to UDP (`udp:0.0.0.0:14551`) in SITL tests could eliminate this delay.
- **Dynamic Tuning**: Expose the `AlignmentController` PID gains via dynamic reconfigure or a tuning API to allow adjustments without restarting the `main.py` script.
- **Payload Masking**: The `vision/payload_mask.py` file is an empty placeholder. Adding actual mask boundaries to prevent detecting QR codes painted on the payload itself could improve robustness.

## Developer Onboarding Guide
**To get started with development on `drone_pi`:**
1. **Understand the Threading Model**: Never put blocking code (like `time.sleep` or heavy image processing) inside `state_machine.py`. If you need a heavy task, create a worker thread like `VisionPipeline` and pass data via a thread-safe dictionary (`threading.Lock()`).
2. **MAVLink Abstraction**: Do not call `MAVLinkInterface` directly from the FSM for flight maneuvers. Use the `FlightControl` class in `flight_control.py` which provides safe wrappers (e.g., `send_velocity`).
3. **Configurations**: All tuning parameters live in `config/`. If you add a new feature, add its configurable constants to the appropriate YAML file instead of hardcoding.
4. **SITL Testing**: You can test the full pipeline using `python3 verify_sitl.py` which will spawn the Gazebo simulator, ArduPilot, and the companion script simultaneously.

## Final Architecture Summary
The `drone_pi` repository presents a robust, highly parallelized companion computer stack tailored for precision drone tasks. By decoupling the slow, non-deterministic computer vision tasks (QR/ArUco decoding) from the fast, critical flight control loops, the system achieves stable visual servoing and payload deployment. While there is technical debt regarding SITL workarounds and hardcoded assumptions (like camera mounting), the architecture is highly extensible and provides a solid foundation for advanced autonomous missions.
