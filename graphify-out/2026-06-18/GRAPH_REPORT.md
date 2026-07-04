# Graph Report - .  (2026-06-18)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 147 nodes · 180 edges · 23 communities (17 shown, 6 thin omitted)
- Extraction: 94% EXTRACTED · 6% INFERRED · 0% AMBIGUOUS · INFERRED: 11 edges (avg confidence: 0.64)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]
- [[_COMMUNITY_Community 4|Community 4]]
- [[_COMMUNITY_Community 5|Community 5]]
- [[_COMMUNITY_Community 6|Community 6]]
- [[_COMMUNITY_Community 7|Community 7]]
- [[_COMMUNITY_Community 8|Community 8]]
- [[_COMMUNITY_Community 9|Community 9]]
- [[_COMMUNITY_Community 10|Community 10]]
- [[_COMMUNITY_Community 11|Community 11]]
- [[_COMMUNITY_Community 12|Community 12]]
- [[_COMMUNITY_Community 13|Community 13]]
- [[_COMMUNITY_Community 14|Community 14]]
- [[_COMMUNITY_Community 18|Community 18]]

## God Nodes (most connected - your core abstractions)
1. `StateMachine` - 17 edges
2. `MAVLinkInterface` - 15 edges
3. `FlightControl` - 14 edges
4. `DroneSystem` - 14 edges
5. `CameraManager` - 12 edges
6. `AlignmentController` - 8 edges
7. `QRDecoder` - 7 edges
8. `QRDetector` - 7 edges
9. `main()` - 5 edges
10. `AlignmentController` - 5 edges

## Surprising Connections (you probably didn't know these)
- `DroneSystem` --uses--> `FlightControl`  [INFERRED]
  main.py → core/flight_control.py
- `DroneSystem` --uses--> `MAVLinkInterface`  [INFERRED]
  main.py → core/mavlink_interface.py
- `DroneSystem` --uses--> `StateMachine`  [INFERRED]
  main.py → core/state_machine.py
- `DroneSystem` --uses--> `AlignmentController`  [INFERRED]
  main.py → vision/alignment_controller.py
- `DroneSystem` --uses--> `CameraManager`  [INFERRED]
  main.py → vision/camera_manager.py

## Import Cycles
- None detected.

## Communities (23 total, 6 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.12
Nodes (13): DroneSystem, main(), Handle shutdown signals., Main drone system coordinator., Start all subsystems., signal_handler(), QRDecoder, QR code decoder with retry logic. (+5 more)

### Community 1 - "Community 1"
Cohesion: 0.09
Nodes (12): MAVLinkInterface, MAVLink interface with auto-reconnect and USB safety., Increment tick counter for heartbeat monitoring., Discover Pixhawk port in priority order., Thread-safe MAVLink connection with auto-reconnect., Main connection thread loop., Start the connection thread., Stop the connection thread. (+4 more)

### Community 2 - "Community 2"
Cohesion: 0.18
Nodes (9): Hold position briefly., Align with QR code center., Execute return to launch., Main mission state machine., Execute one FSM tick (non-blocking)., Transition to new state., Wait for MAVLink connection., Monitor for AUTO mode completion. (+1 more)

### Community 3 - "Community 3"
Cohesion: 0.11
Nodes (9): FlightControl, Flight control abstraction layer., High-level flight control interface., Check if vehicle is in AUTO mode., Get distance to current waypoint (stub for safety)., Hold current position (zero velocity)., Send velocity command (NED body frame)., Command return to launch. (+1 more)

### Community 4 - "Community 4"
Cohesion: 0.15
Nodes (8): CameraManager, Camera interface using rpicam-vid., Manage rpicam-vid subprocess and frame buffer., Start camera capture., Get most recent frame (non-blocking)., Start rpicam-vid subprocess., Stop rpicam-vid subprocess., Main capture loop with auto-restart.

### Community 5 - "Community 5"
Cohesion: 0.22
Nodes (5): AlignmentController, Visual servoing controller for QR alignment., Reset controller state., Compute velocity command to center QR code.                  Args:             c, Convert pixel error to velocity commands.

### Community 6 - "Community 6"
Cohesion: 0.22
Nodes (5): AlignmentController, Visual servoing controller for QR alignment., Reset controller state., Compute velocity command to center QR code.                  Args:             c, Convert pixel error to velocity commands.

### Community 7 - "Community 7"
Cohesion: 0.67
Nodes (4): QR Decode Configuration, Vision Configuration, QR Code Image, opencv-python

### Community 8 - "Community 8"
Cohesion: 0.67
Nodes (3): Finite state machine for autonomous mission., State, Enum

### Community 9 - "Community 9"
Cohesion: 0.67
Nodes (3): MAVLink Configuration, pymavlink, pyserial

## Knowledge Gaps
- **7 isolated node(s):** `System Configuration`, `Camera Configuration`, `Alignment Configuration`, `Flight Configuration`, `pymavlink` (+2 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **6 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MAVLinkInterface` connect `Community 1` to `Community 0`?**
  _High betweenness centrality (0.199) - this node is a cross-community bridge._
- **Why does `StateMachine` connect `Community 2` to `Community 8`, `Community 0`?**
  _High betweenness centrality (0.189) - this node is a cross-community bridge._
- **Why does `FlightControl` connect `Community 3` to `Community 0`?**
  _High betweenness centrality (0.163) - this node is a cross-community bridge._
- **Are the 7 inferred relationships involving `DroneSystem` (e.g. with `FlightControl` and `MAVLinkInterface`) actually correct?**
  _`DroneSystem` has 7 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Core flight control and MAVLink communication modules.`, `Flight control abstraction layer.`, `High-level flight control interface.` to the rest of the system?**
  _59 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.11666666666666667 - nodes in this community are weakly interconnected._
- **Should `Community 1` be split into smaller, more focused modules?**
  _Cohesion score 0.09090909090909091 - nodes in this community are weakly interconnected._