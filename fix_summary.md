I identified the root cause of the issue in the SITL simulation.

### The Problem
ArduCopter was stuck printing `No JSON sensor message received, resending servos` because it was not receiving simulated sensor data from Gazebo. Because ArduCopter was hanging in its initialization loop, it never sent `HEARTBEAT` messages. This caused the `drone_pi` companion script to correctly report `[HB-STALE] is_guided_mode: autopilot HB age=infs`.

The underlying issue was a port configuration mismatch in the Gazebo `iris_with_gimbal` SDF model. 
- The `ArduPilotPlugin` was set with `<fdm_port_in>9002</fdm_port_in>` but no `fdm_port_out`.
- The ArduCopter SITL JSON backend by default binds to `9002` to *receive* data, and `9003` to *send* data.
- The `ArduPilotPlugin` in Gazebo expects `fdm_port_out` as the port to *send* data to ArduPilot, and `fdm_port_in` to *receive* motor commands from ArduPilot.

Because both were trying to use `9002` in conflicting ways, they could not communicate.

### The Fix
I updated `/home/ace/gz_ws/src/ardupilot_gazebo/models/iris_with_gimbal/model.sdf` to correctly map the ports:
```xml
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9003</fdm_port_in>
      <fdm_port_out>9002</fdm_port_out>
```

You can now restart your ArduCopter SITL instance and Gazebo. The simulation should run normally and the `drone_pi` companion computer will receive valid telemetry.
