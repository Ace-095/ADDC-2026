The actual root cause is a bug inside the ArduPilot `SIM_JSON` source code in your repository!

When using `--model JSON`, ArduPilot sends motor commands to Gazebo on port 9002. However, the ArduPilot code was missing the crucial `bind()` call to listen for Gazebo's sensor replies on port 9003. Because of this, ArduPilot was sending data but ignoring all incoming sensor data, causing the lockup.

I have directly patched `ardupilot/libraries/SITL/SIM_JSON.cpp` to correctly bind to the `port_in` (9003).

Since you already have the ArduPilot build environment set up, you just need to recompile the `arducopter` binary for SITL. Please run the following in your first terminal:

```bash
cd ~/DUMP/AEROCLUB/"ADDC 2026"/CODEBASE/RESOURCES/drone_pi/ardupilot
source ~/venv-ardupilot/bin/activate
./waf copter
```

Once it finishes building, restart your `arducopter`, `mavproxy`, and `gz sim` terminals. It should now successfully establish two-way communication!
