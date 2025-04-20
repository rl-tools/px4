# RLtools: PX4 Module

> **⚠️ Note**: This was tested with RLtools 9607f5b948808b69588e4f53351eff9c0407d114
>

This example aims at the `Pixracer Pro` which is a `Pixhawk 6C` compliant hardware implementation (have a look at `./px4_autopilot` for the changes to the mro/pixracerpro board definition and compilation config).

## Installation
The commands provided here assume that this repo is cloned as a submodule of RLtools (you can also have RLtools separately but need to adjust the paths in that case).
If you haven't cloned RLtools already:
```
git clone https://github.com/rl-tools/rl-tools.git rl_tools
cd rl_tools
git submodule update --init --recursive embedded_platforms/px4
```

## Integration

### Integrating with a different board/vendor platform

This repo already integrates a modified PX4 that integrates the policy modules but if you want to integrate it in additional boards:

Add the following to the `default.px4board` (e.g. `./px4_autopilot/boards/mro/pixracerpro/default.px4board`)
CONFIG_MODULES_ACTUATOR_MOTORS_MULTIPLEXER=y
CONFIG_MODULES_RL_TOOLS_BENCHMARK=y
CONFIG_MODULES_RL_TOOLS_POLICY=y

You might have to make space (flash) for the module e.g. by disabling unnecessary modules in the `px4config.default` file of your board :

```
CONFIG_DRIVERS_CAMERA_CAPTURE=n
CONFIG_DRIVERS_CAMERA_TRIGGER=n
CONFIG_COMMON_OPTICAL_FLOW=n
CONFIG_DRIVERS_UAVCAN=n
CONFIG_MODULES_CAMERA_FEEDBACK=n
```



Main CMakeList:
- Move the loading of external modules to after then "include(px4_add_library)"
- Change to c++17 in the main

Build with (after changing the path to wherever this repository is located)::
```
docker run -it --rm -w /rl-tools/embedded_platforms/px4/px4_autopilot -v $(cd ../../ && pwd):/rl-tools rltools/px4 bash -c 'make distclean && make px4_fmu-v6c_default EXTERNAL_MODULES_LOCATION=$(cd .. && cd external_modules && pwd) RL_TOOLS_ROOT=$(cd ../../../ && pwd) -j8'
````
Flash with `make mro_pixracerpro_default upload` or using QGroundControl.

For other boards you can substitute `mro_pixracerpro_default` with the appropriate board name (e.g. `px4_fmu-v6c_default`) and conduct changes in the corresponding `default.px4board` file.

## Usage (Policy):
- The policy is triggered by the `AUX1` channel, so you should map it in the QGroundControl to a trigger on you remote control. It is best to use a switch that automatically goes back, so you can test the policy for a split second (the controller will switch back to the original controller).
- You can test the switching (without a battery connected) by looking at the `dmesg` output in the QGroundControl console after pulling the trigger
- Make sure that you have a good position reference (e.g. GPS) because the policy will try to hover in place (where the trigger was activated)
- It is highly recommended to test your policies in the HITL simulation at first
- Make sure that the dynamics (parameters) of the simulation match the real drone!
- Flying is on your own risk!

## Usage (Benchmark):
- Go into the QGroundControl console
- Start module: `rl_tools_benchmark start`
- Watch printed output: `dmesg`

## Configuration

Note: It is recommended to disable the landing detection (which might trigger in-flight when using the trained policy):
```
COM_DISARM_LAND=-1
COM_DISARM_PRFLT=-1
```
Note: It is necessary to disable the integral parts in the integrated controllers as they might accumulate in the background when the policy is active:
```
MC_***_I=0
```
To prevent the phase shift/delay in the gyro data:
```
IMU_GYRO_CUTOFF>=100Hz
```
To log high-frequency data required for sysid:
```
SDLOG_PROFILE=8
```
When switching to and from the policy all integral components of the default control stack should be disabled (otherwise they might saturate while the policy is in action and e.g. send maximum throttle when switching back)

Make sure that the PX4 `IMU_GYRO_RATEMAX` setting is matching the rate that is [expected by the policy](https://github.com/rl-tools/px4/blob/fe4c79b987b3d6928b26240096edbbe2cd608e69/external_modules/src/modules/rl_tools_policy/RLtoolsPolicy.hpp#L118) (`400 Hz` by default)

Note to activate the policy using a dead man's switch the aux passthrough should be activated in QGC.


## Hardware in the Loop (HITL)
1. Enable the simulation quadrotor (x) in QGroundControl
2. Disable the automatic connection to the Pixhawk in QGroundControl
3. Compile and update firmware `make px4_fmu-v6c_default EXTERNAL_MODULES_LOCATION=$(cd .. && cd external_modules && pwd) RL_TOOLS_ROOT=$(cd ../../../ && pwd)`
4. Compile gazebo sitl/hitl `DONT_RUN=1 make px4_sitl_default gazebo-classic`
5. Source gazebo stuff `source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`
6. Start gazebo (classic) `gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world`
7. Start QGroundControl (connects over UDP to the forwarded MAVLINK)



## Software in the Loop (SITL)
```
docker build . -t rltools/px4
docker network create rl-tools-px4
```
`distclean` for good measure:
```
docker run -it --rm -w /rl-tools/embedded_platforms/px4/px4_autopilot -v $(cd ../../ && pwd):/rl-tools rltools/px4 bash -c 'make distclean'
```
build SITL (without running)
```
docker run -it --rm -w /rl-tools/embedded_platforms/px4/px4_autopilot -v $(cd ../../ && pwd):/rl-tools rltools/px4 bash -c 'DONT_RUN=1 make px4_sitl EXTERNAL_MODULES_LOCATION=$(cd .. && cd external_modules && pwd) RL_TOOLS_ROOT=$(cd ../../../ && pwd) -j8'
```
Run SITL (headless):
```
docker run -it --rm --name rl-tools-px4 --network host --hostname rl-tools-px4 -w /rl-tools/embedded_platforms/px4/px4_autopilot -v $(cd ../../ && pwd):/rl-tools rltools/px4 bash -c 'PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 HEADLESS=1 ./build/px4_sitl_default/bin/px4'
```
Connect GUI client:
```
xhost +local:docker && docker run --rm --network host -it --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --mount type=bind,source=$(pwd)/px4_autopilot/Tools/simulation/gz/models,target=/models,readonly rltools/px4 bash -c 'GZ_PARTITION=rl-tools-px4:root GZ_SIM_RESOURCE_PATH=/models gz sim'
```
note: Gazebo has some auto-discovery and global partition namespace, which is set based on the `--hostname` of the server

Run QGroundControl in UDP discovery mode


### Using a Joystick
The Joystick is forwarded from QGroundControl through Mavlink to the SITL. 
1. Do the Joystick setup in QGroundControl and map one button to arm the drone.
2. Another button should be used for switching to the RLtools policy, remember the ID (from the Joystick setup view) of the desired button
3. In the parameters set `RLT_ACTIV_SRC` to Mavlink and `RLT_ACTIV_BTN` to the button id identified before

### Notes on the simulation frequency / IMU rate

The low-level controllers are synchronized on the IMU rate (`IMU_INTEG_RATE`) but they are limited by the Gazebo simulation rate. Policies that take the history into account rely on the time between steps to match the one experienced during training (which is often 100 Hz). We can control at higher frequencies, but we need to make sure to advance e.g. the action history at a lower rate. The difference should be an integer multiple such that there is no aliasing. By default the Gazebo sim rate is 250Hz wich is not an integer multiple. To set the rate to e.g. 400 Hz we need to change it in the `physics` tag and the `imu_sensor` tag (works best if both exactly match):

```diff
diff --git a/Tools/simulation/gz/models/x500/model.sdf b/Tools/simulation/gz/models/x500/model.sdf
index b544e01d49..f158b1c28c 100644
--- a/Tools/simulation/gz/models/x500/model.sdf
+++ b/Tools/simulation/gz/models/x500/model.sdf
@@ -229,7 +229,7 @@
       </sensor>
       <sensor name="imu_sensor" type="imu">
         <always_on>1</always_on>
-        <update_rate>250</update_rate>
+        <update_rate>400</update_rate>
       </sensor>
     </link>
     <link name="rotor_0">
diff --git a/Tools/simulation/gz/worlds/default.sdf b/Tools/simulation/gz/worlds/default.sdf
index 4b6f384002..b4a7c4770a 100644
--- a/Tools/simulation/gz/worlds/default.sdf
+++ b/Tools/simulation/gz/worlds/default.sdf
@@ -1,9 +1,8 @@
 <sdf version='1.9'>
   <world name='default'>
-    <physics type="ode">
-      <max_step_size>0.004</max_step_size>
+    <physics name="1ms" type="ode">
+      <max_step_size>0.0025</max_step_size>
       <real_time_factor>1.0</real_time_factor>
-      <real_time_update_rate>250</real_time_update_rate>
     </physics>
     <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
     <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
```

## Simulation in Hardware (SIH)
HITL but the dynamics simulation is running on the flight controller
- cd `px4_autopilot`
- Build and upload (mind to check the device path) `make px4_fmu-v6c_default EXTERNAL_MODULES_LOCATION=$(cd .. && cd external_modules && pwd) RL_TOOLS_ROOT=$(cd ../../../ && pwd) -j8 upload SERIAL_PORT=/dev/ttyACM0`
  - If you run Ubuntu 24.04 (not supported by the PX4 toolchain): Build in docker 
    - `docker build . -t rltools/px4`
    - `docker run -it --rm -w /rl-tools/embedded_platforms/px4/px4_autopilot -v $(cd ../../ && pwd):/rl-tools rltools/px4 bash -c 'make px4_fmu-v6c_default EXTERNAL_MODULES_LOCATION=$(cd .. && cd external_modules && pwd) RL_TOOLS_ROOT=$(cd ../../../ && pwd) -j8'`
    - Upload the `.px4` using QGroundControl
- Run mavproxy (because when using JMavSim as the relay the Mavlink console did not work): `mavproxy.py --master=/dev/ttyACM0 --baud=921600 --out=udp:127.0.0.1:4560 --out=udp:127.0.0.1:14550 || mavproxy.py --master=/dev/ttyACM1 --baud=921600 --out=udp:127.0.0.1:4560 --out=udp:127.0.0.1:14550`

- Run JMavSim in simulation only mode (default UDP port: `4560`): `./Tools/simulation/jmavsim/jmavsim_run.sh -q -u -o`

- QGroundControl (set autoconnect to UDP only)

- Set the `SIH_*` dynamics parameters to match the model the policy was trained for (for the Crazyflie e.g. `external_modules/src/modules/rl_tools_policy/crazyflie_sih.params`)

## Notes on v1.15 and Ubuntu 24.04
Note that the Pixracer pro does not properly support 1.15 due to the [compass not found issue](https://github.com/PX4/PX4-Autopilot/issues/23552). Also Ubuntu 24.04 is not well supported by the build system (some compiler mismatch) and SITL. If you still want to try your luck:
1. Ubuntu 24.04: Check out `main` to use the new `./Tools/setup/ubuntu.sh` script.
2. `git checkout v1.15.2`
3. `make px4_fmu-v6c_default` to test if the compilation works
4. `make px4_fmu-v6c_default EXTERNAL_MODULES_LOCATION=$(cd .. && cd external_modules && pwd) RL_TOOLS_ROOT=$(cd ../../../ && pwd) -j8`



## More config
Flow
Run mocap docker container to connect to Vicon and forward the ROS topics using the WebSocket bridge

Run Mavproxy to proxy between QGroundControl and the Vicon forwarding script:
```
mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 || mavproxy.py --master=/dev/ttyACM1 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 || mavproxy.py --master=/dev/ttyUSB0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 || mavproxy.py --master=/dev/ttyUSB1 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

Run the Vicon forwarding script (check the `.vscode/launch.json`) to connect to the right Vicon topic and the localhost:14551 UDP MavLink endpoint.

Turn off the Barometer height estimation when using a mocap system:
```
EKF2_HGT_REF = "Vision"
```
Because we send the position reference through the MAVLINK vision message
```
EKF2_BARO_CTRL = Disabled
```
```
EKF2_MAG_TYPE = None
```