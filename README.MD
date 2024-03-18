# RLtools: PX4 Module

This example aims at the `Pixracer Pro` which is a `Pixhawk 6C` compliant hardware implementation (have a look at `./px4_autopilot` for the changes to the mro/pixracerpro board definition and compilation config).

[//]: # (Add `CONFIG_MODULES_BACKPROP_TOOLS_BENCHMARK=y` in the boards `px4config.default` &#40;of whichever board you are using&#41;)
You might have to make space (flash) for the module e.g. by disabling unnecessary modules in the `px4config.default` file of your board (e.g. `./px4_autopilot/boards/mro/pixracerpro/default.px4board`):
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
cd px4_autopilot
make mro_pixracerpro_default EXTERNAL_MODULES_LOCATION=$(cd .. && cd external_modules && pwd) RL_TOOLS_ROOT=$(cd ../../../ && pwd) -j8
````
Flash with:
```
make mro_pixracerpro_default upload
```

For other boards you can substitute `mro_pixracerpro_default` with the appropriate board name (e.g. `px4_fmu-v6c_default`) and conduct changes in the corresponding `default.px4board` file.


For the SITL simulation you can use the following form:

```
make px4_sitl gz_x500 EXTERNAL_MODULES_LOCATION=/home/jonas/rl_tools/embedded_platforms/px4/external_modules RL_TOOLS_ROOT=/home/jonas/rl_tools
```


HITL
1. Enable the simulation quadrotor (x) in QGroundControl
2. Disable the automatic connection to the Pixhawk in QGroundControl
3. Compile and update firmware `make px4_fmu-v6c_default EXTERNAL_MODULES_LOCATION=/home/jonas/rl_tools/embedded_platforms/px4/external_modules RL_TOOLS_ROOT=/home/jonas/rl_tools upload`
4. Compile gazebo sitl/hitl `DONT_RUN=1 make px4_sitl_default gazebo-classic`
5. Source gazebo stuff `source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`
6. Start gazebo (classic) `gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world`
7. Start QGroundControl (connects over UDP to the forwarded MAVLINK)

Note: It is recommended to disable the landing detection (which might trigger in-flight when using the trained policy):
```
COM_DISARM_LAND=-1
COM_DISARM_PRFLT=-1
```
Note: It is necessary to disable the integral parts in the integrated controllers as they might accumulate in the background when the policy is active:
```
MC_***_I=0
```
To log high-frequency data required for sysid:
```
SDLOG_PROFILE=8
```
When switching to and from the policy all integral components of the default control stack should be disabled (otherwise they might saturate while the policy is in action and e.g. send maximum throttle when switching back)

Note to activate the policy using a dead man's switch the aux passthrough should be activated in QGC.


Usage:
- Go into the QGroundControl console
- Start module: `rl_tools_benchmark start`
- Watch printed output: `dmesg`
