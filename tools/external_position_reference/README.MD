# USB
```
mavlink-routerd  /dev/ttyACM0:57600
```
# Telemetry
```
mavlink-routerd  /dev/ttyUSB0:57600
```
```
. ~/arpl_workspace/devel/setup.bash
roslaunch mocap_vicon vicon.launch
```
The modified vicon.launch includes the ros websocket bridge

```
python3 forward_ros_to_mavlink.py
```