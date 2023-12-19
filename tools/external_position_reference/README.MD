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


Set `EKF2_HGT_REF` to `Vision`
Set `EKF2_MAG_TYPE` to `None`
Set `EKF2_EV_CTRL` to `11`
