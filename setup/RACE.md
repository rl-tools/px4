# PX4 From Scratch
1. Reset all parameters
2. Start from the Generic 250 Racer frame setup
3. Assign motors (make sure to set Dshot600 instead of PWM)
4. To receive `SET_POSITION_TARGET_LOCAL_NED` MavLink setpoints on the `trajectory_setpoint` topic set `MAV_0_MODE` to Onboard. Note that you still need to switch the flightmode to Offboard for the setpoints to be forwarded

# ELRS
Use SBUS output (`RC_INPUT_PROTO`). CRSF leads to jitter on the aux channel which causes unwanted bursts of policy activation. Configure from the ELRS Lua script in the Transmitter. Note that telemetry has to be enabled for the configuration menu (Other devices) to show up

# Wifi Telemetry
1. Using the Waveshare ESP32-C3-Zero. Flash DroneBridge using the web flasher
2. power cycle
3. Set client mode, ssid and password of the router used for experiments
4. Set TX = Pin 0, RX = Pin 1 (and solder the connector accordingly)
5. In the experiment router, give a static IP to the MAC of the Wifi adapter
6. Forwarding: `mavproxy.py --master=tcp:192.168.8.4:5760 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551` where `192.168.8.4` is the static IP set in the router. `14550` should be automagically detected by QGroundControl and `14551` can, e.g., be used by the `./tools/external_position_reference/forward_ros_to_mavlink.py` script to forward a mocap reference to the vehicle
7. Monitor the actual position received in EKF2: ```(while true; do sleep 2; echo "listener vehicle_local_position"; done) | python3 px4_autopilot/Tools/mavlink_shell.py tcp:192.168.8.4:5760 | grep -A2 '\sx\:'```