# PX4 From Scratch
1. Reset all parameters
2. Start from the Generic 250 Racer frame setup
3. Assign motors (make sure to set Dshot600 instead of PWM)

# ELRS
Use SBUS output (`RC_INPUT_PROTO`). CRSF leads to jitter on the aux channel which causes unwanted bursts of policy activation. Configure from the ELRS Lua script in the Transmitter. Note that telemetry has to be enabled for the configuration menu (Other devices) to show up

# Wifi Telemetry
1. Using the Waveshare ESP32-C3-Zero. Flash DroneBridge using the web flasher
2. power cycle
3. Set client mode, ssid and password of the router used for experiments
4. Set TX = Pin 0, RX = Pin 1 (and solder the connector accordingly)
5. In the experiment router, give a static IP to the MAC of the Wifi adapter