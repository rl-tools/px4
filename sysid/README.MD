```
FILE= ; mkdir logs_csv/$FILE; ulog2csv logs/$FILE -o logs_csv/$FILE
```


High-frequency logging, in the mavlink console:
```
logger stop
logger start -r 2000 -b 100
```

Tipps for collecting data:

Do three separate runs (arming-flying-landing-disarming) so that the log-files are separate
1. Up and down straight acceleration (Tip: Go high and let the Drone drop at a fast speed, then "catch" it again, this produces high accelerations without risking to slam into the ceiling, if there is no ceiling, accelerate straight up)
2. Left right movements. Use the stabilized or manual mode so that the angular accelerations are more agile. Tip (Horizontal) velocity does not matter so you can just wiggle left and right and forward and backward in the same position. Try to give step-inputs if safely possible.
3. Yaw angular acceleration. Yaw left and right (only acceleration is important so wiggle in place as before). Altitude mode is recommended because maintaining height and controlling yaw can be challenging.