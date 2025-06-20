import asyncio
from twisted.internet import asyncioreactor
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
asyncioreactor.install(asyncio.get_event_loop())
from twisted.internet import reactor

import roslibpy
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import threading
import asyncio


from pymavlink import mavutil
import time



last_update = None
interval = 0.10

POSITION_STD = 0.01 # m
VELOCITY_STD = 0.10 # m/s
ORIENTATION_STD = 5.0 # degrees
def vicon_callback(msg):
    global last_update
    secs  = msg["header"]["stamp"]["secs"]
    nsecs = msg["header"]["stamp"]["nsecs"]
    usec  = int(secs * 1e6 + nsecs / 1e3)

    x,  y,  z  =  [msg["pose"]["pose"]["position"][x] for x in ["x", "y", "z"]]
    x,  y,  z  =  x, -y, -z
    qw, qx, qy, qz = [msg["pose"]["pose"]["orientation"][x] for x in ["w", "x", "y", "z"]]
    q_mav = [qw,  qx, -qy, -qz]

    vx, vy, vz = [msg["twist"]["twist"]["linear"][x] for x in ["x", "y", "z"]]
    vx, vy, vz =  vx, -vy, -vz
    Rwb = R.from_quat([q_mav[-1], q_mav[0], q_mav[1], q_mav[2]])
    vx_body, vy_body, vz_body = Rwb.apply([vx, vy, vz], inverse=True)

    pose_cov = np.full(21, np.nan,  dtype=np.float32)
    vel_cov  = np.full(21, np.nan,  dtype=np.float32)
    pose_cov[0] = pose_cov[6] = pose_cov[11] = POSITION_STD**2
    pose_cov[15] = pose_cov[18] = pose_cov[20] = (np.deg2rad(ORIENTATION_STD))**2
    vel_cov[0]  = vel_cov[6]  = vel_cov[11]  = VELOCITY_STD**2
    now = time.time()
    if last_update is None or interval is None or now - last_update > interval:
        connection.mav.odometry_send(
            usec,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # pose frame
            mavutil.mavlink.MAV_FRAME_BODY_FRD,     # twist frame
            x, y, z,
            q_mav,
            vx_body, vy_body, vz_body,
            float('nan'), float('nan'), float('nan'),   # angular rates
            pose_cov,
            vel_cov,
            0,                                         # reset_counter
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
            100 # quality (100% confidence, 0-100)
        )
        print(f"Forwarding position (FRD) to MAVLink: {x:.2f}, {y:.2f}, {z:.2f}, q: {qw:.2f}, {qx:.2f}, {qy:.2f}, {qz:.2f}")
        last_update = now

async def main():
    while True:
        position = [
            0,
            0,
            -0.07 - 0.30
        ]
        connection.mav.set_position_target_local_ned_send(
            0,
            1, 
            1,
            # connection.target_system,
            # connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b1111111111000000,
            *position,  # x, y, z
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0, 0,  # yaw, yaw_rate
        )
        await asyncio.sleep(0.1)

if __name__ == "__main__":
    mavlink_url = os.environ["MAVLINK_URL"] if "MAVLINK_URL" in os.environ else "tcp:localhost:5760"
    connection = mavutil.mavlink_connection(mavlink_url)

    connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

    ros = roslibpy.Ros(host='localhost', port=9090)
    vicon_pose_topic = os.environ["MAVLINK_POSE_TOPIC"] if "MAVLINK_POSE_TOPIC" in os.environ else None
    if vicon_pose_topic is not None:
        print(f"Subscribing to {vicon_pose_topic}")
        vicon_listener = roslibpy.Topic(ros, vicon_pose_topic, 'nav_msgs/Odometry')
        vicon_listener.subscribe(vicon_callback)
    loop.create_task(main())
    reactor.run()


