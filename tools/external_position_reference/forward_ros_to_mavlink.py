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

def vicon_callback(message):
    global last_update
    x = message["pose"]["position"]["x"]
    y = -message["pose"]["position"]["y"]
    z = -message["pose"]["position"]["z"]
    qw = message["pose"]["orientation"]["w"]
    qx = message["pose"]["orientation"]["x"]
    qy = -message["pose"]["orientation"]["y"]
    qz = -message["pose"]["orientation"]["z"]
    quaternion = np.array([qx, qy, qz, qw])
    rotation = R.from_quat(quaternion)
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)
    usec = int(message["header"]["stamp"]["secs"] * 1e6 + message["header"]["stamp"]["nsecs"] / 1e3)
    now = time.time()
    if last_update is None or interval is None or now - last_update > interval:
        connection.mav.vision_position_estimate_send(usec, x, y, z, roll, pitch, yaw)
        print(f"Forwarding position (FRD) to MAVLink: {x}, {y}, {z}, {roll}, {pitch}, {yaw}")
        last_update = now

async def main():
    while True:
        print("hi")
        await asyncio.sleep(1)

if __name__ == "__main__":
    mavlink_url = os.environ["MAVLINK_URL"] if "MAVLINK_URL" in os.environ else "tcp:localhost:5760"
    connection = mavutil.mavlink_connection(mavlink_url)

    connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

    ros = roslibpy.Ros(host='localhost', port=9090)
    vicon_pose_topic = os.environ["MAVLINK_POSE_TOPIC"] if "MAVLINK_POSE_TOPIC" in os.environ else "/vicon/race6/pose"
    print(f"Subscribing to {vicon_pose_topic}")
    vicon_listener = roslibpy.Topic(ros, vicon_pose_topic, 'geometry_msgs/PoseStamped')
    vicon_listener.subscribe(vicon_callback)
    loop.create_task(main())
    reactor.run()


