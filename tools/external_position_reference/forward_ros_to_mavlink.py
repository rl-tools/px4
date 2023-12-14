import roslibpy
import os
import numpy as np
from scipy.spatial.transform import Rotation as R


from pymavlink import mavutil
import time

mavlink_url = os.environ["MAVLINK_URL"] if "MAVLINK_URL" in os.environ else "tcp:localhost:5760"
connection = mavutil.mavlink_connection(mavlink_url)

connection.wait_heartbeat()
connection.mav.play_tune_send(1, 1, b"a")

def vicon_callback(message):
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
    connection.mav.vision_position_estimate_send(usec, x, y, z, roll, pitch, yaw)
    print(f"Forwarding position (FRD) to MAVLink: {x}, {y}, {z}, {roll}, {pitch}, {yaw}")
ros = roslibpy.Ros(host='localhost', port=9090)
vicon_pose_topic = os.environ["MAVLINK_POSE_TOPIC"] if "MAVLINK_POSE_TOPIC" in os.environ else "/vicon/race6/pose"
vicon_listener = roslibpy.Topic(ros, vicon_pose_topic, 'geometry_msgs/PoseStamped')
vicon_counter = 0
vicon_listener.subscribe(vicon_callback)
ros.run_forever()

