import asyncio
import websockets
import json
import time
import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import subprocess

def state_message(position, orientation):
    example_state = {
        "pose": {
            "position": list(position),
            "orientation": list(orientation)
        },
        "rotor_states": [
            {"rpm": 0},
            {"rpm": 0},
            {"rpm": 0},
            {"rpm": 0}
        ]
    }

    set_state_message = {
        "channel": "setDroneState",
        "data": {
            "id": "default",
            "data": example_state
        }
    }
    return set_state_message


with open("sysid/model.json", "r") as f:
    model = json.load(f)

# logfile = "sysid/logs_csv/log_149_2023-11-30-14-11-24.ulg"
# logfile = "sysid/logs_csv/log_147_2023-11-30-14-02-32.ulg"
# logfile = "sysid/logs_csv/log_161_2023-12-12-21-48-20.ulg"
# logfile = "sysid/logs_csv/log_162_2023-12-12-22-02-56.ulg"
logfile_input = "sysid/logs/log_164_UnknownDate.ulg"
subprocess.run(["ulog2csv", logfile_input, "-o", f"sysid/logs_csv/{os.path.split(logfile_input)[-1]}"])
logfile = os.path.join("sysid", "logs_csv", os.path.split(logfile_input)[-1])


import pandas as pd
import os
import re

dfs = {}

for file in os.listdir(logfile):
    if file.endswith('.csv'):
        file_path = os.path.join(logfile, file)

        identifier = re.search(r'log_\d+_\d+-\d+-\d+-\d+-\d+-\d+_(.+?)_\d+\.csv', file)
        if identifier:
            identifier = identifier.group(1)

        df = pd.read_csv(file_path)
        df.columns = [f"{identifier}_{col}" if col != 'timestamp' else col for col in df.columns]
        dfs[identifier] = df

merged_df = pd.DataFrame()
for df in dfs.values():
    if merged_df.empty:
        merged_df = df
    else:
        merged_df = pd.merge(merged_df, df, on='timestamp', how='outer')
merged_df = merged_df.sort_values(by='timestamp')
merged_df = merged_df.reset_index(drop=True)
merged_df["timestamp"] = (merged_df["timestamp"] - merged_df.loc[0]["timestamp"]) / 1e6

col_frequencies = [merged_df[col].isna().mean() for col in merged_df.columns]
for col, frequency in sorted(list(zip(merged_df.columns, col_frequencies)), key=lambda x: x[1]):
    if "motor" in col:
        print(f"{col}: {frequency}")

thrust_setpoint = merged_df[["timestamp", "vehicle_rates_setpoint_thrust_body[2]"]].dropna()
actuator_motors = merged_df[["timestamp", "actuator_motors_control[0]"]].dropna()
dt = vehicle_acceleration["timestamp"].diff()
plt.plot(vehicle_acceleration['timestamp'], dt)
plt.plot(vehicle_acceleration['timestamp'], vehicle_acceleration["sensor_combined_accelerometer_m_s2[2]"])
plt.plot(thrust_setpoint['timestamp'], thrust_setpoint["vehicle_rates_setpoint_thrust_body[2]"])
plt.plot(actuator_motors['timestamp'], actuator_motors["actuator_motors_control[0]"])
plt.show()

# low-pass filter the acceleration
import scipy.signal
b, a = scipy.signal.butter(4, 0.50)
time_start = 0
time_end = 100000
merged_timeframe = merged_df[(merged_df["timestamp"] > time_start) & (merged_df["timestamp"] < time_end)]
vehicle_acceleration = merged_timeframe.rename(columns={
    "sensor_combined_accelerometer_m_s2[0]": "x",
    "sensor_combined_accelerometer_m_s2[1]": "y",
    "sensor_combined_accelerometer_m_s2[2]": "z"
})[["timestamp", "x", "y", "z"]].dropna()
actuator_controls = merged_timeframe.rename(columns={
    "actuator_motors_control[0]": "0",
    "actuator_motors_control[1]": "1",
    "actuator_motors_control[2]": "2",
    "actuator_motors_control[3]": "3",
})[["timestamp", "0", "1", "2", "3"]].dropna()
vehicle_angular_rate = merged_timeframe.rename(columns={
    "vehicle_angular_velocity_xyz[0]": "x",
    "vehicle_angular_velocity_xyz[1]": "y",
    "vehicle_angular_velocity_xyz[2]": "z",
})[["timestamp", "x", "y", "z"]].dropna()
for axis in ["x", "y", "z"]:
    vehicle_acceleration[axis] = scipy.signal.filtfilt(b, a, vehicle_acceleration[axis])
    # vehicle_acceleration[axis] = vehicle_acceleration[axis] - vehicle_acceleration[axis].mean()
    # vehicle_acceleration[axis] = vehicle_acceleration[axis] / vehicle_acceleration[axis].std()
for motor in ["0", "1", "2", "3"]:
    actuator_controls[motor] = scipy.signal.filtfilt(b, a, actuator_controls[motor])

plt.plot(vehicle_acceleration['timestamp'], vehicle_acceleration["z"], label="acc_z")
plt.plot(actuator_controls['timestamp'], actuator_controls["0"], label="m_0")
plt.plot(actuator_controls['timestamp'], actuator_controls["1"], label="m_1")
plt.plot(actuator_controls['timestamp'], actuator_controls["2"], label="m_2")
plt.plot(actuator_controls['timestamp'], actuator_controls["3"], label="m_3")
# plt.plot(vehicle_angular_rate['timestamp'], vehicle_angular_rate["x"], label="gyro_x")
plt.legend()
plt.show()





exit()

async def connect_to_websocket():
    uri = "ws://localhost:8080"
    async with websockets.connect(uri) as websocket:

        add_drone_message = {
            "channel": "addDrone",
            "data": model
        }

        await websocket.send(json.dumps(add_drone_message))
        for t_ms in range(0, 10000, 10):
            t = t_ms / 1000
            position = [(t % 10)/10, 0, 0]
            orientation = [
                [1.0, 0.0, 0.0],
                [0.0, 0.984807753012208, -0.17364817766693033],
                [0.0, 0.17364817766693033, 0.984807753012208]
            ]
            await websocket.send(json.dumps(state_message(position, orientation)))
            time.sleep(0.01)

asyncio.get_event_loop().run_until_complete(connect_to_websocket())
