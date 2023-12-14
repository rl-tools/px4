import asyncio
import websockets
import json
import time
import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import subprocess
import numpy as np
from matplotlib.animation import FuncAnimation
from sklearn.linear_model import LinearRegression

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

logfile_input = "sysid/logs/log_45_2023-12-13-21-07-46.ulg"
retval = subprocess.run(["ulog2csv", logfile_input, "-o", f"sysid/logs_csv/{os.path.split(logfile_input)[-1]}"])
assert(retval.returncode == 0)

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
        if "timestamp_sample" in df.columns:
            df.columns = [f"{identifier}_{col}" if col != 'timestamp_sample' else col for col in df.columns]
            df.rename(columns={f"timestamp_sample": "timestamp"}, inplace=True)
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
    if "timestamp_sample" in col:
        print(f"{col}: {frequency}")

import scipy.signal
b, a = scipy.signal.butter(4, 0.50)
time_start = 1
time_end = 10
merged_timeframe = merged_df[(merged_df["timestamp"] > time_start) & (merged_df["timestamp"] < time_end)].copy()
merged_timeframe.rename(columns={
    "vehicle_acceleration_xyz[0]": "acc_x",
    "vehicle_acceleration_xyz[1]": "acc_y",
    "vehicle_acceleration_xyz[2]": "acc_z",
}, inplace=True)
vehicle_acceleration = merged_timeframe[["timestamp", "acc_x", "acc_y", "acc_z"]].dropna()
merged_timeframe.rename(columns={
    "vehicle_attitude_q[0]": "q_0",
    "vehicle_attitude_q[1]": "q_1",
    "vehicle_attitude_q[2]": "q_2",
    "vehicle_attitude_q[3]": "q_3",
}, inplace=True)
vehicle_attitude = merged_timeframe[["timestamp", "q_0", "q_1", "q_2", "q_3"]].dropna()
merged_timeframe.rename(columns={
    "actuator_motors_control[0]": "m_0",
    "actuator_motors_control[1]": "m_1",
    "actuator_motors_control[2]": "m_2",
    "actuator_motors_control[3]": "m_3",
}, inplace=True)
actuator_controls_orig = merged_timeframe[["timestamp", "m_0", "m_1", "m_2", "m_3"]].dropna()
merged_timeframe.rename(columns={
    "actuator_motors_rl_tools_control[0]": "mrl_0",
    "actuator_motors_rl_tools_control[1]": "mrl_1",
    "actuator_motors_rl_tools_control[2]": "mrl_2",
    "actuator_motors_rl_tools_control[3]": "mrl_3",
}, inplace=True)
actuator_controls_policy = merged_timeframe[["timestamp", "mrl_0", "mrl_1", "mrl_2", "mrl_3"]].dropna()
merged_timeframe.rename(columns={
    "vehicle_angular_velocity_xyz[0]": "w_x",
    "vehicle_angular_velocity_xyz[1]": "w_y",
    "vehicle_angular_velocity_xyz[2]": "w_z",
}, inplace=True)
vehicle_angular_rate = merged_timeframe[["timestamp", "w_x", "w_y", "w_z"]].dropna()
# for axis in ["x", "y", "z"]:
#     vehicle_acceleration[axis] = scipy.signal.filtfilt(b, a, vehicle_acceleration[axis])
#     # vehicle_acceleration[axis] = vehicle_acceleration[axis] - vehicle_acceleration[axis].mean()
#     # vehicle_acceleration[axis] = vehicle_acceleration[axis] / vehicle_acceleration[axis].std()
# for motor in ["0", "1", "2", "3"]:
#     actuator_controls_orig[motor] = scipy.signal.filtfilt(b, a, actuator_controls_orig[motor])

plt.plot(vehicle_acceleration['timestamp'], vehicle_acceleration["acc_x"], label="acc_x")
plt.plot(vehicle_acceleration['timestamp'], vehicle_acceleration["acc_y"], label="acc_y")
plt.plot(vehicle_acceleration['timestamp'], vehicle_acceleration["acc_z"], label="acc_z")
plt.legend()
plt.show()
plt.plot(vehicle_angular_rate['timestamp'], vehicle_angular_rate["w_x"], label="w_x")
plt.plot(vehicle_angular_rate['timestamp'], vehicle_angular_rate["w_y"], label="w_y")
plt.plot(vehicle_angular_rate['timestamp'], vehicle_angular_rate["w_z"], label="w_z")
plt.legend()
plt.show()
plt.plot(actuator_controls_orig['timestamp'], actuator_controls_orig["m_0"], label="m_0")
plt.plot(actuator_controls_orig['timestamp'], actuator_controls_orig["m_1"], label="m_1")
plt.plot(actuator_controls_orig['timestamp'], actuator_controls_orig["m_2"], label="m_2")
plt.plot(actuator_controls_orig['timestamp'], actuator_controls_orig["m_3"], label="m_3")
plt.legend()
plt.show()
plt.plot(actuator_controls_policy['timestamp'], actuator_controls_policy["mrl_0"], label="m_0")
plt.plot(actuator_controls_policy['timestamp'], actuator_controls_policy["mrl_1"], label="m_1")
plt.plot(actuator_controls_policy['timestamp'], actuator_controls_policy["mrl_2"], label="m_2")
plt.plot(actuator_controls_policy['timestamp'], actuator_controls_policy["mrl_3"], label="m_3")
plt.legend()
plt.show()
plt.plot(vehicle_attitude['timestamp'], vehicle_attitude["q_0"], label="q_0")
plt.plot(vehicle_attitude['timestamp'], vehicle_attitude["q_1"], label="q_1")
plt.plot(vehicle_attitude['timestamp'], vehicle_attitude["q_2"], label="q_2")
plt.plot(vehicle_attitude['timestamp'], vehicle_attitude["q_3"], label="q_3")
plt.legend()
plt.show()
# plt.plot(vehicle_angular_rate['timestamp'], vehicle_angular_rate["x"], label="gyro_x")


actuator_controls_orig.describe()
vehicle_attitude.describe()
vehicle_acceleration.describe()
vehicle_angular_rate.describe()

thrust_acc = merged_timeframe[["timestamp", "m_0", "m_1", "m_2", "m_3", "acc_x", "acc_y", "acc_z"]].dropna()
thrust = thrust_acc["m_0"] ** 2 + thrust_acc["m_1"] ** 2 + thrust_acc["m_2"] ** 2 + thrust_acc["m_3"] ** 2
acceleration = np.sqrt(thrust_acc["acc_x"] ** 2 + thrust_acc["acc_y"] ** 2 + thrust_acc["acc_z"] ** 2)
model = LinearRegression()
model.fit(thrust.values.reshape(-1, 1), acceleration.values.reshape(-1, 1))
model.coef_[0][0]
model.intercept_[0]

def plot_thrust_acc(tau, ax):
    print(f"tau: {tau}")
    thrust_acc = merged_timeframe[["timestamp", "m_0", "m_1", "m_2", "m_3", "acc_x", "acc_y", "acc_z"]].dropna()
    thrust = thrust_acc["m_0"] ** 2 + thrust_acc["m_1"] ** 2 + thrust_acc["m_2"] ** 2 + thrust_acc["m_3"] ** 2
    thrust_acc = thrust_acc[(thrust > 0.15) & (thrust < 0.5)]
    thrust = thrust_acc["m_0"] ** 2 + thrust_acc["m_1"] ** 2 + thrust_acc["m_2"] ** 2 + thrust_acc["m_3"] ** 2
    acceleration = np.sqrt(thrust_acc["acc_x"] ** 2 + thrust_acc["acc_y"] ** 2 + thrust_acc["acc_z"] ** 2)

    thrust = thrust.ewm(halflife=f"{tau} s", times=pd.to_datetime(thrust_acc["timestamp"], unit="s")).mean()
    model = LinearRegression()

    model.fit(thrust.values.reshape(-1, 1), acceleration.values.reshape(-1, 1))

    correlation = thrust.corr(acceleration)
    if ax is not None:
        ax.scatter(thrust, acceleration)
        ax.set_title(f"Tau = {tau}")
        ax.set_xlabel("Thrust")
        ax.set_ylabel("Acceleration")
    return correlation, (model.coef_[0][0], model.intercept_[0])

tau_values = list(np.linspace(0.005, 0.04, 60))

correlations = [plot_thrust_acc(tau, None) for tau in tau_values]
best_tau_index = np.argmax([np.abs(x[0]) for x in correlations])
best_tau = tau_values[best_tau_index]
best_model = correlations[best_tau_index][1]
print(f"Best tau: {best_tau}")

fig, ax = plt.subplots()
plot_thrust_acc(best_tau, ax)
x = np.linspace(0, 1, 100)
ax.plot(x, best_model[0] * x + best_model[1], color="red")
fig.show()



fig, ax = plt.subplots()
def animate(i):
    ax.clear()
    plot_thrust_acc(tau_values[i], ax)

ani = FuncAnimation(fig, animate, frames=len(tau_values), interval=500)
ani.save('thrust_acc_animation.gif', writer='imagemagick', fps=1)

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
