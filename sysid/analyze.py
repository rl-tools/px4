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
from sklearn.linear_model import LinearRegression, HuberRegressor
import pandas as pd
import os
import re
import scipy


mass = 1
rotor_x_displacement = 0.0775
rotor_y_displacement = 0.0981
rotor_positions = np.array([
    [ rotor_x_displacement, -rotor_y_displacement, 0],
    [-rotor_x_displacement,  rotor_y_displacement, 0],
    [ rotor_x_displacement,  rotor_y_displacement, 0],
    [-rotor_x_displacement, -rotor_y_displacement, 0]
])

def load_file(logfile_input, plot=False):
    timestamp = time.time()
    csv_path = f"sysid/logs_csv/{os.path.split(logfile_input)[-1]}_{timestamp}"
    retval = subprocess.run(["ulog2csv", logfile_input, "-o", csv_path])
    assert(retval.returncode == 0)

    logfile = csv_path
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
    merged_df["timestamp"] = pd.to_datetime(merged_df["timestamp"], unit="s")
    merged_df.set_index('timestamp', inplace=True)

    if plot:
        plt.plot(merged_df.index.to_series().diff().dropna().apply(lambda x: x.total_seconds())[1:])
        plt.show()
    return merged_df

def find_col(colname, merged_df):
    col_frequencies = [merged_df[col].isna().mean() for col in merged_df.columns]
    for col, frequency in sorted(list(zip(merged_df.columns, col_frequencies)), key=lambda x: x[1]):
        if colname in col:
            print(f"{col}: {frequency}")


def get_timeframe(merged_df, time_start, time_end, plot=False):
    merged_timeframe = merged_df[(merged_df.index > pd.to_datetime(time_start, unit="s")) & (merged_df.index < pd.to_datetime(time_end, unit="s"))].copy()
    merged_timeframe.rename(columns={
        "vehicle_acceleration_xyz[0]": "acc_x",
        "vehicle_acceleration_xyz[1]": "acc_y",
        "vehicle_acceleration_xyz[2]": "acc_z",
    }, inplace=True)
    vehicle_acceleration = merged_timeframe[["acc_x", "acc_y", "acc_z"]].dropna()
    merged_timeframe.rename(columns={
        "vehicle_attitude_q[0]": "q_0",
        "vehicle_attitude_q[1]": "q_1",
        "vehicle_attitude_q[2]": "q_2",
        "vehicle_attitude_q[3]": "q_3",
    }, inplace=True)
    vehicle_attitude = merged_timeframe[["q_0", "q_1", "q_2", "q_3"]].dropna()
    merged_timeframe.rename(columns={
        "actuator_motors_control[0]": "m_0",
        "actuator_motors_control[1]": "m_1",
        "actuator_motors_control[2]": "m_2",
        "actuator_motors_control[3]": "m_3",
    }, inplace=True)
    actuator_controls_orig = merged_timeframe[["m_0", "m_1", "m_2", "m_3"]].dropna()
    merged_timeframe.rename(columns={
        "actuator_motors_rl_tools_control[0]": "mrl_0",
        "actuator_motors_rl_tools_control[1]": "mrl_1",
        "actuator_motors_rl_tools_control[2]": "mrl_2",
        "actuator_motors_rl_tools_control[3]": "mrl_3",
    }, inplace=True)
    if "mrl_0" in merged_timeframe.columns:
        actuator_controls_policy = merged_timeframe[["mrl_0", "mrl_1", "mrl_2", "mrl_3"]].dropna()
    else:
        actuator_controls_policy = None
    merged_timeframe.rename(columns={
        "vehicle_angular_velocity_xyz[0]": "w_x",
        "vehicle_angular_velocity_xyz[1]": "w_y",
        "vehicle_angular_velocity_xyz[2]": "w_z",
    }, inplace=True)
    vehicle_angular_rate = merged_timeframe[["w_x", "w_y", "w_z"]].dropna()
    merged_timeframe.rename(columns={
        "vehicle_angular_velocity_xyz_derivative[0]": "dw_x",
        "vehicle_angular_velocity_xyz_derivative[1]": "dw_y",
        "vehicle_angular_velocity_xyz_derivative[2]": "dw_z",
    }, inplace=True)
    vehicle_angular_rate_derivative = merged_timeframe[["dw_x", "dw_y", "dw_z"]].dropna()
# for axis in ["x", "y", "z"]:
#     vehicle_acceleration[axis] = scipy.signal.filtfilt(b, a, vehicle_acceleration[axis])
#     # vehicle_acceleration[axis] = vehicle_acceleration[axis] - vehicle_acceleration[axis].mean()
#     # vehicle_acceleration[axis] = vehicle_acceleration[axis] / vehicle_acceleration[axis].std()
# for motor in ["0", "1", "2", "3"]:
#     actuator_controls_orig[motor] = scipy.signal.filtfilt(b, a, actuator_controls_orig[motor])

    if plot:
        plt.plot(vehicle_acceleration.index, vehicle_acceleration["acc_x"], label="acc_x")
        plt.plot(vehicle_acceleration.index, vehicle_acceleration["acc_y"], label="acc_y")
        plt.plot(vehicle_acceleration.index, vehicle_acceleration["acc_z"], label="acc_z")
        plt.legend()
        plt.show()
        plt.plot(vehicle_angular_rate.index, vehicle_angular_rate["w_x"], label="w_x")
        plt.plot(vehicle_angular_rate.index, vehicle_angular_rate["w_y"], label="w_y")
        plt.plot(vehicle_angular_rate.index, vehicle_angular_rate["w_z"], label="w_z")
        plt.legend()
        plt.show()
        plt.plot(vehicle_angular_rate_derivative.index, vehicle_angular_rate_derivative["dw_x"], label="dw_x")
        plt.plot(vehicle_angular_rate_derivative.index, vehicle_angular_rate_derivative["dw_y"], label="dw_y")
        plt.plot(vehicle_angular_rate_derivative.index, vehicle_angular_rate_derivative["dw_z"], label="dw_z")
        plt.legend()
        plt.show()
        plt.plot(actuator_controls_orig.index, actuator_controls_orig["m_0"], label="m_0")
        plt.plot(actuator_controls_orig.index, actuator_controls_orig["m_1"], label="m_1")
        plt.plot(actuator_controls_orig.index, actuator_controls_orig["m_2"], label="m_2")
        plt.plot(actuator_controls_orig.index, actuator_controls_orig["m_3"], label="m_3")
        plt.legend()
        plt.show()
        if actuator_controls_policy is not None:
            plt.plot(actuator_controls_policy.index, actuator_controls_policy["mrl_0"], label="m_0")
            plt.plot(actuator_controls_policy.index, actuator_controls_policy["mrl_1"], label="m_1")
            plt.plot(actuator_controls_policy.index, actuator_controls_policy["mrl_2"], label="m_2")
            plt.plot(actuator_controls_policy.index, actuator_controls_policy["mrl_3"], label="m_3")
            plt.legend()
            plt.show()
        plt.plot(vehicle_attitude.index, vehicle_attitude["q_0"], label="q_0")
        plt.plot(vehicle_attitude.index, vehicle_attitude["q_1"], label="q_1")
        plt.plot(vehicle_attitude.index, vehicle_attitude["q_2"], label="q_2")
        plt.plot(vehicle_attitude.index, vehicle_attitude["q_3"], label="q_3")
        plt.legend()
        plt.show()
    actuator_controls_orig.describe()
    vehicle_attitude.describe()
    vehicle_acceleration.describe()
    vehicle_angular_rate.describe()
    return merged_timeframe

def throttle_thrust(tau, merged_timeframes, ax=None, squared_throttle_lower_bound = 0.0, squared_throttle_upper_bound = 4.0):
    print(f"tau: {tau}")
    throttles = []
    thrusts = []
    def filter_acc(acc):
        cutoff_frequency = 50
        sampling_rate = 1000
        nyquist = sampling_rate / 2
        b, a = scipy.signal.butter(3, cutoff_frequency / nyquist, btype="low")
        acc = scipy.signal.filtfilt(b, a, acc)
        return acc
    for merged_timeframe in merged_timeframes:
        throttle_thrust_sparse = merged_timeframe[["m_0", "m_1", "m_2", "m_3", "acc_x", "acc_y", "acc_z"]].dropna(how="all", subset=["m_0", "m_1", "m_2", "m_3", "acc_x", "acc_y", "acc_z"])
        throttle_thrust = throttle_thrust_sparse.interpolate(method="time").dropna()
        throttle = throttle_thrust["m_0"] ** 2 + throttle_thrust["m_1"] ** 2 + throttle_thrust["m_2"] ** 2 + throttle_thrust["m_3"] ** 2
        throttle_thrust = throttle_thrust[(throttle > squared_throttle_lower_bound) & (throttle < squared_throttle_upper_bound)]
        throttle = throttle_thrust["m_0"] ** 2 + throttle_thrust["m_1"] ** 2 + throttle_thrust["m_2"] ** 2 + throttle_thrust["m_3"] ** 2
        acceleration = pd.Series(np.sqrt(filter_acc(throttle_thrust["acc_x"]) ** 2 + filter_acc(throttle_thrust["acc_y"]) ** 2 + filter_acc(throttle_thrust["acc_z"]) ** 2), index=throttle_thrust.index)
        thrust = acceleration * mass
        throttle = throttle.ewm(halflife=f"{tau*np.log(2)} s", times=throttle_thrust.index).mean()
        throttles.append(throttle)
        thrusts.append(thrust)
    throttle = pd.concat(throttles)
    thrust = pd.concat(thrusts)
    model = LinearRegression(fit_intercept=False)

    model.fit(throttle.values.reshape(-1, 1), thrust.values.reshape(-1, 1))

    correlation = throttle.corr(thrust)
    if ax is not None:
        ax.scatter(throttle, thrust)
        ax.set_title(f"Tau = {tau}")
        ax.set_xlabel("Throttle")
        ax.set_ylabel("Thrust")
    return correlation, (model.coef_[0][0], model.intercept_[0] if model.fit_intercept else 0)

def plot_tau(tau, merged_timeframes, best_model=None, squared_throttle_lower_bound=0):
    fig = plt.figure()
    throttle_thrust(tau, merged_timeframes, squared_throttle_lower_bound=squared_throttle_lower_bound, ax=plt.gca())
    x = np.linspace(0, 4, 100)
    if best_model is not None:
        plt.plot(x, best_model[0] * x + best_model[1], color="red")
        plt.title(f"Tau = {tau}, Slope = {best_model[0]} Intercept = {best_model[1]}")
    else:
        plt.title(f"Tau = {tau}")
    plt.show()
    
def find_tau(merged_timeframes, plot=False, squared_throttle_lower_bound=0.2):

    tau_values = list(np.linspace(0.005, 0.5, 60))



    correlations = [throttle_thrust(tau, merged_timeframes, squared_throttle_lower_bound=squared_throttle_lower_bound) for tau in tau_values]
    best_tau_index = np.argmax([np.abs(x[0]) for x in correlations])
    best_tau = tau_values[best_tau_index]
    best_model = correlations[best_tau_index][1]
    if plot:
        plt.plot(tau_values, [x[0] for x in correlations])
        plt.title("Correlation")
        plt.xlabel("Tau")
        plt.ylabel("Correlation")
        plt.plot([best_tau, best_tau], [0, 1], color="red")
        plt.show()
    print(f"Best tau: {best_tau}")

    if plot:
        plot_tau(best_tau, merged_timeframes, best_model=best_model, squared_throttle_lower_bound=squared_throttle_lower_bound)
    animate = False
    if animate:

        fig, ax = plt.subplots()
        def animate(i):
            ax.clear()
            throttle_thrust(tau_values[i], merged_timeframes, ax, squared_throttle_lower_bound=squared_throttle_lower_bound)

        ani = FuncAnimation(fig, animate, frames=len(tau_values), interval=500)
        ani.save('throttle_thrust_animation.gif', writer='imagemagick', fps=3)
    return best_tau


def find_inertia(merged_timeframes, selected_tau, selected_slope, selected_intercept, plot=False):

    motor_thrusts_angular_accelerations = []
    for merged_timeframe in merged_timeframes:
        throttle_thrust_sparse = merged_timeframe[["m_0", "m_1", "m_2", "m_3", "dw_x", "dw_y", "dw_z"]].dropna(how="all", subset=["m_0", "m_1", "m_2", "m_3", "dw_x", "dw_y", "dw_z"])
        throttle_thrust = throttle_thrust_sparse.interpolate(method="time").dropna()
        motor_thrusts = [(selected_intercept + selected_slope*throttle_thrust[motor]**2).ewm(halflife=f"{selected_tau*np.log(2)} s", times=throttle_thrust.index).mean() for motor in ["m_0", "m_1", "m_2", "m_3"]]
        motor_thrusts_angular_acceleration = throttle_thrust.copy()
        motor_thrusts_angular_acceleration["m_0"] = motor_thrusts[0]
        motor_thrusts_angular_acceleration["m_1"] = motor_thrusts[1]
        motor_thrusts_angular_acceleration["m_2"] = motor_thrusts[2]
        motor_thrusts_angular_acceleration["m_3"] = motor_thrusts[3]

        motor = 0
        motor_torque = lambda motor, thrust: np.cross(rotor_positions[motor], np.array([0, 0, thrust]))
        torque = motor_thrusts_angular_acceleration["m_0"].map(lambda thrust: motor_torque(0, thrust))
        torque += motor_thrusts_angular_acceleration["m_1"].map(lambda thrust: motor_torque(1, thrust))
        torque += motor_thrusts_angular_acceleration["m_2"].map(lambda thrust: motor_torque(2, thrust))
        torque += motor_thrusts_angular_acceleration["m_3"].map(lambda thrust: motor_torque(3, thrust))
        motor_thrusts_angular_acceleration["torque_x"] = torque.map(lambda torque: torque[0])
        motor_thrusts_angular_acceleration["torque_y"] = torque.map(lambda torque: torque[1])
        motor_thrusts_angular_acceleration["torque_z"] = torque.map(lambda torque: torque[2])

        if plot:
            plt.plot(motor_thrusts_angular_acceleration.index, motor_thrusts_angular_acceleration["m_0"], label="m_0")
            plt.plot(motor_thrusts_angular_acceleration.index, motor_thrusts_angular_acceleration["m_1"], label="m_1")
            plt.plot(motor_thrusts_angular_acceleration.index, motor_thrusts_angular_acceleration["m_2"], label="m_2")
            plt.plot(motor_thrusts_angular_acceleration.index, motor_thrusts_angular_acceleration["m_3"], label="m_3")
            plt.plot(motor_thrusts_angular_acceleration.index, motor_thrusts_angular_acceleration["torque_x"], label="torque_x")
            plt.show()
        for axis in ["x", "y", "z"]:
            motor_thrusts_angular_acceleration[f"dw_{axis}"] = motor_thrusts_angular_acceleration[f"dw_{axis}"].ewm(halflife=f"{0.01} s", times=motor_thrusts_angular_acceleration.index).mean()
        motor_thrusts_angular_accelerations.append(motor_thrusts_angular_acceleration)
    
    motor_thrusts_angular_acceleration = pd.concat(motor_thrusts_angular_accelerations)

    def moment_of_inertia(axis, plot=False):
    # axis = "x"
        invert = axis == "y" or axis == "z"
        # model = LinearRegression()
        model = HuberRegressor()
        d = motor_thrusts_angular_acceleration.copy()
        d = d[(np.abs(d[f"dw_{axis}"]) > 10)]
        d = d[(np.abs(d[f"torque_{axis}"]) > 0.01)]
        model.fit(d[f"torque_{axis}"].values.reshape(-1, 1), d[f"dw_{axis}"].values.reshape(-1, 1) * (-1 if invert else 1))
        # I_inv, intercept = (model.coef_[0][0], model.intercept_[0]) # intercept should be close to 0
        I_inv, intercept = (model.coef_[0], model.intercept_) # intercept should be close to 0
        I = 1/I_inv
        if plot:
            plt.scatter(d[f"torque_{axis}"], d[f"dw_{axis}"] * (-1 if invert else 1))
            x = np.linspace(d[f"torque_{axis}"].min(), d[f"torque_{axis}"].max(), 100)
            plt.plot(x, x * I_inv + intercept, color="red")
            plt.title(f"I_{axis}: {I}")
            plt.show()
        return I
    I_x_inv = moment_of_inertia("x", plot=True)
    I_y_inv = moment_of_inertia("y", plot=True)
    return (1/I_x_inv, 1/I_y_inv)

plot_preproc = False

# logfile_input_up_and_down = "sysid/logs/2023-12-13-nishanth-sysid/log_45_2023-12-13-21-07-46-up-and-down.ulg" # up and down
# merged_df_0 = load_file(logfile_input_up_and_down, plot=plot_preproc)
# time_range_0 = (2, 1000)
# merged_timeframe_0 = get_timeframe(merged_df_0, *time_range_0, plot=plot_preproc)
logfile_input_yang_sysid = "sysid/logs/2023-12-14-yang-crash/log_60_2023-12-14-20-26-16.ulg" # yang sysid
merged_df_1 = load_file(logfile_input_yang_sysid, plot=plot_preproc)
# time_range_1 = (1.5, 1000)
# merged_timeframe_1 = get_timeframe(merged_df_1, *time_range_1, plot=plot_preproc)

# merged_timeframes = [merged_timeframe_0, merged_timeframe_1]

# squared_throttle_lower_bound = 0.3
# best_tau = find_tau(merged_timeframes, plot=True, squared_throttle_lower_bound=squared_throttle_lower_bound)


# selected_tau = best_tau #0.04

# _, (found_slope, found_intercept) = throttle_thrust(selected_tau, merged_timeframes, squared_throttle_lower_bound=squared_throttle_lower_bound)
# print(f"Found slope: {found_slope}, Found intercept: {found_intercept}")
# plot_tau(selected_tau, merged_timeframes, best_model=[found_slope, found_intercept], squared_throttle_lower_bound=squared_throttle_lower_bound)

# plt.show()

# selected_slope = 35
# selected_intercept = 0



# plot_preproc = True
# logfile_input_angular = "sysid/logs/2023-12-13-nishanth-sysid/log_46_2023-12-13-21-08-08-left-and-right.ulg" # left and right
# merged_df_angular_0 = load_file(logfile_input_angular, plot=plot_preproc)
# time_range_angular_0 = (3, 19)
# merged_timeframe_angular_0 = get_timeframe(merged_df_angular_0, *time_range_angular_0, plot=plot_preproc)

# merged_timeframes_angular = [merged_timeframe_angular_0]

# found_I_x, found_I_y = find_inertia(merged_timeframes_angular, selected_tau, selected_slope, selected_intercept, plot=True)

# merged_df_test_0 = load_file("sysid/logs/log_14_2023-12-15-14-49-46.ulg", plot=plot_preproc)
# merged_df_test_0 = load_file("sysid/logs/log_15_2023-12-15-15-25-40.ulg", plot=plot_preproc)
# merged_df_test_0 = load_file("sysid/logs/log_16_2023-12-15-15-33-22.ulg", plot=plot_preproc)
# merged_df_test_0 = load_file("sysid/logs/log_19_2023-12-15-15-46-50.ulg", plot=plot_preproc)
merged_df_test_0 = load_file("sysid/logs/log_26_2023-12-15-16-43-56.ulg", plot=plot_preproc)
find_col("actuator", merged_df_test_0)
merged_df_test_0.index.to_series().describe()
# motors_topic = "actuator_motors_rl_tools_control"
motors_topic = "actuator_motors_control"
merged_df_rl_tools_control = merged_df_test_0[[f"{motors_topic}[{motor}]" for motor in range(4)]].dropna()
for motor in range(4):
    plt.plot(merged_df_rl_tools_control.index, merged_df_rl_tools_control[f"{motors_topic}[{motor}]"], label=f"motor {motor}")
plt.show()


data = pd.read_csv("sysid/logs_csv/log_17_2023-12-15-15-39-16.ulg/log_17_2023-12-15-15-39-16_actuator_motors_0.csv")
data = pd.read_csv("sysid/logs_csv/log_21_2023-12-15-16-23-02.ulg/log_21_2023-12-15-16-23-02_actuator_motors_0.csv")
find_col("timestamp", data)
plt.plot(data["timestamp_sample"], data["timestamp_sample"].diff())
plt.plot(data["timestamp_sample"], data[""].diff())
plt.show()


df = load_file("sysid/logs/log_13_2023-12-16-23-14-12.ulg", plot=plot_preproc)
for col in df.columns:
    if "position" in col:
        print(col)


for motor in range(4):
    plt.plot(df[f"actuator_motors_rl_tools_control[{motor}]"].dropna())
plt.show()


df_so = df[[*[f"rl_tools_policy_status_state_observation[{axis_i}]" for axis_i in range(3)], "rl_tools_policy_status_exit_reason", "rl_tools_policy_status_command_stale"]].dropna()
df_so = df_so[df_so["rl_tools_policy_status_exit_reason"] == 0]
df_so = df_so.dropna()

other_axis = plt.twinx()
other_axis.plot(df_so.index, df_so["rl_tools_policy_status_command_stale"], label="rl_tools_policy_status_command_stale")
for axis_i, axis_name in enumerate(["x", "y", "z"]):
    plt.plot(df_so[f"rl_tools_policy_status_state_observation[{axis_i}]"], label=f"rl_tools_policy_status_state_observation[{axis_name}]")
# for motor in range(4):
#     plt.plot(df[f"actuator_motors_rl_tools_control[{motor}]"].dropna(), label=f"actuator_motors_rl_tools_control[{motor}]")
# for motor in range(4):
#     plt.plot(df[f"actuator_motors_control[{motor}]"].dropna(), label=f"actuator_motors_control[{motor}]")
# plt.plot(df["vehicle_local_position_z"].dropna(), label="vehicle_local_position_z")
plt.legend()
plt.show()