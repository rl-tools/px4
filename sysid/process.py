import scipy
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression, HuberRegressor
import matplotlib.pyplot as plt
from tqdm import tqdm

def throttle_thrust(df, tau, model, output_topic): 
    squared_throttle_lower_bound = 0.0
    squared_throttle_upper_bound = 4.0
    df = df.copy()
    def filter_acc(acc):
        cutoff_frequency = 50
        sampling_rate = 1000
        nyquist = sampling_rate / 2
        b, a = scipy.signal.butter(3, cutoff_frequency / nyquist, btype="low")
        acc = scipy.signal.filtfilt(b, a, acc)
        return acc
    selected_cols = [*[f"{output_topic}_control[{i}]" for i in range(4)], "vehicle_acceleration_xyz[2]"]
    throttle_thrust_sparse = df[selected_cols].dropna(how="all")

    old_index = throttle_thrust_sparse.index
    throttle_thrust_sparse.index = pd.to_datetime(throttle_thrust_sparse.index, unit="s")
    throttle_thrust = throttle_thrust_sparse.interpolate(method="time")
    throttle_thrust.index = old_index
    throttle_thrust = throttle_thrust.dropna()
    for i in range(4):
        topic = f"{output_topic}_control[{i}]"
        throttle_thrust[topic] = throttle_thrust[topic].ewm(halflife=f"{tau*np.log(2)} s", times=pd.to_datetime(throttle_thrust.index, unit="s")).mean()
    throttle = sum([throttle_thrust[f"{output_topic}_control[{i}]"] ** 2 for i in range(4)])
    acceleration = pd.Series(filter_acc(-throttle_thrust["vehicle_acceleration_xyz[2]"]), index=throttle_thrust.index)
    thrust = acceleration * model.mass
    df["throttle"] = throttle
    df["thrust"] = thrust
    return df

def fit_tau(dfs, tau, model, output_topic, fit_intercept=True):
    dfs = [throttle_thrust(df, tau, model, output_topic) for df in dfs]
    df = pd.concat(dfs)
    thrust_model = LinearRegression(fit_intercept=fit_intercept)

    df_sysid = df[["thrust", "throttle"]].dropna()
    thrust = df_sysid["thrust"]
    throttle = df_sysid["throttle"]
    thrust_model.fit(throttle.values.reshape(-1, 1), thrust.values.reshape(-1, 1))

    correlation = throttle.corr(thrust)
    return correlation, (thrust_model.coef_[0][0], thrust_model.intercept_[0] if thrust_model.fit_intercept else 0)

def find_tau(dfs, model, output_topic, tau_min=0.001, tau_max=0.2, fit_intercept=True):
    taus = np.linspace(tau_min, tau_max, 100)
    correlations = [fit_tau(dfs, tau, model, output_topic, fit_intercept)[0] for tau in tqdm(taus)]
    return np.array(list(zip(taus, correlations)))

def plot_thrust_curve(df_tt, model, output_topic, tau, slope, intercept, hovering_throttle):
    df_sysid = df_tt[["thrust", "throttle"]].dropna()
    thrust = df_sysid["thrust"]
    throttle = df_sysid["throttle"]
    plt.figure()
    plt.scatter(throttle, thrust)
    throttles = np.linspace(throttle.min(), throttle.max(), 100)
    predicted_thrusts = throttles * slope + intercept
    plt.plot(throttles, predicted_thrusts, color="red", label=f"thrust = {slope:.3f} * throttle^2 + {intercept:.3f}")
    hovering_point = model.mass * model.gravity
    plt.hlines(hovering_point, throttle.min(), throttle.max(), color="orange", label="Hovering Point")
    plt.vlines(hovering_throttle ** 2 * 4, thrust.min(), thrust.max(), color="green", label="Expected Hovering Throttle")
    plt.legend()
    plt.show()

def torque_angular_acceleration(df, model, output_topic, tau, slope, intercept):
    df_orig = df.copy()
    selected_cols = [*[f"{output_topic}_control[{i}]" for i in range(4)], *[f"vehicle_angular_velocity_xyz_derivative[{i}]" for i in range(3)]]
    df = df_orig[selected_cols].dropna(how="all").copy()
    old_index = df.index
    df.index = pd.to_datetime(df.index, unit="s")
    df = df.interpolate(method="time")
    df.index = old_index
    df = df.dropna()
    for dim, axis in zip(range(3), ["x", "y", "z"]):
        df[f"dw_{axis}"] = df[f"vehicle_angular_velocity_xyz_derivative[{dim}]"]
        df_orig[f"dw_{axis}"] = df[f"dw_{axis}"]

    motor_torque = lambda motor, thrust: np.cross(model.rotor_positions[motor], model.rotor_thrust_directions[motor]*thrust)
    throttle2thrust = lambda throttle: throttle ** 2 * slope + intercept / 4
    throttle2torque = lambda motor, throttle: motor_torque(motor, throttle2thrust(throttle))

    torque  = df[f"{output_topic}_control[0]"].ewm(halflife=f"{tau*np.log(2)} s", times=pd.to_datetime(df.index, unit="s")).mean().map(lambda throttle: throttle2torque(0, throttle))
    torque += df[f"{output_topic}_control[1]"].ewm(halflife=f"{tau*np.log(2)} s", times=pd.to_datetime(df.index, unit="s")).mean().map(lambda throttle: throttle2torque(1, throttle))
    torque += df[f"{output_topic}_control[2]"].ewm(halflife=f"{tau*np.log(2)} s", times=pd.to_datetime(df.index, unit="s")).mean().map(lambda throttle: throttle2torque(2, throttle))
    torque += df[f"{output_topic}_control[3]"].ewm(halflife=f"{tau*np.log(2)} s", times=pd.to_datetime(df.index, unit="s")).mean().map(lambda throttle: throttle2torque(3, throttle))
    df["torque_x"] = torque.map(lambda torque: torque[0])
    df["torque_y"] = torque.map(lambda torque: torque[1])
    df["torque_z"] = torque.map(lambda torque: torque[2])
    df_orig["torque_x"] = df["torque_x"]
    df_orig["torque_y"] = df["torque_y"]
    df_orig["torque_z"] = df["torque_z"]
    return df_orig

def fit_inertia(dfs, model, output_topic, tau, slope, intercept, verbose=False):
    dfs_tac = [torque_angular_acceleration(df, model, output_topic, tau, slope, intercept) for df in dfs]
    df_tt_tac = pd.concat(dfs_tac)
    def moment_of_inertia(axis, plot=False):
        invert = axis == "y" or axis == "z" # invert y and z to convert from FRD (angular acceleration) to FLU (torque)
        model = LinearRegression(fit_intercept=False)
        d = df_tt_tac[[f"torque_{axis}", f"dw_{axis}"]].dropna().copy()
        print(f"Correlation {axis} {d[f'torque_{axis}'].corr(d[f'dw_{axis}']) * (-1 if invert else 1)}") if verbose else None
        model.fit(d[f"torque_{axis}"].values.reshape(-1, 1), d[f"dw_{axis}"].values * (-1 if invert else 1))
        I_inv, intercept = (model.coef_[0], model.intercept_) # intercept should be close to 0
        I = 1/I_inv
        return I
    I_x = moment_of_inertia("x", plot=True)
    I_y = moment_of_inertia("y", plot=True)
    return I_x, I_y

def plot_torque_angular_acceleration_curve(dfs, model, output_topic, tau, slope, intercept):
    I_x, I_y = fit_inertia(dfs, model, output_topic, tau, slope, intercept, verbose=True)
    dfs_tac = [torque_angular_acceleration(df, model, output_topic, tau, slope, intercept) for df in dfs]
    df_tac = pd.concat(dfs_tac)
    for axis in ["x", "y"]:
        invert = axis == "y" or axis == "z"
        plt.figure()
        plt.scatter(df_tac[f"torque_{axis}"], df_tac[f"dw_{axis}"] * (-1 if invert else 1), s=0.1)
        x = np.linspace(df_tac[f"torque_{axis}"].min(), df_tac[f"torque_{axis}"].max(), 100)
        I = I_x if axis == "x" else I_y
        I_inv = 1 / I
        plt.plot(x, x * I_inv, color="red")
        plt.title(f"I_{axis}: {I}")
        plt.show()

