import numpy as np

gravity = 9.81
mass = 3
rotor_x_displacement = 0.35
rotor_y_displacement = 0.35
rotor_positions = np.array([
    [ rotor_x_displacement, -rotor_y_displacement, 0],
    [-rotor_x_displacement,  rotor_y_displacement, 0],
    [ rotor_x_displacement,  rotor_y_displacement, 0],
    [-rotor_x_displacement, -rotor_y_displacement, 0]
])
rotor_thrust_directions = np.array([
    [0, 0, 1],
    [0, 0, 1],
    [0, 0, 1],
    [0, 0, 1]
])
rotor_torque_directions = np.array([
    [0, 0, -1],
    [0, 0, -1],
    [0, 0,  1],
    [0, 0,  1]
])