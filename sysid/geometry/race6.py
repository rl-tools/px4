import numpy as np

gravity = 9.81
mass = 1
rotor_x_displacement = 0.0775
rotor_y_displacement = 0.0981
rotor_positions = np.array([
    [ rotor_x_displacement, -rotor_y_displacement, 0],
    [-rotor_x_displacement,  rotor_y_displacement, 0],
    [ rotor_x_displacement,  rotor_y_displacement, 0],
    [-rotor_x_displacement, -rotor_y_displacement, 0]
])