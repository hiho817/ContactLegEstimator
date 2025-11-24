"""
Configuration parameters for quadruped robot dynamics
"""
SIM = True  # Simulation mode flag

class RobotConfig:
    """Robot configuration parameters"""

    if SIM:
    
        # Robot physical parameters
        BASE_WEIGHT = 19.0      # kg - Base mass
        LEG_WEIGHT = 0.65       # kg - Leg mass
        BASE_LENGTH = 0.5       # m - Base length
        BASE_WIDTH = 0.2        # m - Base width
        BASE_HEIGHT = 0.1       # m - Base height
        ROBOT_WEIGHT = BASE_WEIGHT + 4 * LEG_WEIGHT

        # Geometric parameters
        X_OFFSET = 0.222       # m - X offset of hip from base center
        Y_OFFSET = 0.193       # m - Y offset of hip from base center
    
    else:
        # Robot physical parameters
        BASE_WEIGHT = 18.65      # kg - Base mass
        LEG_WEIGHT = 0.68        # kg - Leg mass
        BASE_LENGTH = 0.61       # m - Base length
        BASE_WIDTH = 0.22       # m - Base width
        BASE_HEIGHT = 0.163       # m - Base height
        ROBOT_WEIGHT = BASE_WEIGHT + 4 * LEG_WEIGHT

        # Geometric parameters
        X_OFFSET = 0.222       # m - X offset of hip from base center
        Y_OFFSET = 0.193       # m - Y offset of hip from base center

    # Base inertias
    BODY_I_XX = BASE_WEIGHT * (BASE_LENGTH**2 + BASE_HEIGHT**2) / 12.0  # kg*m^2 - Body inertia around x-axis
    BODY_I_YY = BASE_WEIGHT * (BASE_WIDTH**2 + BASE_HEIGHT**2) / 12.0   # kg*m^2 - Body inertia around y-axis

    # Physical constants
    GRAVITY = 9.81          # m/s^2 - Gravitational acceleration

    # Fitting polynomial coefficients
    import numpy as np
    A = np.array([-0.0035, 0.0110, 0.0030, 0.0500, -0.0132]) #rm_coeff
    # B = np.array([0.0001, -0.0001, -0.0013, 0.0043, 0.0041]) #Io_coeff
    B = np.array([1e-06, -1E-05, 0.0001, -0.0002, -0.0012, 0.0042, 0.0041]) #Ic_coeff

class SensorConfig:
    """Sensor configuration parameters"""

    pass