import numpy as np
import os
from Config import RobotConfig
from scipy.spatial.transform import Rotation as R

def LPF(data, cutoff_freq, fs):
    """
    Apply a low-pass filter to the data.
    
    Parameters:
    - data: The input data to filter.
    - cutoff_freq: The cutoff frequency for the low-pass filter.
    - fs: The sampling frequency of the data.
    
    Returns:
    - filtered_data: The filtered data.
    """
    from scipy.signal import butter, lfilter

    
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(1, normal_cutoff, btype='low', analog=False)
    filtered_data = lfilter(b, a, data)
    
    return filtered_data


def calculate_force_RP2Motor_single(theta, F_Rm, troque_beta):
    """
    Calculate motor torques from single RP force and torque values.
    
    Parameters:
    - theta: Single theta value
    - F_Rm: Single radial force value
    - troque_beta: Single beta torque value
    
    Returns:
    - troque_R: Right motor torque
    - troque_L: Left motor torque
    """
    J_theta_11 = np.polyval(np.polyder(RobotConfig.A), theta)
    
    J_theta = np.array([[J_theta_11,    0], 
                        [         0,    1]])
    J_eta = np.array([[ 1/2, -1/2],
                      [ 1/2,  1/2]])

    # 虛功法計算
    force = J_eta.transpose() @ J_theta.transpose() @ np.array([[F_Rm], [troque_beta]])
    
    return force[0, 0], force[1, 0]

def calculate_force_Motor2RP_single(theta, troque_R, troque_L):
    """
    Calculate RP force and torque from single motor torque values.
    
    Parameters:
    - theta: Single theta value
    - troque_R: Single right motor torque
    - troque_L: Single left motor torque
    
    Returns:
    - F_Rm: Radial force
    - troque_beta: Beta torque
    """
    J_theta_11 = np.polyval(np.polyder(RobotConfig.A), theta)
    
    J_theta = np.array([[J_theta_11,    0], 
                        [         0,    1]])
    J_eta = np.array([[ 1/2, -1/2],
                      [ 1/2,  1/2]])
    
    J_theta_inv = np.linalg.inv(J_theta)
    J_eta_inv = np.linalg.inv(J_eta)

    # 虛功法計算
    force = J_theta_inv.transpose() @ J_eta_inv.transpose() @ np.array([[troque_R], [troque_L]])

    return force[0, 0], force[1, 0]

def calculate_force_RP2Motor(theta, F_Rm, troque_beta):
    
    # Initialize output arrays
    troque_R = np.zeros_like(theta)
    troque_L = np.zeros_like(theta)
    
    # Process each element
    for i in range(len(theta)):
        troque_R[i], troque_L[i] = calculate_force_RP2Motor_single(theta[i], F_Rm[i], troque_beta[i])

    return troque_R, troque_L

def calculate_force_Motor2RP(theta, troque_R, troque_L):
    
    # Initialize output arrays
    F_Rm = np.zeros_like(theta)
    troque_beta = np.zeros_like(theta)
    
    # Process each element
    for i in range(len(theta)):
        F_Rm[i], troque_beta[i] = calculate_force_Motor2RP_single(theta[i], troque_R[i], troque_L[i])

    return F_Rm, -troque_beta

def check_file_exists(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File not found at {file_path}")
        exit()

def calculate_orientation(imu_orien_x, imu_orien_y, imu_orien_z, imu_orien_w):
    # 將四元數轉換為歐拉角 (roll, pitch, yaw)
    rotation = R.from_quat([imu_orien_x, imu_orien_y, imu_orien_z, imu_orien_w])
    euler_angles = rotation.as_euler('xyz', degrees=False)  # 順序: roll, pitch, yaw
    
    phi = euler_angles[0]  # roll
    psi = euler_angles[1]  # pitch
    
    return phi, psi