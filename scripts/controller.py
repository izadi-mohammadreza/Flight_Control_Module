import numpy as np
from scipy.spatial.transform import Rotation as R
from numpy.linalg import norm

def calculate_controller_output(position_W, r_position_W, velocity_W, r_velocity_W, R_B_W, uav_mass, gravity, r_acceleration_W, position_gain, velocity_gain, r_yaw, angular_velocity_B, inertia_matrix, attitude_gain, angular_rate_gain, r_yaw_rate):
    # Compute translational tracking errors
    e_p = position_W - r_position_W
    e_v = velocity_W - r_velocity_W
    I_a_d = -position_gain * e_p - velocity_gain * e_v + uav_mass * gravity * np.array([0, 0, 1]) + uav_mass * r_acceleration_W
    thrust = np.dot(I_a_d, R_B_W[:, 2])
    B_z_d = I_a_d / norm(I_a_d)

    # Calculate desired rotation matrix
    B_x_d = np.array([np.cos(r_yaw), np.sin(r_yaw), 0])
    B_y_d = np.cross(B_z_d, B_x_d)
    #B_y_d /= norm(B_y_d)
    B_y_d /= np.linalg.norm(B_y_d)
    R_d_w = np.column_stack([np.cross(B_y_d, B_z_d), B_y_d, B_z_d])
    desired_quaternion = R.from_matrix(R_d_w).as_quat()


    # Attitude tracking
    e_R_matrix = 0.5 * (R_d_w.T @ R_B_W - R_B_W.T @ R_d_w)
    e_R = np.array([e_R_matrix[2, 1], e_R_matrix[0, 2], e_R_matrix[1, 0]])

    omega_ref = r_yaw_rate * np.array([0, 0, 1])
    e_omega = angular_velocity_B - R_B_W.T @ R_d_w @ omega_ref
    tau = -attitude_gain * e_R - angular_rate_gain * e_omega + np.cross(angular_velocity_B, inertia_matrix @ angular_velocity_B)

    controller_torque_thrust = np.concatenate([tau, [thrust]])
    
    #print(controller_torque_thrust)
    return controller_torque_thrust, desired_quaternion
    
    
