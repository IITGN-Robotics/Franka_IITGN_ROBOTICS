import numpy as np


def compute_object_in_franka_frame(T_camera_temp, T_camera_object, T_franka_temp):
    # Compute inverse of T_camera_temp
    R = T_camera_temp[:3, :3]
    t = T_camera_temp[:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    T_camera_temp_inv = np.eye(4)
    T_camera_temp_inv[:3, :3] = R_inv
    T_camera_temp_inv[:3, 3] = t_inv

    # Compute T_franka_object
    T_franka_object = T_franka_temp @ T_camera_temp_inv @ T_camera_object
    return T_franka_object


def extract_yaw(R):
    """Extract yaw (ψ) from a given rotation matrix."""
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return yaw


def create_rotation_matrix(roll, pitch, yaw):
    """Construct a rotation matrix given roll, pitch, and yaw angles."""
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    return R_z @ R_y @ R_x  # Combined rotation matrix


def modify_transformation_matrix(T):
    """Modify the given transformation matrix as per the requirements."""
    # Extract the yaw from the input transformation matrix
    R_input = T[:3, :3]
    yaw = extract_yaw(R_input)
    
    # Given roll and pitch values
    roll = np.arctan2(-0.00235364, -0.999996)  # Extracted roll
    pitch = np.arcsin(-0.00165837)  # Extracted pitch
    
    # Create new rotation matrix with given roll, pitch and extracted yaw
    R_new = create_rotation_matrix(roll, pitch, yaw)
    
    # Modify transformation matrix
    T_new = T.copy()
    T_new[:3, :3] = R_new  # Update rotation matrix
    T_new[2, 3] += 0.18  # Add 0.03 to Z-translation
    
    # Compute the transpose of the final transformation matrix
    T_transpose = T_new.T
    
    # Convert to 1D array
    T_1D = T_transpose.flatten()
    
    # Format the output as a comma-separated string
    target_pose = ', '.join([f"{x:.8e}" for x in T_1D])
    
    print("Final Transformation Matrix:")
    print(T_new)
    print("\nTranspose of Final Transformation Matrix:")
    print(T_transpose)
    print("\nFlattened Transformation Matrix with commas:")
    print(target_pose)
    
    return target_pose


# Example inputs
T_camera_temp = np.array([
    [-0.00443941, 0.00548865, -0.9999538, 0.02493073],
    [-0.66837034, -0.74382538, -0.00111581, -0.10342617],
    [-0.74381099, 0.66834893, 0.00697032, 0.92717104],
    [0, 0, 0, 1]
])

T_franka_temp = np.array([
    [0, -1, 0, 0.1],
    [0, 0, -1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
])

# Example input transformation matrix (T_camera_object)
T_camera_object = np.array([
    [0.00272434, -0.57796307, 0.81605934, -0.16511092],
    [-0.64978756, 0.61927675, 0.44076329, 0.10135315],
    [-0.76011067, -0.53146589, -0.37386592, 0.58065968],
    [0, 0, 0, 1]
])

# Compute T_franka_object
T_franka_object = compute_object_in_franka_frame(T_camera_temp, T_camera_object, T_franka_temp)

# Modify and get the flattened transpose
T_1D_transposed = modify_transformation_matrix(T_franka_object)
