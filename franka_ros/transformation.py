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

# Example inputs
T_camera_temp = np.array([
    [-0.00443941, 0.00548865, -0.9999538, 0.02493073],
    [-0.66837034, -0.74382538, -0.00111581, -0.10342617],
    [-0.74381099, 0.66834893, 0.00697032, 0.92717104],
    [0, 0, 0, 1]
])

T_camera_object = np.array([
    [0.00272434, -0.57796307, 0.81605934, -0.16511092],
    [-0.64978756, 0.61927675, 0.44076329, 0.10135315],
    [-0.76011067, -0.53146589, -0.37386592, 0.58065968],
    [0, 0, 0, 1]
])


T_franka_temp = np.array([
    [0, -1, 0, 0.1],
    [0, 0, -1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
])

# Compute result
T_franka_object = compute_object_in_franka_frame(T_camera_temp, T_camera_object, T_franka_temp)
print("Transformation matrix of object with respect to Franka frame:")
print(T_franka_object)



# import numpy as np


# def compute_object_in_franka_frame(T_camera_temp, T_camera_object, T_franka_temp):
#     """Compute the transformation of the object in Franka's frame."""
#     # Compute inverse of T_camera_temp
#     R = T_camera_temp[:3, :3]
#     t = T_camera_temp[:3, 3]
#     R_inv = R.T
#     t_inv = -R_inv @ t
#     T_camera_temp_inv = np.eye(4)
#     T_camera_temp_inv[:3, :3] = R_inv
#     T_camera_temp_inv[:3, 3] = t_inv

#     # Compute T_franka_object
#     T_franka_object = T_franka_temp @ T_camera_temp_inv @ T_camera_object
#     return T_franka_object


# def extract_rpy(R):
#     """Extract roll, pitch, yaw from a rotation matrix."""
#     roll = np.arctan2(-R[2, 1], -R[2, 2])  # Roll (φ)
#     pitch = np.arcsin(-R[2, 0])  # Pitch (θ)
#     yaw = np.arctan2(R[1, 0], R[0, 0])  # Yaw (ψ)
#     return roll, pitch, yaw


# def modify_transformation_matrix(T, roll, pitch):
#     """Modify the given transformation matrix."""
#     # Extract yaw and translation from input matrix
#     R_input = T[:3, :3]
#     x, y, z = T[:3, 3]
#     _, _, yaw = extract_rpy(R_input)

#     # Construct new rotation matrix using extracted roll, pitch, yaw
#     R_new = create_rotation_matrix(roll, pitch, yaw)

#     # Create the new transformation matrix
#     T_new = T.copy()
#     T_new[:3, :3] = R_new  # Update rotation matrix
#     T_new[2, 3] += 0.03  # Add 0.03 to Z translation

#     return T_new


# def create_rotation_matrix(roll, pitch, yaw):
#     """Create a rotation matrix from roll, pitch, and yaw."""
#     R_x = np.array([
#         [1, 0, 0],
#         [0, np.cos(roll), -np.sin(roll)],
#         [0, np.sin(roll), np.cos(roll)]
#     ])

#     R_y = np.array([
#         [np.cos(pitch), 0, np.sin(pitch)],
#         [0, 1, 0],
#         [-np.sin(pitch), 0, np.cos(pitch)]
#     ])

#     R_z = np.array([
#         [np.cos(yaw), -np.sin(yaw), 0],
#         [np.sin(yaw), np.cos(yaw), 0],
#         [0, 0, 1]
#     ])

#     return R_z @ R_y @ R_x  # Combined rotation matrix


# # Given transformation matrix for extracting roll and pitch
# T1 = np.array([
#     [0.999954, -0.00943733, 0.00168051, 0],
#     [-0.00944125, -0.999953, 0.00233788, 0],
#     [0.00165837, -0.00235364, -0.999996, 0],
#     [0.307535, -0.000656046, 0.486955, 1]
# ])

# # Extract roll and pitch
# R1 = T1[:3, :3]
# roll, pitch, _ = extract_rpy(R1)

# # Given input transformation matrices
# T_camera_temp = np.array([
#     [-0.00443941, 0.00548865, -0.9999538, 0.02493073],
#     [-0.66837034, -0.74382538, -0.00111581, -0.10342617],
#     [-0.74381099, 0.66834893, 0.00697032, 0.92717104],
#     [0, 0, 0, 1]
# ])

# T_camera_object = np.array([
#     [0.00272434, -0.57796307, 0.81605934, -0.16511092],
#     [-0.64978756, 0.61927675, 0.44076329, 0.10135315],
#     [-0.76011067, -0.53146589, -0.37386592, 0.58065968],
#     [0, 0, 0, 1]
# ])

# T_franka_temp = np.array([
#     [0, -1, 0, 0.1],
#     [0, 0, -1, 0],
#     [1, 0, 0, 0],
#     [0, 0, 0, 1]
# ])

# # Compute T_franka_object
# T_franka_object = compute_object_in_franka_frame(T_camera_temp, T_camera_object, T_franka_temp)

# # Modify transformation matrix using extracted roll & pitch
# T_final = modify_transformation_matrix(T_franka_object, roll, pitch)

# # Compute the transpose
# T_transpose = T_final.T

# # Convert to 1D flattened array (comma-separated)
# T_1D = T_transpose.flatten()
# target_pose = ', '.join([f"{x:.8e}" for x in T_1D])

# # Print results
# print("Final Transformation Matrix (T_franka_object):")
# print(T_final)

# print("\nTranspose of Final Transformation Matrix:")
# print(T_transpose)

# print("\nFlattened Transformation Matrix (comma-separated):")
# print(target_pose)
