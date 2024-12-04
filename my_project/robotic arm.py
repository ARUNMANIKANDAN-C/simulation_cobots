import numpy as np
from typing import List

# DH matrix definition remains the same
def dh_matrix(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Forward kinematics for UR5e using its specific DH parameters
def ur5e_forward_kinematics(joint_angles: List[float]) -> List[float]:
    # UR5e DH parameters (a, alpha, d)
    a = [0, -0.425, -0.3922, 0, 0, 0]
    alpha = [-np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
    
    # Initialize the transformation matrix as an identity matrix
    T = np.eye(4)
    
    # Compute the forward kinematics by multiplying the transformation matrices
    for i in range(6):
        T = np.dot(T, dh_matrix(a[i], alpha[i], d[i], joint_angles[i]))
    
    # Extract the position of the end effector from the transformation matrix
    end_effector_pos = T[:3, 3]  # [x, y, z] position of the end effector
    
    return end_effector_pos.tolist()

# Example usage with joint angles in radians
joint_angles = [0, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, 0]
end_effector_position = ur5e_forward_kinematics(joint_angles)
print("End effector position:", end_effector_position)


def ur5e_inverse_kinematics(end_effector_pos: List[float], orientation: List[float]) -> List[float]:
    # UR5e specific parameters
    a = [0, -0.425, -0.3922, 0, 0, 0]
    alpha = [-np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
    
    # Calculate the wrist center position
    wrist_center = [
        end_effector_pos[0] - d[5] * orientation[0],
        end_effector_pos[1] - d[5] * orientation[1],
        end_effector_pos[2] - d[5] * orientation[2]
    ]
    
    # Calculate joint angles using geometric approach
    theta1 = np.arctan2(wrist_center[1], wrist_center[0])
    
    # Calculate theta2 and theta3 using the triangle formed by the first three joints
    r = np.sqrt(wrist_center[0]**2 + wrist_center[1]**2)
    s = wrist_center[2] - d[0]
    D = (r**2 + s**2 - a[1]**2 - a[2]**2) / (2 * a[1] * a[2])
    
    theta3 = np.arctan2(-np.sqrt(1 - D**2), D)  # Elbow down solution
    theta2 = np.arctan2(s, r) - np.arctan2(a[2] * np.sin(theta3), a[1] + a[2] * np.cos(theta3))
    
    # Calculate theta4, theta5, and theta6 based on the desired orientation
    theta4 = orientation[0]  # Assuming orientation is given in the form of Euler angles
    theta5 = orientation[1]
    theta6 = orientation[2]
    
    return [theta1, theta2, theta3, theta4, theta5, theta6]

end_effector_pos = [0.5, 0.5, 0.5]
orientation = [0, 0, 0]
joint_angles = ur5e_inverse_kinematics(end_effector_pos, orientation)
print("Joint angles:", joint_angles)

