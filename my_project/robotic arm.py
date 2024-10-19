from typing import List
import numpy as np

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


def inverse_dynamics(endeffector: List[int,int,int],) -> list[int,int,int,int,int,int]:
    