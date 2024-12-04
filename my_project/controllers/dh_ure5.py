import numpy as np

#base positions 
x,y,z =(0.1,0.2,0.3)

def dh_transformation(a, alpha, d, theta):
    """
    Computes the DH transformation matrix.
    Args:
        a (float): Link length.
        alpha (float): Link twist.
        d (float): Link offset.
        theta (float): Joint angle.
    Returns:
        np.ndarray: 4x4 transformation matrix.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Original DH parameters for UR5e
dh_params = [
    {"a": 0, "alpha": np.pi / 2, "d": 0.1625, "theta": 0},          # Joint 1
    {"a": -0.425, "alpha": 0, "d": 0, "theta": 0},                 # Joint 2
    {"a": -0.3922, "alpha": 0, "d": 0, "theta": 0},                # Joint 3
    {"a": 0, "alpha": np.pi / 2, "d": 0.1333, "theta": 0},         # Joint 4
    {"a": 0, "alpha": -np.pi / 2, "d": 0.0997, "theta": 0},        # Joint 5
    {"a": 0, "alpha": 0, "d": 0.0996, "theta": 0},                 # Joint 6
]

# Adjusted base frame (e.g., a rotation and translation)
base_transform = np.array([
    [1, 0, 0, x],  # 0.1m translation along x
    [0, 1, 0, y],  # 0.2m translation along y
    [0, 0, 1, z],  # 0.3m translation along z
    [0, 0, 0, 1]
])

# Compute the adjusted transformation matrices
def compute_adjusted_transformations(dh_params, base_transform):
    transformations = []
    current_transform = base_transform  # Start with the base transform
    for params in dh_params:
        dh_matrix = dh_transformation(params["a"], params["alpha"], params["d"], params["theta"])
        current_transform = np.dot(current_transform, dh_matrix)  # Chain transformations
        transformations.append(current_transform)
    return transformations

# Compute all adjusted transformations
adjusted_transformations = compute_adjusted_transformations(dh_params, base_transform)

# Print adjusted transformations for each joint
for i, transform in enumerate(adjusted_transformations):
    print(f"Adjusted transformation for Joint {i + 1}:\n{transform}\n")
