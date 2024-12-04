from controller import Robot, DistanceSensor, Motor, PositionSensor, GPS
import numpy as np


from ikpy.chain import Chain
from ikpy.link import DHLink

# Define the UR5 chain using the Denavit-Hartenberg parameters
ur5_chain = Chain(name="UR5", links=[
    DHLink(
        name="base_link",
        d=0.089159, a=0, alpha=np.pi / 2, theta=0
    ),
    DHLink(
        name="shoulder_link",
        d=0, a=-0.425, alpha=0, theta=0
    ),
    DHLink(
        name="upper_arm_link",
        d=0, a=-0.39225, alpha=0, theta=0
    ),
    DHLink(
        name="forearm_link",
        d=0.10915, a=0, alpha=np.pi / 2, theta=0
    ),
    DHLink(
        name="wrist_1_link",
        d=0.09465, a=0, alpha=-np.pi / 2, theta=0
    ),
    DHLink(
        name="wrist_2_link",
        d=0.0823, a=0, alpha=0, theta=0
    ),
])

def ikpy_inverse_kinematics(target_position, target_orientation=None):
    """
    Use ikpy to compute the inverse kinematics for the target position.
    """
    print(target_position)
    target_frame = np.eye(4)  # Homogeneous transformation matrix
    target_frame[:3, 3] = target_position  # Set the target position

    # If orientation is specified, set the rotation part of the target frame
    if target_orientation is not None:
        target_frame[:3, :3] = target_orientation

    # Compute IK using ikpy
    ik_result = ur5_chain.inverse_kinematics(target_frame)

    # Return the joint angles (ignoring the base and fixed links if needed)
    return ik_result[1:-1]  # Exclude the first and last joints (if they are fixed)


# Constants
TIME_STEP = 32
WAITING = 0
GRASPING = 1
ROTATING = 2
RELEASING = 3
ROTATING_BACK = 4

# DH Parameters (UR5 robot example)
DH_PARAMS = [
    (0.089159, 0, 0, np.pi / 2),  # Joint 1
    (0, -0.425, 0, 0),           # Joint 2
    (0, -0.39225, 0, np.pi / 2), # Joint 3
    (0.10915, 0, 0, -np.pi / 2), # Joint 4
    (0.09465, 0, 0, np.pi / 2),  # Joint 5
    (0.0823, 0, 0, 0)            # Joint 6
]


def main():
    robot = Robot()
    counter = 0
    state = WAITING
    speed = 1.0

    # Devices
    hand_motors = [
        robot.getDevice("finger_1_joint_1"),
        robot.getDevice("finger_2_joint_1"),
        robot.getDevice("finger_middle_joint_1"),
    ]
    ur_motors = [
        robot.getDevice("shoulder_lift_joint"),
        robot.getDevice("elbow_joint"),
        robot.getDevice("wrist_1_joint"),
        robot.getDevice("wrist_2_joint"),
    ]

    # Set motor velocities
    for motor in ur_motors:
        motor.setVelocity(speed)

    # Distance sensor
    distance_sensor = robot.getDevice("distance sensor")
    distance_sensor.enable(TIME_STEP)

    # Position sensor for wrist_1_joint
    position_sensor = robot.getDevice("wrist_1_joint_sensor")
    position_sensor.enable(TIME_STEP)

    # GPS sensor
    gps = robot.getDevice("gps")
    gps.enable(TIME_STEP)

    # Main loop
    while robot.step(TIME_STEP) != -1:
        if counter <= 0:
            if state == WAITING:
                if distance_sensor.getValue() < 500:
                    state = GRASPING
                    counter = 8
                    print("Grasping can")
                    for motor in hand_motors:
                        motor.setPosition(0.85)
            elif state == GRASPING:
                # Move arm to grasp position
                joint_angles = inverse_kinematics(target_position=np.array( [0.140076616758057, 0.6780209293407507, -0.7746487983215595-0.61]))
                print(joint_angles)
                for i in range(4):
                    ur_motors[i].setPosition(joint_angles[i])
                gps_position = gps.getValues()
                print(f"Grasping at GPS position: {gps_position}")
                state = ROTATING
            elif state == ROTATING:
                if position_sensor.getValue() < -2.3:
                    counter = 8
                    print("Releasing can")
                    state = RELEASING
                    for motor in hand_motors:
                        motor.setPosition(motor.getMinPosition())
            elif state == RELEASING:
                # Move arm to release position
                joint_angles = inverse_kinematics(target_position=np.array([1.0, 0.0, 0.5]))
                set_arm_position(ur_motors, joint_angles)

                gps_position = gps.getValues()
                print(f"Releasing at GPS position: {gps_position}")
                state = ROTATING_BACK
            elif state == ROTATING_BACK:
                if position_sensor.getValue() > -0.1:
                    state = WAITING
                    print("Waiting can")

        counter -= 1

    robot.cleanup()


def dh_transformation(theta, d, a, alpha):
    """
    Compute individual transformation matrix using Denavit-Hartenberg parameters.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])


def forward_kinematics(joint_angles):
    """
    Compute the forward kinematics to find the end-effector position.
    """
    t = np.eye(4)
    for i, (d, a, alpha, _) in enumerate(DH_PARAMS):
        t = t @ dh_transformation(joint_angles[i], d, a, alpha)
    return t[:3, 3]  # Return only the x, y, z position


def inverse_kinematics(target_position, max_iterations=100, tolerance=1e-3):
    """
    Simplified inverse kinematics using gradient descent or numerical methods.
    """
    joint_angles = np.zeros(len(DH_PARAMS))  # Initial guess for joint angles
    for _ in range(max_iterations):
        # Forward kinematics to get the current position
        current_position = forward_kinematics(joint_angles)

        # Compute error
        error = target_position - current_position
        if np.linalg.norm(error) < tolerance:
            break

        # Update joint angles (this example uses a simple Jacobian transpose approach)
        jacobian = compute_jacobian(joint_angles)
        joint_angles += jacobian.T @ error * 0.1 
        #joint_angles[3] =1.2
    # Small step size for stability
    return joint_angles


def compute_jacobian(joint_angles):
    """
    Compute the Jacobian matrix for the UR5 robot using numerical differentiation.
    """
    delta = 1e-6
    jacobian = np.zeros((3, len(DH_PARAMS)))  # 3 rows for x, y, z; 6 columns for joints
    current_position = forward_kinematics(joint_angles)

    for i in range(len(DH_PARAMS)):
        perturbed_angles = joint_angles.copy()
        perturbed_angles[i] += delta
        perturbed_position = forward_kinematics(perturbed_angles)
        jacobian[:, i] = (perturbed_position - current_position) / delta

    return jacobian


def set_arm_position(ur_motors, joint_angles):
    for i, motor in enumerate(ur_motors):
        motor.setPosition(joint_angles[i])


if __name__ == "__main__":
    main()
