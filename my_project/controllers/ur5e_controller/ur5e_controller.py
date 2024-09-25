import math  # Import the math module to use mathematical functions
from controller import Robot

# Create the Robot instance
robot = Robot()

# Get the timestep of the current simulation (used for sensor updates)
timestep = int(robot.getBasicTimeStep())

# Get the motors for the UR5e robot joints using Robot.getDevice
joints = []
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# Set the target joint velocity for faster movement (adjust as necessary)
joint_velocity = 2.0  # Increase the speed (radians per second)

for joint_name in joint_names:
    joint_motor = robot.getDevice(joint_name)
    joint_motor.setPosition(float('inf'))  # Set control mode to velocity control
    joint_motor.setVelocity(joint_velocity)  # Set a higher velocity for faster movement
    joints.append(joint_motor)

# Main control loop: Perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Example: Move the UR5e arm in a larger, faster motion (sinusoidal trajectory for demonstration)
    
    # Get the current simulation time in seconds
    current_time = robot.getTime()
    
    # Increase the amplitude for more significant joint movement
    positions = [
        2.0 * math.sin(current_time),         # Shoulder pan
        1.5 * math.sin(current_time + 1),     # Shoulder lift
        1.5 * math.sin(current_time + 2),     # Elbow
        1.0 * math.sin(current_time + 3),     # Wrist 1
        1.0 * math.sin(current_time + 4),     # Wrist 2
        1.0 * math.sin(current_time + 5)      # Wrist 3
    ]

    # Set the target position for each joint
    for i in range(len(joints)):
        joints[i].setPosition(positions[i])

# Cleanup: Webots will automatically clean up when the simulation is stopped
