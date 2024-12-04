import roboticstoolbox as rtb
from spatialmath import SE3
import math
from controller import Robot, Motor, DistanceSensor, PositionSensor

TIME_STEP = 32
WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK = range(5)

# Define the robotic arm using DH parameters (adjust these to match your robot)
robot = rtb.DHRobot([
    rtb.RevoluteDH(a=0, d=0.1, alpha=0, offset=0),  # shoulder_lift_joint
    rtb.RevoluteDH(a=0, d=0.3, alpha=0, offset=0),  # elbow_joint
    rtb.RevoluteDH(a=0, d=0.2, alpha=0, offset=0),  # wrist_1_joint
    rtb.RevoluteDH(a=0, d=0.2, alpha=0, offset=0),  # wrist_2_joint
], name="robot_arm")

def main():
    # Initialize Webots
    sim_robot = Robot()
    hand_motors = [
        sim_robot.getDevice("finger_1_joint_1"),
        sim_robot.getDevice("finger_2_joint_1"),
        sim_robot.getDevice("finger_middle_joint_1")
    ]
    ur_motors = [
        sim_robot.getDevice("shoulder_lift_joint"),
        sim_robot.getDevice("elbow_joint"),
        sim_robot.getDevice("wrist_1_joint"),
        sim_robot.getDevice("wrist_2_joint")
    ]
    distance_sensor = sim_robot.getDevice("distance sensor")
    distance_sensor.enable(TIME_STEP)

    position_sensor = sim_robot.getDevice("wrist_1_joint_sensor")
    position_sensor.enable(TIME_STEP)

    for motor in ur_motors:
        motor.setVelocity(1.0)

    state = WAITING
    counter = 0

    while sim_robot.step(TIME_STEP) != -1:
        if counter <= 0:
            if state == WAITING:
                if distance_sensor.getValue() < 500:
                    state = GRASPING
                    counter = 8
                    print("Grasping can")
                    for motor in hand_motors:
                        motor.setPosition(0.85)

            elif state == GRASPING:
                # Define the target pose
                target_pose = SE3(0.1, 0.2, 0.4)  # Replace with the desired (x, y, z) in meters
                ik_solution = robot.ikine_LMS(target_pose)  # Solve IK
                joint_angles = ik_solution.q
                if not ik_solution.success:
                    print("IK failed")
                    continue
                print(f"Joint angles: {joint_angles}")

                # Apply joint angles to the motors
                for i, motor in enumerate(ur_motors):
                    motor.setPosition(joint_angles[i])
                print("Rotating arm")
                state = ROTATING

            elif state == ROTATING:
                if position_sensor.getValue() < -2.3:
                    counter = 8
                    print("Releasing can")
                    state = RELEASING
                    for motor in hand_motors:
                        motor.setPosition(motor.getMinPosition())

            elif state == RELEASING:
                for motor in ur_motors:
                    motor.setPosition(0.0)
                print("Rotating arm back")
                state = ROTATING_BACK

            elif state == ROTATING_BACK:
                if position_sensor.getValue() > -0.1:
                    state = WAITING
                    print("Waiting for next can")
        counter -= 1


if __name__ == "__main__":
    main()
