# Import Webots modules
from controller import Robot, DistanceSensor, Motor, PositionSensor, GPS

# Constants
TIME_STEP = 32
WAITING = 0
GRASPING = 1
ROTATING = 2
RELEASING = 3
ROTATING_BACK = 4

# Main function
def main():
    robot = Robot()
    counter = 0
    state = WAITING
    target_positions = [-1.88, -2.14, -2.38, -1.51]
    speed = 1.0

    # Devices
    hand_motors = [
        robot.getDevice("finger_1_joint_1"),
        robot.getDevice("finger_2_joint_1"),
        robot.getDevice("finger_middle_joint_1")
    ]
    ur_motors = [
        robot.getDevice("shoulder_lift_joint"),
        robot.getDevice("elbow_joint"),
        robot.getDevice("wrist_1_joint"),
        robot.getDevice("wrist_2_joint")
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

    # GPS device
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
                # Print GPS data when grasping
                gps_data = gps.getValues()
                print(f"Grasping at GPS position: x={gps_data[0]}, y={gps_data[1]}, z={gps_data[2]}")
                
                for i in range(4):
                    ur_motors[i].setPosition(target_positions[i])
                print("Rotating arm")
                state = ROTATING
            elif state == ROTATING:
                if position_sensor.getValue() < -2.3:
                    counter = 8
                    gps_data = gps.getValues()
                    print(f"Grasping at GPS position: x={gps_data[0]}, y={gps_data[1]}, z={gps_data[2]}")
                    print("Releasing can")
                    state = RELEASING
                    for motor in hand_motors:
                        motor.setPosition(motor.getMinPosition())
            elif state == RELEASING:
                # Print GPS data when releasing
                gps_data = gps.getValues()
                print(f"Releasing at GPS position: x={gps_data[0]}, y={gps_data[1]}, z={gps_data[2]}")
                
                for motor in ur_motors:
                    motor.setPosition(0.0)
                print("Rotating arm back")
                state = ROTATING_BACK
            elif state == ROTATING_BACK:
                if position_sensor.getValue() > -0.1:
                    state = WAITING
                    print("Waiting can")
        
        counter -= 1

    robot.cleanup()

# Run the main function
if __name__ == "__main__":
    main()
