from controller import Robot, DistanceSensor, Motor, PositionSensor, GPS
import numpy as np
from scipy.optimize import minimize

# Constants
TIME_STEP = 32
WAITING = 0
GRASPING = 1
ROTATING = 2
RELEASING = 3
ROTATING_BACK = 4
GRASP_THRESHOLD = 500  # Threshold for grasping

# Simple MPC function
def mpc_control(current_position, target_position, dt):
    # Objective: Minimize error between current position and target position over a time horizon
    def objective(u):
        # u is the control input (velocity), and we want to minimize position error
        predicted_position = current_position + u * dt  # Simple linear prediction
        return abs(predicted_position - target_position)

    # Initial guess (velocity input)
    u0 = [0.0]
    
    # Bounds for the control input (velocity)
    bounds = [(-1.0, 1.0)]  # Example: -1.0 to 1.0 velocity limits

    # Minimize the objective function using scipy
    result = minimize(objective, u0, bounds=bounds)
    control_input = result.x[0]  # The optimized control input (scalar)

    return control_input


def main():
    robot = Robot()
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
    position_sensors = []

    # Enable motors and attach position sensors
    for motor in ur_motors:
        motor.setVelocity(speed)
        sensor = motor.getPositionSensor()
        sensor.enable(TIME_STEP)
        position_sensors.append(sensor)

    # Distance sensor
    distance_sensor = robot.getDevice("distance sensor")
    distance_sensor.enable(TIME_STEP)

    # GPS device
    gps = robot.getDevice("gps")
    gps.enable(TIME_STEP)

    # Helper function to update GPS data
    def log_gps_data(action):
        gps_data = gps.getValues()
        print(f"{action} at GPS position: x={gps_data[0]}, y={gps_data[1]}, z={gps_data[2]}")

    # Main loop
    while robot.step(TIME_STEP) != -1:
        if state == WAITING:
            if distance_sensor.getValue() < GRASP_THRESHOLD:
                state = GRASPING
                print("Grasping can")
                log_gps_data("Grasping")
                for motor in hand_motors:
                    motor.setPosition(0.87)

        elif state == GRASPING:
            # Rotate arm to target position
            for i in range(4):
                current_position = position_sensors[i].getValue()
                target_position = target_positions[i]
                control_input = mpc_control(current_position, target_position, TIME_STEP / 1000.0)
                ur_motors[i].setPosition(current_position + control_input)
            #for i in range(4):
            #    ur_motors[i].setPosition(target_positions[i])
            state = ROTATING
            print("Rotating arm")

        elif state == ROTATING:
            if position_sensors[2].getValue() < -2.3:  # Example for wrist_1_joint sensor
                log_gps_data("Grasping")
                print("Releasing can")
                state = RELEASING
                # Release the object
                for motor in hand_motors:
                    motor.setPosition(motor.getMinPosition())

        elif state == RELEASING:
            log_gps_data("Releasing")
            print("Rotating arm back")
            # Move arm back to initial position
            for motor in ur_motors:
                motor.setPosition(0.0)
            state = ROTATING_BACK

        elif state == ROTATING_BACK:
            if position_sensors[0].getValue() > -0.1:  # Example for shoulder_lift_joint sensor
                state = WAITING
                print("Waiting can")

    robot.cleanup()


# Run the main function
if __name__ == "__main__":
    main()
