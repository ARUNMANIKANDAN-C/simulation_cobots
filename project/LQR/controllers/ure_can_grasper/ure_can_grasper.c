#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 32

enum State { WAITING, GRASPING, ROTATING, RELEASING, RETURNING };

// Define target and initial positions
const double target_positions[] = {-1.88, -2.14, -2.38, -1.51}; // Target joint angles for placing the object
const double initial_positions[] = {0.0, 0.0, 0.0, 0.0};         // Initial joint angles for starting position

// Define LQR gain matrix K (example values; calculated offline)
const double K[4][8] = {
    {-10.0, -8.0, -6.0, -4.0, -2.0, -1.0, -0.5, -0.3},
    {-12.0, -9.0, -7.0, -5.0, -2.0, -1.0, -0.6, -0.4},
    {-15.0, -10.0, -8.0, -6.0, -3.0, -1.5, -0.7, -0.5},
    {-13.0, -11.0, -9.0, -7.0, -3.5, -1.8, -0.9, -0.6}
};

int main() {
  wb_robot_init();
  int state = WAITING;
  int counter = 0;

  // Initialize hand motors (gripper)
  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");

  // Initialize UR motors (arm joints)
  WbDeviceTag ur_motors[4];
  double max_velocity[4];
  ur_motors[0] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[1] = wb_robot_get_device("elbow_joint");
  ur_motors[2] = wb_robot_get_device("wrist_1_joint");
  ur_motors[3] = wb_robot_get_device("wrist_2_joint");

  // Get maximum velocity for each motor
  for (int i = 0; i < 4; i++) {
    max_velocity[i] = wb_motor_get_max_velocity(ur_motors[i]);
    if (!ur_motors[i]) {
      printf("Error: Motor %d not initialized properly.\n", i);
      return -1;
    }
  }

  // Initialize position sensors for arm joints
  WbDeviceTag position_sensors[4];
  position_sensors[0] = wb_robot_get_device("shoulder_lift_joint_sensor");
  position_sensors[1] = wb_robot_get_device("elbow_joint_sensor");
  position_sensors[2] = wb_robot_get_device("wrist_1_joint_sensor");
  position_sensors[3] = wb_robot_get_device("wrist_2_joint_sensor");

  // Enable each position sensor
  for (int i = 0; i < 4; i++) {
    if (position_sensors[i]) {
      wb_position_sensor_enable(position_sensors[i], TIME_STEP);
    } else {
      printf("Error: Position sensor %d not initialized properly.\n", i);
      return -1;
    }
  }

  // Initialize distance sensor to detect object
  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  if (distance_sensor) {
    wb_distance_sensor_enable(distance_sensor, TIME_STEP);
  } else {
    printf("Error: Distance sensor not initialized properly.\n");
    return -1;
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    double current_positions[8] = {0};  // Joint angles and velocities

    // Fetch current joint positions and velocities
    for (int i = 0; i < 4; i++) {
      current_positions[i] = wb_position_sensor_get_value(position_sensors[i]);
      current_positions[i + 4] = wb_motor_get_velocity(ur_motors[i]);
    }

    switch (state) {
      case WAITING:
        if (wb_distance_sensor_get_value(distance_sensor) < 500) { // Object detected
          printf("Object detected. Transitioning to GRASPING.\n");
          state = GRASPING;
          counter = 8;
          for (int i = 0; i < 3; i++)
            wb_motor_set_position(hand_motors[i], 0.85);  // Close gripper
        }
        break;

      case GRASPING:
        if (counter <= 0) {  // Grasp complete
          printf("Grasp complete. Transitioning to ROTATING.\n");
          state = ROTATING;
        }
        break;

      case ROTATING:
        for (int i = 0; i < 4; i++) {
          double error = target_positions[i] - current_positions[i];
          double control_input = fmin(fmax(-max_velocity[i], -K[i][i] * error), max_velocity[i]); // Clamp to max_velocity
          wb_motor_set_velocity(ur_motors[i], control_input);
          wb_motor_set_position(ur_motors[i], target_positions[i]);
        }
        if (fabs(current_positions[2] - target_positions[2]) < 0.05) {
          printf("Reached target position. Transitioning to RELEASING.\n");
          state = RELEASING;
          counter = 8;
        }
        break;

      case RELEASING:
        if (counter <= 0) {
          printf("Releasing object.\n");
          for (int i = 0; i < 3; i++)
            wb_motor_set_position(hand_motors[i], wb_motor_get_min_position(hand_motors[i]));  // Open gripper
          state = RETURNING;
        }
        break;

      case RETURNING:
        for (int i = 0; i < 4; i++) {
          double error = initial_positions[i] - current_positions[i];
          double control_input = fmin(fmax(-max_velocity[i], -K[i][i] * error), max_velocity[i]); // Clamp to max_velocity
          wb_motor_set_velocity(ur_motors[i], control_input);
          wb_motor_set_position(ur_motors[i], initial_positions[i]);
        }
        if (fabs(current_positions[2] - initial_positions[2]) < 0.05) {
          printf("Returned to initial position. Waiting for next object.\n");
          state = WAITING;
        }
        break;
    }
    counter--;
  }

  wb_robot_cleanup();
  return 0;
}
