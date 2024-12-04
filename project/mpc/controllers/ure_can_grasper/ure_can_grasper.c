#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <stdio.h>

#define TIME_STEP 32

typedef enum { SUCCESS, FAILURE, RUNNING } Status;
typedef enum { IDLE, DETECT_CAN, GRASP_CAN, ROTATE_ARM, RELEASE_CAN, RETURN_ARM } Node;

double speed = 1.0;
int counter = 0;
int current_node = IDLE;
int print_flag = 1; // Flag to control one-time printing for each state
WbDeviceTag hand_motors[3];
WbDeviceTag ur_motors[4];
WbDeviceTag distance_sensor;
WbDeviceTag position_sensor;

const double target_positions[] = {-1.88, -2.14, -2.38, -1.51};

// Function to initialize devices
void initialize_devices() {
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");
  ur_motors[0] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[1] = wb_robot_get_device("elbow_joint");
  ur_motors[2] = wb_robot_get_device("wrist_1_joint");
  ur_motors[3] = wb_robot_get_device("wrist_2_joint");
  distance_sensor = wb_robot_get_device("distance sensor");
  position_sensor = wb_robot_get_device("wrist_1_joint_sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  for (int i = 0; i < 4; ++i)
    wb_motor_set_velocity(ur_motors[i], speed);
}

// Action Functions (now with timing)
Status detect_can() {
  double start_time = wb_robot_get_time();  // Record start time
  Status result = (wb_distance_sensor_get_value(distance_sensor) < 500) ? SUCCESS : FAILURE;
  double end_time = wb_robot_get_time();  // Record end time
  printf("Detect can action took %.4f seconds.\n", end_time - start_time);  // Log the time
  return result;
}

Status grasp_can() {
  double start_time = wb_robot_get_time();  // Record start time
  if (counter > 0) {
    for (int i = 0; i < 3; ++i)
      wb_motor_set_position(hand_motors[i], 0.85);
    counter--;
    double end_time = wb_robot_get_time();  // Record end time
    printf("Grasp can action took %.4f seconds.\n", end_time - start_time);  // Log the time
    return RUNNING;
  }
  double end_time = wb_robot_get_time();  // Record end time
  printf("Grasp can action took %.4f seconds.\n", end_time - start_time);  // Log the time
  return SUCCESS;
}

Status rotate_arm() {
  double start_time = wb_robot_get_time();  // Record start time
  for (int i = 0; i < 4; ++i)
    wb_motor_set_position(ur_motors[i], target_positions[i]);
  double end_time = wb_robot_get_time();  // Record end time
  printf("Rotate arm action took %.4f seconds.\n", end_time - start_time);  // Log the time
  return (wb_position_sensor_get_value(position_sensor) < -2.3) ? SUCCESS : RUNNING;
}

Status release_can() {
  double start_time = wb_robot_get_time();  // Record start time
  for (int i = 0; i < 3; ++i)
    wb_motor_set_position(hand_motors[i], wb_motor_get_min_position(hand_motors[i]));
  double end_time = wb_robot_get_time();  // Record end time
  printf("Release can action took %.4f seconds.\n", end_time - start_time);  // Log the time
  return SUCCESS;
}

Status return_arm() {
  double start_time = wb_robot_get_time();  // Record start time
  for (int i = 0; i < 4; ++i)
    wb_motor_set_position(ur_motors[i], 0.0);
  double end_time = wb_robot_get_time();  // Record end time
  printf("Return arm action took %.4f seconds.\n", end_time - start_time);  // Log the time
  return (wb_position_sensor_get_value(position_sensor) > -0.1) ? SUCCESS : RUNNING;
}

// Behavior tree for controlling the manipulator
void run_behavior_tree() {
  switch (current_node) {
    case IDLE:
      current_node = DETECT_CAN;
      print_flag = 1;
      break;

    case DETECT_CAN:
      if (detect_can() == SUCCESS) {
        current_node = GRASP_CAN;
        counter = 8; // Set counter for grasping duration
        print_flag = 1;
      }
      break;

    case GRASP_CAN:
      if (print_flag) {
        printf("Grasping can\n");
        print_flag = 0;
      }
      if (grasp_can() == SUCCESS) {
        current_node = ROTATE_ARM;
        print_flag = 1;
      }
      break;

    case ROTATE_ARM:
      if (print_flag) {
        printf("Rotating arm\n");
        print_flag = 0;
      }
      if (rotate_arm() == SUCCESS) {
        current_node = RELEASE_CAN;
        print_flag = 1;
      }
      break;

    case RELEASE_CAN:
      if (print_flag) {
        printf("Releasing can\n");
        print_flag = 0;
      }
      if (release_can() == SUCCESS) {
        current_node = RETURN_ARM;
        print_flag = 1;
      }
      break;

    case RETURN_ARM:
      if (print_flag) {
        printf("Returning arm\n");
        print_flag = 0;
      }
      if (return_arm() == SUCCESS) {
        current_node = IDLE;
        print_flag = 1;
        printf("Returning to Idle\n");
      }
      break;
  }
}

int main(int argc, char **argv) {
  wb_robot_init();
  if (argc == 2)
    sscanf(argv[1], "%lf", &speed);

  initialize_devices();

  while (wb_robot_step(TIME_STEP) != -1) {
    run_behavior_tree();
  }

  wb_robot_cleanup();
  return 0;
}
