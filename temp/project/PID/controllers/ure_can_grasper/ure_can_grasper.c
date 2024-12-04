#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <math.h>

#define TIME_STEP 32

// PID Controller Structure
typedef struct {
  double kp;  // Proportional gain
  double ki;  // Integral gain
  double kd;  // Derivative gain
  double previous_error;
  double integral;
} PIDController;

// Initialize PID Controller
void init_pid(PIDController *pid, double kp, double ki, double kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->previous_error = 0.0;
  pid->integral = 0.0;
}

// Compute PID output
double compute_pid(PIDController *pid, double setpoint, double current_position) {
  double error = setpoint - current_position;
  pid->integral += error * TIME_STEP / 1000.0;  // Accumulate integral
  double derivative = (error - pid->previous_error) / (TIME_STEP / 1000.0);
  pid->previous_error = error;

  // PID formula
  return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

enum State { WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK };

int main(int argc, char **argv) {
  wb_robot_init();
  int counter = 0, i = 0;
  int state = WAITING;
  const double target_positions[] = {-1.88, -2.14, -2.38, -1.51};

  // PID controllers for each arm joint
  PIDController pid_controllers[4];
  init_pid(&pid_controllers[0], 2.0, 0.0, 0.5);  // PID for shoulder
  init_pid(&pid_controllers[1], 2.0, 0.0, 0.5);  // PID for elbow
  init_pid(&pid_controllers[2], 2.0, 0.0, 0.5);  // PID for wrist_1
  init_pid(&pid_controllers[3], 2.0, 0.0, 0.5);  // PID for wrist_2

  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");

  WbDeviceTag ur_motors[4];
  WbDeviceTag ur_position_sensors[4];  // Position sensors for each joint
  ur_motors[0] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[1] = wb_robot_get_device("elbow_joint");
  ur_motors[2] = wb_robot_get_device("wrist_1_joint");
  ur_motors[3] = wb_robot_get_device("wrist_2_joint");
  ur_position_sensors[0] = wb_robot_get_device("shoulder_lift_joint_sensor");
  ur_position_sensors[1] = wb_robot_get_device("elbow_joint_sensor");
  ur_position_sensors[2] = wb_robot_get_device("wrist_1_joint_sensor");
  ur_position_sensors[3] = wb_robot_get_device("wrist_2_joint_sensor");

  for (i = 0; i < 4; ++i) {
    wb_position_sensor_enable(ur_position_sensors[i], TIME_STEP);
    wb_motor_set_position(ur_motors[i], INFINITY);  // Enable velocity control
  }

  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    if (counter <= 0) {
      switch (state) {
        case WAITING:
          if (wb_distance_sensor_get_value(distance_sensor) < 500) {
            state = GRASPING;
            counter = 8;
            printf("Grasping object\n");
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], 0.85);
          }
          break;
        case GRASPING:
          printf("Rotating arm to target positions\n");
          state = ROTATING;
          break;
        case ROTATING: {
          int all_joints_in_position = 1;
          for (i = 0; i < 4; ++i) {
            double current_position = wb_position_sensor_get_value(ur_position_sensors[i]);
            double pid_output = compute_pid(&pid_controllers[i], target_positions[i], current_position);
            wb_motor_set_velocity(ur_motors[i], pid_output);
            if (fabs(target_positions[i] - current_position) > 0.01)  // Tolerance
              all_joints_in_position = 0;
          }
          if (all_joints_in_position) {
            printf("Arm in position, releasing object\n");
            state = RELEASING;
            counter = 8;
          }
          break;
        }
        case RELEASING:
          for (i = 0; i < 3; ++i)
            wb_motor_set_position(hand_motors[i], wb_motor_get_min_position(hand_motors[i]));  // Open gripper
          printf("Rotating arm back to initial position\n");
          state = ROTATING_BACK;
          break;
        case ROTATING_BACK: {
          int all_joints_reset = 1;
          for (i = 0; i < 4; ++i) {
            double current_position = wb_position_sensor_get_value(ur_position_sensors[i]);
            double pid_output = compute_pid(&pid_controllers[i], 0.0, current_position);  // Reset to 0
            wb_motor_set_velocity(ur_motors[i], pid_output);
            if (fabs(current_position) > 0.01)  // Tolerance
              all_joints_reset = 0;
          }
          if (all_joints_reset) {
            printf("Arm reset, waiting for new object\n");
            state = WAITING;
          }
          break;
        }
      }
    }
    counter--;
  }

  wb_robot_cleanup();
  return 0;
}
