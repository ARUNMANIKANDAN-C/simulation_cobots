#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 32
#define PREDICTION_HORIZON 10
#define CONTROL_STEP_SIZE 0.05
#define TARGET_TOLERANCE 0.01

// Define the target positions for each joint
const double target_positions[] = {-1.88, -2.14, -2.38, -1.51};
const int NUM_JOINTS = 4;

// Motor and position sensor tags
WbDeviceTag motors[NUM_JOINTS], position_sensors[NUM_JOINTS];

// Initialize motors and position sensors
void init_motors_and_sensors() {
  const char *motor_names[] = {"shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint"};
  const char *sensor_names[] = {"shoulder_lift_joint_sensor", "elbow_joint_sensor", "wrist_1_joint_sensor", "wrist_2_joint_sensor"};

  for (int i = 0; i < NUM_JOINTS; ++i) {
    motors[i] = wb_robot_get_device(motor_names[i]);
    position_sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_position_sensor_enable(position_sensors[i], TIME_STEP);
    wb_motor_set_position(motors[i], INFINITY);  // Velocity control mode
  }
}

// Simple linear dynamics model for prediction (replace with UR5-specific dynamics if available)
void predict_next_position(double *current_position, double control_input, double *predicted_position) {
  double A = 1.0;  // Simplified identity model
  double B = 0.05; // Control effect (tunable parameter)
  *predicted_position = A * (*current_position) + B * control_input;
}

// Cost function for MPC optimization
double cost_function(double target_position, double predicted_position, double control_input) {
  double position_error = target_position - predicted_position;
  return pow(position_error, 2) + 0.01 * pow(control_input, 2); // Weighting factor for control effort
}

// Main function
int main(int argc, char **argv) {
  wb_robot_init();
  init_motors_and_sensors();

  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    // Loop through each joint for MPC control
    for (int i = 0; i < NUM_JOINTS; ++i) {
      double current_position = wb_position_sensor_get_value(position_sensors[i]);
      double target_position = target_positions[i];
      double best_control_input = 0.0;
      double min_cost = INFINITY;

      // Brute-force search for optimal control input
      for (double control_input = -1.0; control_input <= 1.0; control_input += CONTROL_STEP_SIZE) {
        double predicted_position = current_position;

        // Predict over the horizon
        double total_cost = 0.0;
        for (int j = 0; j < PREDICTION_HORIZON; ++j) {
          predict_next_position(&predicted_position, control_input, &predicted_position);
          total_cost += cost_function(target_position, predicted_position, control_input);
        }

        // Update the best control input if cost is minimized
        if (total_cost < min_cost) {
          min_cost = total_cost;
          best_control_input = control_input;
        }
      }

      // Apply the best control input to the motor
      wb_motor_set_velocity(motors[i], best_control_input);
      
      // Check if the joint reached the target
      if (fabs(target_position - current_position) <= TARGET_TOLERANCE) {
        printf("Joint %d has reached its target position.\n", i);
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}
