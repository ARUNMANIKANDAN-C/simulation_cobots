#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <time.h>

#define TIME_STEP 32

enum State { WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK };

FILE *log_file;
clock_t start_time, end_time;

int main(int argc, char **argv) {
  wb_robot_init();
  int counter = 0;
  int state = WAITING;
  const double target_positions[] = {-1.88, -2.14, -2.38, -1.51};
  double speed = 1.0;

  if (argc == 2)
    sscanf(argv[1], "%lf", &speed);

  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");
  WbDeviceTag ur_motors[4];
  ur_motors[0] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[1] = wb_robot_get_device("elbow_joint");
  ur_motors[2] = wb_robot_get_device("wrist_1_joint");
  ur_motors[3] = wb_robot_get_device("wrist_2_joint");
  for (int i = 0; i < 4; ++i)
    wb_motor_set_velocity(ur_motors[i], speed);

  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  WbDeviceTag position_sensor = wb_robot_get_device("wrist_1_joint_sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  log_file = fopen("fsm_performance_log.csv", "w");
  fprintf(log_file, "Cycle,State,TimeSpent\n");

  while (wb_robot_step(TIME_STEP) != -1) {
    if (counter <= 0) {
      start_time = clock();

      switch (state) {
        case WAITING:
          if (wb_distance_sensor_get_value(distance_sensor) < 500) {
            state = GRASPING;
            counter = 8;
            printf("Grasping can\n");
            for (int i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], 0.85);
          }
          break;
        case GRASPING:
          for (int i = 0; i < 4; ++i)
            wb_motor_set_position(ur_motors[i], target_positions[i]);
          printf("Rotating arm\n");
          state = ROTATING;
          break;
        case ROTATING:
          if (wb_position_sensor_get_value(position_sensor) < -2.3) {
            counter = 8;
            printf("Releasing can\n");
            state = RELEASING;
            for (int i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], wb_motor_get_min_position(hand_motors[i]));
          }
          break;
        case RELEASING:
          for (int i = 0; i < 4; ++i)
            wb_motor_set_position(ur_motors[i], 0.0);
          printf("Rotating arm back\n");
          state = ROTATING_BACK;
          break;
        case ROTATING_BACK:
          if (wb_position_sensor_get_value(position_sensor) > -0.1) {
            state = WAITING;
            printf("Waiting for next can\n");
          }
          break;
      }

      end_time = clock();
      double time_spent = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
      fprintf(log_file, "%d,%d,%lf\n", counter, state, time_spent);
    }
    counter--;
  };

  fclose(log_file);
  wb_robot_cleanup();
  return 0;
}
