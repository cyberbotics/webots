#include <stdio.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 16
#define TEST_DURATION 10.0f
#define SPEED 0.2f

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag motorInput = wb_robot_get_device("hingeInputMotor");
  WbDeviceTag positionInput = wb_robot_get_device("hingeInputSensor");
  wb_position_sensor_enable(positionInput, TIME_STEP);

  wb_motor_set_position(motorInput, INFINITY);
  wb_motor_set_velocity(motorInput, SPEED);

  // TEST #1: by switching direction BEFORE the backlash limit is reached, the endPoint shouldn't move
  printf("ROBOT Test #1: Start.\n");
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    double pos = wb_position_sensor_get_value(positionInput);

    if (pos >= 0.10)
      wb_motor_set_velocity(motorInput, -SPEED);
    if (pos <= -0.10)
      wb_motor_set_velocity(motorInput, SPEED);
  }
  printf("ROBOT Test #1: Done.\n");

  wb_motor_set_velocity(motorInput, 0);
  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);
  wb_motor_set_velocity(motorInput, SPEED);

  // TEST #2: by switching direction AFTER the backlash limit is reached, the endPoint shouldn't move
  printf("ROBOT Test #2: Start.\n");

  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < 2 * TEST_DURATION) {
    // command
    double pos = wb_position_sensor_get_value(positionInput);

    if (pos >= 0.105)
      wb_motor_set_velocity(motorInput, -SPEED);
    if (pos <= -0.105)
      wb_motor_set_velocity(motorInput, SPEED);
  }

  printf("ROBOT Test #2: Done.\n");

  wb_robot_cleanup();

  return 0;
}
