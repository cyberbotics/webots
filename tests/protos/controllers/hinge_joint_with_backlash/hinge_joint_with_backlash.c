#include <stdio.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 16
#define TEST_DURATION 10.0f
#define SPEED 0.2f

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag joint_motor = wb_robot_get_device("joint_motor");
  WbDeviceTag joint_sensor = wb_robot_get_device("joint_sensor");
  wb_position_sensor_enable(joint_sensor, TIME_STEP);

  wb_motor_set_position(joint_motor, INFINITY);
  wb_motor_set_velocity(joint_motor, SPEED);

  // TEST #1: by switching direction BEFORE the backlash limit is reached, the endPoint shouldn't move
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    double position = wb_position_sensor_get_value(joint_sensor);

    if (position >= 0.10)
      wb_motor_set_velocity(joint_motor, -SPEED);
    if (position <= -0.10)
      wb_motor_set_velocity(joint_motor, SPEED);
  }

  wb_motor_set_velocity(joint_motor, 0);
  wb_robot_step(TIME_STEP);  // reset
  wb_motor_set_velocity(joint_motor, SPEED);

  // TEST #2: by switching direction AFTER the backlash limit is reached, the endPoint should move
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < 2 * TEST_DURATION) {
    double position = wb_position_sensor_get_value(joint_sensor);

    if (position >= 0.105)
      wb_motor_set_velocity(joint_motor, -SPEED);
    if (position <= -0.105)
      wb_motor_set_velocity(joint_motor, SPEED);
  }

  wb_robot_cleanup();

  return 0;
}
