#include <math.h>
#include <stdio.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 8

int main() {
  wb_robot_init();

  WbDeviceTag motor = wb_robot_get_device("rotational motor");
  wb_motor_set_position(motor, INFINITY);
  wb_motor_set_velocity(motor, 0.2);

  WbDeviceTag sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(sensor, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    double pos = wb_position_sensor_get_value(sensor);

    if (pos >= M_PI / 4)
      wb_motor_set_velocity(motor, -0.2);
    if (pos <= -M_PI / 4)
      wb_motor_set_velocity(motor, 0.2);
  }

  wb_robot_cleanup();

  return 0;
}
