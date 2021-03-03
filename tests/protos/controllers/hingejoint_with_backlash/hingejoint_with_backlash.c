#include <stdio.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 16

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag motInput = wb_robot_get_device("hingeInputMotor");
  WbDeviceTag posInput = wb_robot_get_device("hingeInputSensor");
  wb_position_sensor_enable(posInput, TIME_STEP);

  wb_motor_set_position(motInput, INFINITY);
  wb_motor_set_velocity(motInput, 0.2);

  while (wb_robot_step(TIME_STEP) != -1 && wb_robot_get_time() < 10.0) {
    double pos = wb_position_sensor_get_value(posInput);

    // by switching direction before the backlash point is reached, the endPoint shouldn't move
    if (pos >= 0.10)
      wb_motor_set_velocity(motInput, -0.2);
    if (pos <= -0.10)
      wb_motor_set_velocity(motInput, 0.2);
  }

  wb_robot_cleanup();

  return 0;
}
