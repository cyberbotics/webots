#include <webots/motor.h>
#include <webots/robot.h>

#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  wb_robot_init();

  int time_step = (int)wb_robot_get_basic_time_step();
  WbDeviceTag motor = wb_robot_get_device("rotational motor 2");
  wb_motor_set_velocity(motor, 3.0);
  wb_motor_set_position(motor, 1000000);

  wb_robot_step(time_step);

  wb_robot_cleanup();
  return 0;
}
