#include <webots/motor.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag arm2 = wb_robot_get_device("arm2");
  WbDeviceTag arm3 = wb_robot_get_device("arm3");
  WbDeviceTag arm4 = wb_robot_get_device("arm4");
  wb_motor_set_position(arm2, 1.57);
  wb_motor_set_position(arm3, -2.635);
  wb_motor_set_position(arm4, 1.78);

  wb_robot_cleanup();

  return 0;
}
