#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag wheel1 = wb_robot_get_device("wheel1");
  WbDeviceTag wheel2;

  wb_motor_set_position(wheel1, INFINITY);
  wb_motor_set_velocity(wheel1, 50);
  if (argc > 1) {
    wheel2 = wb_robot_get_device("wheel2");

    wb_motor_set_position(wheel2, INFINITY);
    wb_motor_set_velocity(wheel2, 50);
  }

  wb_robot_step(TIME_STEP * 50);

  wb_motor_set_velocity(wheel1, 0);
  if (argc > 1)
    wb_motor_set_velocity(wheel2, 0);

  wb_robot_cleanup();
  return 0;
}
