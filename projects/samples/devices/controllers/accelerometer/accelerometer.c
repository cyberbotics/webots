#include <webots/accelerometer.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();

  int time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  WbDeviceTag front_led = wb_robot_get_device("front led");
  WbDeviceTag back_led = wb_robot_get_device("back led");
  WbDeviceTag left_led = wb_robot_get_device("left led");
  WbDeviceTag right_led = wb_robot_get_device("right led");

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.5);
  wb_motor_set_velocity(right_motor, -0.5);

  while (wb_robot_step(time_step) != -1) {
    const double *acceleration = wb_accelerometer_get_values(accelerometer);

    if (fabs(acceleration[0]) > fabs(acceleration[2])) {
      wb_led_set(front_led, false);
      wb_led_set(back_led, false);
      wb_led_set(left_led, acceleration[0] > 0.0);
      wb_led_set(right_led, acceleration[0] < 0.0);
    } else {
      wb_led_set(front_led, acceleration[2] > 0.0);
      wb_led_set(back_led, acceleration[2] < 0.0);
      wb_led_set(left_led, false);
      wb_led_set(right_led, false);
    }
  };

  wb_robot_cleanup();

  return 0;
}
