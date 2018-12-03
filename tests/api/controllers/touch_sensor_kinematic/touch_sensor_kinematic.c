#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag ts = wb_robot_get_device("touch sensor");
  WbDeviceTag motor = wb_robot_get_device("linear motor");
  wb_touch_sensor_enable(ts, TIME_STEP);

  int i;
  for (i = 0; i < 20; i++) {
    wb_motor_set_position(motor, -0.01 * i);

    wb_robot_step(TIME_STEP);
    double value = wb_touch_sensor_get_value(ts);
    value = wb_touch_sensor_get_value(ts);
    // printf("i=%d value = %g\n",i,value);
    if (i < 10)
      ts_assert_double_in_delta(value, 0.0, 0.0, "The \"bumper\" TouchSensor should return 0.0 when there is no collision.");
    else
      ts_assert_double_in_delta(value, 1.0, 0.0, "The \"bumper\" TouchSensor should return 1.0 when there is a collision.");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
