/*
 * Description:  Test Webots ray collision with Joint's end point solid.
 *               This controller handles both "obstacle" and "sensor" robot,
 *               and tests:
 *               - initial computation of bounding sphere for end point solid
 *               - update of bounding sphere when joint moves
 */

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <string.h>

#define TIME_STEP 32

enum { OBSTACLE, SENSOR };

int main(int argc, char **argv) {
  ts_setup(argv[1]);
  WbDeviceTag motor = 0, ir1 = 0, ir2 = 0, ir3 = 0;
  int robotType, i;
  double value, tolerance = 50;
  const char *name;

  name = wb_robot_get_name();
  if (strcmp(name, "obstacle") == 0) {
    robotType = OBSTACLE;
    motor = wb_robot_get_device("motor");
  } else {
    robotType = SENSOR;
    ir1 = wb_robot_get_device("ir1");
    ir2 = wb_robot_get_device("ir2");
    ir3 = wb_robot_get_device("ir3");
    wb_distance_sensor_enable(ir1, TIME_STEP);
    wb_distance_sensor_enable(ir2, TIME_STEP);
    wb_distance_sensor_enable(ir3, TIME_STEP);
  }

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  if (robotType == SENSOR) {
    // check initial sensor values
    value = wb_distance_sensor_get_value(ir1);
    ts_assert_double_in_delta(value, 350, tolerance,
                              "The value measured by the light sensor \"ir1\" should be 350 and not %g before joint moves",
                              value);

    value = wb_distance_sensor_get_value(ir2);
    ts_assert_double_in_delta(value, 350, tolerance,
                              "The value measured by the light sensor \"ir2\" should be 350 and not %g before joint moves",
                              value);

    value = wb_distance_sensor_get_value(ir3);
    ts_assert_double_in_delta(value, 1000, tolerance,
                              "The value measured by the light sensor \"ir3\" should be 1000 and not %g before joint moves",
                              value);
  }

  wb_robot_step(TIME_STEP);

  if (robotType == OBSTACLE)
    wb_motor_set_position(motor, 0.1);

  i = 10;
  while (i > 0 && wb_robot_step(TIME_STEP) != -1) {
    --i;
  }

  if (robotType == SENSOR) {
    // check updated sensor values
    value = wb_distance_sensor_get_value(ir1);
    ts_assert_double_in_delta(value, 350, tolerance,
                              "The value measured by the light sensor \"ir1\" should be 350 and not %g after joint moved",
                              value);

    value = wb_distance_sensor_get_value(ir2);
    ts_assert_double_in_delta(value, 1000, tolerance,
                              "The value measured by the light sensor \"ir2\" should be 1000 and not %g after joint moved",
                              value);

    value = wb_distance_sensor_get_value(ir3);
    ts_assert_double_in_delta(value, 350, tolerance,
                              "The value measured by the light sensor \"ir3\" should be 350 and not %g after joint moved",
                              value);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
