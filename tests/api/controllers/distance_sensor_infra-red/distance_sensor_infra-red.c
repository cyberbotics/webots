#include <webots/distance_sensor.h>
#include <webots/pen.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag pen0, pen1, ds0, ds1;
  double value = 0.0;

  ds0 = wb_robot_get_device("ds0");
  ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  pen0 = wb_robot_get_device("pen0");
  pen1 = wb_robot_get_device("pen1");
  wb_pen_set_ink_color(pen0, 0x00FF00, 0.3);
  wb_pen_set_ink_color(pen1, 0x00FF00, 0.3);
  wb_pen_write(pen0, false);
  wb_pen_write(pen1, false);

  if (ts_webots_major_version() < 7)
    wb_robot_step(TIME_STEP);

  // check ds0 type
  int type = wb_distance_sensor_get_type(ds0);
  ts_assert_int_equal(type, WB_DISTANCE_SENSOR_INFRA_RED, "Wrong type returned.");

  // check ds0 range
  double min_range = wb_distance_sensor_get_min_value(ds0) / 10000.0;  // cf. DistanceSensor.lookupTable
  double max_range = wb_distance_sensor_get_max_value(ds0) / 10000.0;  // cf. DistanceSensor.lookupTable
  ts_assert_double_equal(min_range, 0.0, "Minumum range returned is wrong (%lf instead of 0.0)", min_range);
  ts_assert_double_equal(max_range, 0.2, "Maximum range returned is wrong (%lf instead of 0.2)", max_range);

  // FIRST STEP - distance to texture
  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  // check ds0
  value = wb_distance_sensor_get_value(ds0);  // no texture
  ts_assert_double_in_delta(value, 773.810, 0.001,
                            "Distance sensor 'ds0' doesn't return the right distance when hitting an object with white texture "
                            "(expected = %f, received = %f)",
                            773.810, value);

  // check ds1
  value = wb_distance_sensor_get_value(ds1);  // green texture
  ts_assert_double_in_delta(value, 1172.206506, 0.001,
                            "Distance sensor 'ds1' doesn't return the right distance when hitting an object with green texture "
                            " (expected = %f, received = %f)",
                            1172.206506, value);

  wb_pen_write(pen0, true);
  wb_pen_write(pen1, true);

  // SECOND STEP - distance to texture painted in green
  wb_robot_step(TIME_STEP);

  // check ds0
  value = wb_distance_sensor_get_value(ds0);
  ts_assert_double_in_delta(value, 1001.147620, 0.001,
                            "Distance sensor 'ds0' doesn't return the right distance when hitting a object with white texture "
                            "painted green (expected = %f, received = %f)",
                            1001.147620, value);

  // check ds1
  value = wb_distance_sensor_get_value(ds1);
  ts_assert_double_in_delta(value, 1448.139, 0.001,
                            "Distance sensor 'ds1' doesn't return the right distance when hitting a object with green texture "
                            "painted green (expected = %f, received = %f)",
                            1448.139, value);

  wb_pen_set_ink_color(pen0, 0xCC33FF, 0.3);
  wb_pen_set_ink_color(pen1, 0xCC33FF, 0.3);

  // THIRD STEP - distance to texture painted in green and violet
  wb_robot_step(TIME_STEP);

  // check ds0
  value = wb_distance_sensor_get_value(ds0);
  ts_assert_double_in_delta(value, 973.118797, 0.001,
                            "Distance sensor 'ds0' doesn't return the right distance when hitting an object with white texture "
                            "painted in violet (expected = %f, received = %f)",
                            973.118797, value);

  // check ds1
  value = wb_distance_sensor_get_value(ds1);
  ts_assert_double_in_delta(value, 1415.472520, 0.001,
                            "Distance sensor 'ds1' doesn't return the right distance when hitting an object with green texture "
                            "painted in violet (expected = %f, received = %f)",
                            1415.472520, value);

  wb_pen_set_ink_color(pen0, 0xFF0000, 0.7);
  wb_pen_set_ink_color(pen1, 0xFF0000, 0.7);

  // FORTH STEP - distance to texture painted in green, violet and red
  wb_robot_step(TIME_STEP);

  // check ds0
  value = wb_distance_sensor_get_value(ds0);
  ts_assert_double_in_delta(value, 824.821526, 0.001,
                            "Distance sensor 'ds0' doesn't return the right distance when hitting an object with white texture "
                            "painted in red (expected = %f, received = %f)",
                            824.821526, value);

  // check ds1
  value = wb_distance_sensor_get_value(ds1);
  ts_assert_double_in_delta(value, 1236.368597, 0.001,
                            "Distance sensor 'ds1' doesn't return the right distance when hitting an object with green texture "
                            "painted in red (expected = %f, received = %f)",
                            1236.368597, value);

  ts_send_success();
  return EXIT_SUCCESS;
}
