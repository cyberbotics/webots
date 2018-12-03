#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // get and enable the acc
  WbDeviceTag ds_static1 = wb_robot_get_device("static 1");
  WbDeviceTag ds_static2 = wb_robot_get_device("static 2");
  WbDeviceTag ds_non_static1 = wb_robot_get_device("non static 1");
  WbDeviceTag ds_non_static2 = wb_robot_get_device("non static 2");

  wb_distance_sensor_enable(ds_static1, TIME_STEP);
  wb_distance_sensor_enable(ds_static2, TIME_STEP);
  wb_distance_sensor_enable(ds_non_static1, TIME_STEP);
  wb_distance_sensor_enable(ds_non_static2, TIME_STEP);

  wb_robot_step(TIME_STEP);

  // check that both 'static' PROTO nodes are equivalent
  ts_assert_double_equal(wb_distance_sensor_get_value(ds_static1), wb_distance_sensor_get_value(ds_static2),
                         "Static procedural PROTO nodes should have the same size (%lf != %lf)",
                         wb_distance_sensor_get_value(ds_static1), wb_distance_sensor_get_value(ds_static2));

  // check that both 'non-static' PROTO nodes are not equivalent
  ts_assert_double_not_equal(wb_distance_sensor_get_value(ds_non_static1), wb_distance_sensor_get_value(ds_non_static2),
                             "Non-static procedural PROTO nodes should have different sizes (%lf = %lf)",
                             wb_distance_sensor_get_value(ds_non_static1), wb_distance_sensor_get_value(ds_non_static2));

  // Make sure static and non-static PROTO nodes are not equivalent
  ts_assert_double_not_equal(wb_distance_sensor_get_value(ds_static1), wb_distance_sensor_get_value(ds_non_static2),
                             "Non-static and static procedural PROTO nodes should have different sizes (%lf = %lf)",
                             wb_distance_sensor_get_value(ds_static1), wb_distance_sensor_get_value(ds_non_static2));

  ts_send_success();
  return EXIT_SUCCESS;
}
