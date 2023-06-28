#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  double value = 0.0;
  ts_setup(argv[0]);  // give the controller args

  WbDeviceTag ds1 = wb_robot_get_device("ds1");
  WbDeviceTag ds2 = wb_robot_get_device("ds2");
  wb_distance_sensor_enable(ds1, TIME_STEP);
  wb_distance_sensor_enable(ds2, TIME_STEP);

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds1);
  ts_assert_double_is_bigger(value, 500, "Unexpected shape: default base PROTO Box shape shouldn't be created.");

  value = wb_distance_sensor_get_value(ds2);
  ts_assert_double_is_bigger(1000, value, "Unexpected shape: derived PROTO Cylinder shape should be created.");

  ts_send_success();
  return EXIT_SUCCESS;
}
