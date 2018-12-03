#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag ds = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(ds, TIME_STEP);

  wb_robot_step(TIME_STEP);

  double value = wb_distance_sensor_get_value(ds);
  ts_assert_double_in_delta(value, 500.0, 10.0, "Slot node in parameter is not loaded correctly.");

  ts_send_success();
  return EXIT_SUCCESS;
}
