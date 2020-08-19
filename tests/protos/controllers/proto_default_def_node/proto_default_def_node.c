#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);  // give the controller args

  WbDeviceTag box_sensor = wb_robot_get_device("box sensor");

  wb_distance_sensor_enable(box_sensor, TIME_STEP);

  wb_robot_step(TIME_STEP);

  // test default DEF BOX parameter is found at load
  double value = wb_distance_sensor_get_value(box_sensor);
  ts_assert_double_in_delta(value, 300.0, 0.1, "Default DEF node not found at load.");

  ts_send_success();
  return EXIT_SUCCESS;
}
