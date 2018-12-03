#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

static void step(int time_step) {
  if (wb_robot_step(time_step) == -1)
    ts_assert_boolean_equal(0, "Controller stopped before having being able to conclude.");
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = (int)wb_robot_get_basic_time_step();

  const char *ts_names[4] = {"tsA", "tsB", "tsC", "tsD"};
  WbDeviceTag ts[4];
  int i;
  for (i = 0; i < 4; ++i) {
    ts[i] = wb_robot_get_device(ts_names[i]);
    wb_touch_sensor_enable(ts[i], time_step);
  }

  step(time_step);

  ts_assert_boolean_equal(wb_touch_sensor_get_value(ts[0]) == false, "Collision detected on tsA too soon.");
  ts_assert_boolean_equal(wb_touch_sensor_get_value(ts[1]) == false, "Collision detected on tsB too soon.");
  ts_assert_boolean_equal(wb_touch_sensor_get_value(ts[2]) == false, "Collision detected on tsC too soon.");
  ts_assert_boolean_equal(wb_touch_sensor_get_value(ts[3]) == false, "Collision detected on tsD too soon.");

  int counter = 0;

  // check ball B is touching first
  while (true) {
    step(time_step);
    ts_assert_boolean_equal(counter++ < 1000, "Ball B is not touching the ground.");
    ts_assert_boolean_equal(wb_touch_sensor_get_value(ts[0]) == false && wb_touch_sensor_get_value(ts[2]) == false &&
                              wb_touch_sensor_get_value(ts[3]) == false,
                            "A collision has occured before or at the same time than BALL B.");
    if (wb_touch_sensor_get_value(ts[1]) == true)
      break;
  }

  // check ball A and C are touching secondly together the ground
  while (true) {
    step(time_step);
    ts_assert_boolean_equal(counter++ < 1000, "Ball A or C are not touching the ground.");
    ts_assert_boolean_equal(wb_touch_sensor_get_value(ts[3]) == false,
                            "A collision has occured before or at the same time than BALL A and C.");
    if (wb_touch_sensor_get_value(ts[0]) == true || wb_touch_sensor_get_value(ts[2]) == true) {
      ts_assert_boolean_equal(wb_touch_sensor_get_value(ts[0]) == true && wb_touch_sensor_get_value(ts[2]) == true,
                              "Balls A and C should arrive at the same time.");
      break;
    }
  }

  // check ball D is touching last
  while (true) {
    step(time_step);
    ts_assert_boolean_equal(counter++ < 1000, "Ball D is not touching the ground.");
    if (wb_touch_sensor_get_value(ts[3]) == true)
      break;
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
