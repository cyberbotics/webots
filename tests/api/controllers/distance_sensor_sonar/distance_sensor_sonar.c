#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define N 4

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  struct {
    char name[10];
    double expected;
    WbDeviceTag tag;
  } us[N] = {{"ds0", 0.1}, {"ds1", 1.0}, {"ds2", 0.05}, {"ds3", 0.218207}};

  int i;
  for (i = 0; i < N; i++) {
    us[i].tag = wb_robot_get_device(us[i].name);
    wb_distance_sensor_enable(us[i].tag, TIME_STEP);
  }

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  for (i = 0; i < N; i++) {
    double value = wb_distance_sensor_get_value(us[i].tag);
    ts_assert_double_in_delta(value, us[i].expected, 0.00001,
                              "The distance sensor '%s' (sonar) doesn't return the right distance when hitting an object.",
                              us[i].name);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
