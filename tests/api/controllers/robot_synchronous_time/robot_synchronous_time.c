#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  double r;
  double e = 0.0;
  int i, v;

  ts_setup(argv[0]);

  ts_assert_boolean_equal(wb_robot_get_synchronization(), "Cannot get the synchronization flag");

  // perform regular 20 steps and check time
  for (i = 0; i < 20; i++) {
    r = wb_robot_get_time();
    // printf("Time = %f\n", r);
    ts_assert_double_in_delta(r, e, 0.000001, "Regular time test: The time measured is wrong. Expected %f. Received %f.", e, r);
    v = wb_robot_step(TIME_STEP);
    // printf("Step = %d\n", v);
    ts_assert_int_equal(v, 0, "Regular time test: The return value of wb_robot_step is wrong. Expected 0. Received %d.", v);
    e += 0.001 * TIME_STEP;
  }

  // perform 20 steps multiple of time step and check time
  for (i = 0; i < 20; i++) {
    r = wb_robot_get_time();
    // printf("Time = %f\n", r);
    ts_assert_double_in_delta(r, e, 0.000001, "Multiple time test: The time measured is wrong. Expected %f. Received %f.", e,
                              r);
    v = wb_robot_step(TIME_STEP * i);
    // printf("Step = %d\n", v);
    ts_assert_int_equal(v, 0, "Regular time test: The return value of wb_robot_step is wrong. Expected 0. Received %d.", v);
    e += 0.001 * TIME_STEP * i;
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
