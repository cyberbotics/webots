#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16
#define TEST_DURATION 10.0f

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef pendulum_bob = wb_supervisor_node_get_from_def("PENDULUM_BOB");

  // TEST #1: by switching direction BEFORE the backlash limit is reached, the endPoint shouldn't move
  const double expected[] = {0.0, -0.4, 0.0};

  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    const double *values = wb_supervisor_node_get_position(pendulum_bob);
    ts_assert_vec3_in_delta(values[0], values[1], values[2], expected[0], expected[1], expected[2], 0.0000001,
                            "The pendulum moves despite being inside the backlash range."
                            " (expected = {%f, %f, %f}, measured = {%f, %f, %f})",
                            expected[0], expected[1], expected[2], values[0], values[1], values[2]);
  }

  wb_supervisor_simulation_reset_physics();

  // TEST #2: by switching direction AFTER the backlash limit is reached, the endPoint should move
  double x_max = -INFINITY;
  double x_min = INFINITY;

  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < 2 * TEST_DURATION) {
    const double *values = wb_supervisor_node_get_position(pendulum_bob);
    if (values[0] < x_min)
      x_min = values[0];
    if (values[0] > x_max)
      x_max = values[0];
  }

  // Test that it moved
  ts_assert_double_in_delta(x_max, 0.0421, 0.0001, "Positive swing overshoot or undershoot. (expected = 0.0421, measured = %f)",
                            x_max);

  ts_assert_double_in_delta(x_min, -0.0421, 0.0001,
                            "Negative swing overshoot or undershoot. (expected = -0.0421, measured = %f)", x_min);

  // Test that the swings are sufficiently symmetric
  ts_assert_double_in_delta(fabs(x_max) - fabs(x_min), 0.0, 0.00005, "The swings differ by too much. (x_min = %f, x_max = %f)",
                            x_min, x_max);

  ts_send_success();
  return EXIT_SUCCESS;
}
