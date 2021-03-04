#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16
#define TEST_DURATION 10.0f
#define SPEED 0.2f

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // wb_robot_init();
  WbNodeRef pendulum_bob = wb_supervisor_node_get_from_def("PENDULUM_BOB");

  // TEST #1: by switching direction BEFORE the backlash limit is reached, the endPoint shouldn't move
  printf("SUPERVISOR Test #1: Start.\n");
  const double expected[] = {0.0, -0.4, 0.0};
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    // control
    const double *values = wb_supervisor_node_get_position(pendulum_bob);
    ts_assert_vec3_in_delta(values[0], values[1], values[2], expected[0], expected[1], expected[2], 0.0000001,
                            "The pendulum moves despite being inside the backlash range."
                            " (expected = {%f, %f, %f}, received = {%f, %f, %f})",
                            expected[0], expected[1], expected[2], values[0], values[1], values[2]);
  }
  printf("SUPERVISOR Test #1: Done.\n");

  wb_supervisor_simulation_reset_physics();

  // TEST #2: by switching direction AFTER the backlash limit is reached, the endPoint shouldn't move
  printf("SUPERVISOR Test #2: Start.\n");

  double xmax = -INFINITY;
  double xmin = INFINITY;
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < 2 * TEST_DURATION) {
    const double *values = wb_supervisor_node_get_position(pendulum_bob);
    if (values[0] < xmin)
      xmin = values[0];
    if (values[0] > xmax)
      xmax = values[0];
  }

  printf("SUPERVISOR Test #2: Done.\n");
  // Test that it moved ...
  double exp = 0.0421;
  double delta = fabs(xmax) - exp;
  ts_assert_double_in_delta(delta, 0.0, 0.0001, "Positive swing overshoot or undershoot. (expected = %.10f, value = %.10f", exp,
                            fabs(xmax));

  delta = fabs(xmin) - exp;
  ts_assert_double_in_delta(delta, 0.0, 0.0001, "Negative swing overshoot or undershoot. (expected = %.10f, value = %.10f", exp,
                            fabs(xmin));

  // Test that the swings are sufficiently symmetric
  delta = fabs(xmax) - fabs(xmin);
  ts_assert_double_in_delta(delta, 0.0, 0.00005, "The swings differ by too much. (values: %.10f // %.10f)", xmin, xmax);

  ts_send_success();
  return EXIT_SUCCESS;
}
