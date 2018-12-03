#include <webots/robot.h>
#include <webots/utils/default_robot_window.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  wb_robot_step(TIME_STEP);

  wb_robot_wwi_send_text("test wwi functions from complete_test controller.");

  wb_robot_step(TIME_STEP);

  ts_assert_string_equal(wb_robot_wwi_receive_text(), "Answer: test wwi functions from complete_test controller.",
                         "Wrong message received from the HTML robot-window.");

  ts_send_success();
  return EXIT_SUCCESS;
}
