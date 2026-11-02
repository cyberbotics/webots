#include <webots/plugins/robot_window/default.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  for (;;) {  // receive message sent by the robot window.
    wb_robot_step(TIME_STEP);
    const char *configure_message = wb_robot_wwi_receive_text();
    if (configure_message) {
      if (strcmp(configure_message, "configure") == 0) {
        wb_robot_wwi_send_text("test wwi functions from robot_window_html controller.");
        break;
      } else
        ts_send_error_and_exit("Wrong configure message received from the HTML robot-window: %s", configure_message);
    }
  }

  for (;;) {  // receive message sent by Webots.
    wb_robot_step(TIME_STEP);
    const char *test_message = wb_robot_wwi_receive_text();
    if (test_message) {
      if (strcmp(test_message, "Answer: test wwi functions from robot_window_html controller.") == 0)
        break;
      else
        ts_send_error_and_exit("Wrong test message received from the HTML robot-window: %s", test_message);
    }
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
