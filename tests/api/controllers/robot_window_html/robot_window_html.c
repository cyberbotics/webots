#include <webots/plugins/robot_window/default.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  bool configured = false, received = false;
  ts_setup(argv[0]);

  while (!configured) {  // receive message sent by the robot window.
    const char *configure_message = wb_robot_wwi_receive_text();
    wb_robot_step(TIME_STEP);
    while (message) {
      if (strcmp(configure_message, "configure") == 0) {
        configured = true;
        wb_robot_wwi_send_text("test wwi functions from complete_test controller.");
      } else
        ts_send_error_and_exit("Wrong configure message received from the HTML robot-window: %s", configure_message);

      configure_message = wb_robot_wwi_receive_text();
    }
  }

  while (!received) {  // receive message sent by Webots.
    const char *test_message = wb_robot_wwi_receive_text();
    wb_robot_step(TIME_STEP);
    while (message) {
      if (strcmp(test_message, "Answer: test wwi functions from complete_test controller.") == 0)
        received = true;
      else
        ts_send_error_and_exit("Wrong test message received from the HTML robot-window: %s", test_message);

      test_message = wb_robot_wwi_receive_text();
    }
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
