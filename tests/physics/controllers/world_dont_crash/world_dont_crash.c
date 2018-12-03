#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  ts_assert_boolean_equal(argc == 2, "world_dont_crash badly used. world name expected as argument.");

  while (wb_robot_step(TIME_STEP) != -1) {
    if (wb_robot_get_time() > 5.0)
      break;
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
