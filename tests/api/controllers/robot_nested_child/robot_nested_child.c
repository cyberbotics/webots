#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  ts_assert_int_equal(wb_robot_get_device("gyro"), 0, "The parent robot's gyro device should not be found by the child robot");

  ts_send_success();
  return EXIT_SUCCESS;
}
