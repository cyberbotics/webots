#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // get acclerometer
  WbDeviceTag acc1 = wb_robot_get_device("acc1");

  wb_robot_step(TIME_STEP);

  // The accelerometer is in valid slot => the device should be accessible => the tag should not be 0
  ts_assert_int_not_equal(acc1, 0,
                          "Accelerometer 'acc1' is in a valid pair of slot and is therefore supposed to be added to the robot");

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
