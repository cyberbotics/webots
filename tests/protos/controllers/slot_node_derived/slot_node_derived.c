#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // get and enable the acc
  WbDeviceTag acc1 = wb_robot_get_device("acc1");
  WbDeviceTag acc2 = wb_robot_get_device("acc2");

  wb_robot_step(TIME_STEP);

  // first accelerometer is in valid derived slot => the device should be accessible => the tag should not be 0
  ts_assert_int_not_equal(acc1, 0,
                          "Accelerometer 'acc1' is in a valid pair of slot and is therefore supposed to be added to the robot, "
                          "there is a problem due to the fact that the slot type is not defined in the derived proto but only "
                          "in the base proto.");

  // the second accelerometer is not in valid slot => the device should not be accessible => the tag should be 0
  ts_assert_int_equal(acc2, 0,
                      "Accelerometer 'acc2' is not in a valid pair of slot and is therefore not supposed to be added to the "
                      "robot, the default slot type has been applied to the derived proto instead of the one defined in the "
                      "base proto.");

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
