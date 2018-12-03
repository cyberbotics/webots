#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // get and enable the accelerometer
  WbDeviceTag acc1 = wb_robot_get_device("acc1");
  WbDeviceTag acc2 = wb_robot_get_device("acc2");
  WbDeviceTag acc3 = wb_robot_get_device("acc3");
  WbDeviceTag acc4 = wb_robot_get_device("acc4");
  WbDeviceTag acc5 = wb_robot_get_device("acc5");
  WbDeviceTag acc6 = wb_robot_get_device("acc6");
  WbDeviceTag acc7 = wb_robot_get_device("acc7");
  WbDeviceTag acc8 = wb_robot_get_device("acc8");

  wb_robot_step(TIME_STEP);

  // first six accelerometers are in valid slot => the device should be accessible => the tag should not be 0
  ts_assert_int_not_equal(acc1, 0,
                          "Accelerometer 'acc1' is in a valid pair of slot and is therefore supposed to be added to the robot");
  ts_assert_int_not_equal(acc2, 0,
                          "Accelerometer 'acc2' is in a valid pair of slot including some gender test and is therefore "
                          "supposed to be added to the robot");
  ts_assert_int_not_equal(acc3, 0,
                          "Accelerometer 'acc3' is in a valid pair of slot and is therefore supposed to be added to the robot");
  ts_assert_int_not_equal(
    acc4, 0, "Accelerometer 'acc4' is in the 'children' field of 'acc3' and is therefore supposed to be added to the robot");
  ts_assert_int_not_equal(
    acc5, 0,
    "Accelerometer 'acc5' is in a valid pair of slot nodes with empty type and is therefore supposed to be added to the robot");
  ts_assert_int_not_equal(acc6, 0,
                          "Accelerometer 'acc6' is in a valid pair of slot nodes whose type contains spaces and is therefore "
                          "supposed to be added to the robot");

  // the last two accelerometers are not in valid slot => the device should not be accessible => the tag should be 0
  ts_assert_int_equal(acc7, 0,
                      "Accelerometer 'acc7' is in a valid pair of slot but the gender are the same and is therefore not "
                      "supposed to be added to the robot");
  ts_assert_int_equal(
    acc8, 0, "Accelerometer 'acc8' is not in a valid pair of slot and is therefore not supposed to be added to the robot");

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
