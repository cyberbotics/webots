#include <webots/radar.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

bool check_all_target_angle_is_correct(WbDeviceTag radar, double expected_angle) {
  int i = 0;
  int target_number = wb_radar_get_number_of_targets(radar);
  for (i = 0; i < target_number; ++i) {
    if (abs(wb_radar_get_targets(radar)[i].azimuth - expected_angle) > 0.0000001)
      return false;
  }
  return true;
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag radar1 = wb_robot_get_device("radar1");
  WbDeviceTag radar2 = wb_robot_get_device("radar2");
  WbDeviceTag radar3 = wb_robot_get_device("radar3");
  WbDeviceTag radar4 = wb_robot_get_device("radar4");

  wb_radar_enable(radar1, TIME_STEP);
  wb_radar_enable(radar2, TIME_STEP);
  wb_radar_enable(radar3, TIME_STEP);
  wb_radar_enable(radar4, TIME_STEP);

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  // check number of target
  ts_assert_int_equal(wb_radar_get_number_of_targets(radar1), 3,
                      "The radar 1 should see 3 targets because 3 targets are merged into one twice.");
  ts_assert_int_equal(wb_radar_get_number_of_targets(radar2), 4,
                      "The radar 2 should see 4 targets because 3 targets are merged into one and 2 other into one.");
  ts_assert_int_equal(wb_radar_get_number_of_targets(radar3), 7,
                      "The radar 3 should see all the 7 targets because no targets should be merged.");
  ts_assert_int_equal(
    wb_radar_get_number_of_targets(radar4), 5,
    "The radar 4 should see 5 targets, only 2 tagets are merged into one twice because of the very small cell speed.");

  // check angle of merged targets is correct
  ts_assert_boolean_equal(check_all_target_angle_is_correct(radar1, 0.0),
                          "After merging all the targets of radar 1 should be aligned in the center (i.e. angle should be 0)");
  ts_assert_boolean_equal(check_all_target_angle_is_correct(radar2, 0.0),
                          "After merging all the targets of radar 2 should be aligned in the center (i.e. angle should be 0)");

  ts_send_success();
  return EXIT_SUCCESS;
}
