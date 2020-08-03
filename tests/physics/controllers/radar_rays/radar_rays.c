#include <webots/motor.h>
#include <webots/radar.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  int count = 0;
  int expected_count = 0;
  int time_step = wb_robot_get_basic_time_step();
  const char *robot_name = wb_robot_get_name();
  bool dynamic = true;
  if (strcmp(robot_name, "static") == 0)
    dynamic = false;

  WbDeviceTag rs = wb_robot_get_device("radar static");
  WbDeviceTag rd = wb_robot_get_device("radar dynamic");
  WbDeviceTag ros = wb_robot_get_device("radar occlusion static");
  WbDeviceTag rod = wb_robot_get_device("radar occlusion dynamic");
  wb_radar_enable(rs, time_step);
  wb_radar_enable(rd, time_step);
  wb_radar_enable(ros, time_step);
  wb_radar_enable(rod, time_step);

  // stabilize the system
  wb_robot_step(3 * time_step);

  if (dynamic) {
    // move sensors during ODE physics step
    WbDeviceTag left_motor = wb_robot_get_device("left motor");
    WbDeviceTag right_motor = wb_robot_get_device("right motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 100);
    wb_motor_set_velocity(right_motor, 100);
  }

  // without occlusion (no ODE rays)
  count = wb_radar_get_number_of_targets(rs);
  ts_assert_int_equal(count, 1,
                      "Radar without occlusion detects the wrong number of static objects "
                      "at check 1: expected %d, detected %d",
                      1, count);
  count = wb_radar_get_number_of_targets(rd);
  ts_assert_int_equal(count, 1,
                      "Radar without occlusion detects the wrong number of dynamic objects "
                      "at check 1: expected %d, detected %d",
                      1, count);

  // with occlusion (using ODE rays)
  count = wb_radar_get_number_of_targets(ros);
  ts_assert_int_equal(count, 1,
                      "Radar with occlusion detects the wrong number of static objects "
                      "at check 1: expected %d, detected %d",
                      1, count);
  count = wb_radar_get_number_of_targets(rod);
  ts_assert_int_equal(count, 1,
                      "Radar with occlusion detects the wrong number of dynamic objects "
                      "at check 1: expected %d, detected %d",
                      1, count);

  wb_robot_step(time_step);

  // without occlusion (no ODE rays)
  if (dynamic)
    expected_count = 0;
  else
    expected_count = 1;
  count = wb_radar_get_number_of_targets(rs);
  ts_assert_int_equal(count, expected_count,
                      "Radar without occlusion detects the wrong number of static objects "
                      "at check 2: expected %d, detected %d",
                      expected_count, count);

  count = wb_radar_get_number_of_targets(rd);
  ts_assert_int_equal(count, expected_count,
                      "Radar without occlusion detects the wrong number of dynamic objects "
                      "at check 2: expected %d, detected %d",
                      expected_count, count);

  // with occlusion (using ODE rays)
  count = wb_radar_get_number_of_targets(ros);
  ts_assert_int_equal(count, expected_count,
                      "Radar with occlusion detects the wrong number of static objects "
                      "at check 2: expected %d, detected %d",
                      expected_count, count);
  count = wb_radar_get_number_of_targets(rod);
  ts_assert_int_equal(count, expected_count,
                      "Radar with occlusion detects the wrong number of dynamic objects "
                      "at check 2: expected %d, detected %d",
                      expected_count, count);

  wb_robot_step(time_step);

  ts_send_success();
  return EXIT_SUCCESS;
}
