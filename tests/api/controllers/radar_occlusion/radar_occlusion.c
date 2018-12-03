#include <webots/radar.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag radar_with_occlusion = wb_robot_get_device("radar_with_occlusion");
  WbDeviceTag radar_without_occlusion = wb_robot_get_device("radar_without_occlusion");

  wb_radar_enable(radar_with_occlusion, TIME_STEP);
  wb_radar_enable(radar_without_occlusion, TIME_STEP);

  wb_robot_step(TIME_STEP);

  ts_assert_int_equal(wb_radar_get_number_of_targets(radar_with_occlusion), 1,
                      "The radar with occlusion should see only 1 target because the other ones are occluded.");
  ts_assert_int_equal(wb_radar_get_number_of_targets(radar_without_occlusion), 3,
                      "The radar without occlusion should see 3 targets.");

  WbNodeRef front_target = wb_supervisor_node_get_from_def("FRONT_TARGET");
  wb_supervisor_node_remove(front_target);

  wb_robot_step(TIME_STEP);

  ts_assert_int_equal(wb_radar_get_number_of_targets(radar_with_occlusion), 2,
                      "The radar with occlusion should see only 2 targets.");
  ts_assert_int_equal(wb_radar_get_number_of_targets(radar_without_occlusion), 2,
                      "The radar without occlusion should see 2 targets.");

  ts_send_success();
  return EXIT_SUCCESS;
}
