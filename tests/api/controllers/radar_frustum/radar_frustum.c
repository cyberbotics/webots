#include <webots/radar.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define SOLID_NUMBER 6

static const char *solid_names[SOLID_NUMBER] = {"VISIBLE_SPHERE",  "NOT_VISIBLE_SPHERE", "VISIBLE_BOX",
                                                "NOT_VISIBLE_BOX", "VISIBLE_CAPSULE",    "NOT_VISIBLE_CAPSULE"};

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  int i, target_number;

  WbDeviceTag radar = wb_robot_get_device("radar");
  wb_radar_enable(radar, 3 * TIME_STEP);

  wb_robot_step(TIME_STEP);

  target_number = wb_radar_get_number_of_targets(radar);
  ts_assert_int_equal(target_number, 0, "The radar should see 0 targets and not %d after 1 step from enable call.",
                      target_number);

  wb_robot_step(TIME_STEP);

  target_number = wb_radar_get_number_of_targets(radar);
  ts_assert_int_equal(target_number, 0, "The radar should see 0 targets and not %d after 2 steps from enable call.",
                      target_number);

  wb_robot_step(TIME_STEP);

  for (i = 0; i < SOLID_NUMBER; ++i) {
    target_number = wb_radar_get_number_of_targets(radar);
    ts_assert_int_equal(target_number, (SOLID_NUMBER - i) / 2,
                        "The radar should see %d targets and not %d before removing '%s'.", (SOLID_NUMBER - i) / 2,
                        target_number, solid_names[i]);
    WbNodeRef node = wb_supervisor_node_get_from_def(solid_names[i]);
    wb_supervisor_node_remove(node);
    wb_robot_step(2 * TIME_STEP);
    target_number = wb_radar_get_number_of_targets(radar);
    ts_assert_int_equal(
      target_number, (SOLID_NUMBER - i) / 2,
      "The radar should see %d targets and not %d after removing '%s' and before the sampling period elapsed.",
      (SOLID_NUMBER - i) / 2, target_number, solid_names[i]);
    wb_robot_step(TIME_STEP);
    target_number = wb_radar_get_number_of_targets(radar);
    ts_assert_int_equal(target_number, (SOLID_NUMBER - i - 1) / 2,
                        "The radar should see %d targets and not %d after removing '%s'.", (SOLID_NUMBER - i - 1) / 2,
                        target_number, solid_names[i]);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
