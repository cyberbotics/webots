#include <webots/radar.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  const double targetVelocity[6] = {0.0, 0.0, -1.5, 0.0, 0.0, 0.0};
  const double supervisorVelocity[6] = {0.0, 0.0, -0.75, 0.0, 0.0, 0.0};

  ts_setup(argv[0]);

  WbDeviceTag radar = wb_robot_get_device("radar");
  WbNodeRef radarTarget = wb_supervisor_node_get_from_def("RADAR_TARGET");
  WbNodeRef supervisor = wb_supervisor_node_get_self();

  wb_radar_enable(radar, TIME_STEP);

  wb_robot_step(TIME_STEP);
  int target_number = wb_radar_get_number_of_targets(radar);
  ts_assert_int_equal(target_number, 1, "The radar should see 1 target and not %d.", target_number);

  // CASE 1: supervisor still, target moving
  wb_supervisor_node_set_velocity(radarTarget, targetVelocity);

  // step to allow objects to move
  wb_robot_step(TIME_STEP);

  const WbRadarTarget *target = wb_radar_get_targets(radar);
  ts_assert_double_in_delta(target[0].speed, 1.5, 0.001, "The initial speed should be 1.5");

  // CASE 2: supervisor still, target moving
  wb_supervisor_node_set_velocity(supervisor, supervisorVelocity);

  // step to allow objects to move
  wb_robot_step(TIME_STEP);

  const WbRadarTarget *newTarget = wb_radar_get_targets(radar);
  ts_assert_double_in_delta(newTarget[0].speed, 0.75, 0.001, "The secondary speed should be 0.75");

  ts_send_success();
  return EXIT_SUCCESS;

  return 0;
}
