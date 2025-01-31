#include <stdio.h>
#include <webots/brake.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  wb_robot_step(TIME_STEP);
  // Test that a world with a Muscle node will load. See issue #6659.
  wb_supervisor_world_load("../../worlds/supervisor_set_hinge_position_dynamic.wbt");

  ts_send_success();
  return EXIT_SUCCESS;
}
