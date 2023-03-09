#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

void check_existence(const char *defName) {
  WbNodeRef node = wb_supervisor_node_get_from_def(defName);
  ts_assert_pointer_not_null(node, "Node '%s' should exist but does not.", defName);
  wb_robot_step(TIME_STEP);
}

int main(int argc, const char *argv[]) {
  ts_setup(argv[0]);

  wb_robot_step(TIME_STEP);

  check_existence("TEXTURED_BACKGROUND");
  check_existence("TEXTURED_BACKGROUND_LIGHT");
  check_existence("ARENA");
  check_existence("SPOT");
  check_existence("STRAIGHT_STAIRS");
  check_existence("WOODEN_BOX");
  check_existence("PLASTIC_CRATE");
  check_existence("TRAFFIC_CONE");
  check_existence("OIL_BARREL");
  check_existence("CARDBOARD_BOX");
  check_existence("PLATFORM_CART");
  check_existence("WOODEN_PALLET_STACK_1");
  check_existence("WOODEN_PALLET_STACK_2");
  check_existence("RUBBER_DUCK");

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
