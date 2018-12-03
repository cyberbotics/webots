#include <stdio.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define JOINTS_COUNT 6
#define POSITION_COUNT 4

static const char *JOINT_NAMES[4] = {"Slider", "Hinge", "Ball"};

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = (int)wb_robot_get_basic_time_step();
  int i = 0;
  WbFieldRef *positionFields = malloc(POSITION_COUNT * sizeof(WbFieldRef));
  char name[32];
  for (i = 0; i < POSITION_COUNT; ++i) {
    sprintf(name, "JOINT_PARAMETER_%d", i + 1);
    WbNodeRef node = wb_supervisor_node_get_from_def(name);
    positionFields[i] = wb_supervisor_node_get_field(node, "position");
  }
  WbFieldRef *translationFields = malloc(JOINTS_COUNT * sizeof(WbFieldRef));
  for (i = 0; i < JOINTS_COUNT; ++i) {
    sprintf(name, "END_POINT_SOLID_%d", i + 1);
    WbNodeRef node = wb_supervisor_node_get_from_def(name);
    translationFields[i] = wb_supervisor_node_get_field(node, "translation");
  }

  // initial position is 0 for all the joints

  i = 0;
  int step = 1;
  double position = 0.0;
  double position2 = 0.0;
  const double *translation = NULL;
  const double *translation2 = NULL;
  while (wb_robot_step(time_step) != -1) {
    for (i = 0; i < POSITION_COUNT; i += 2) {
      position = wb_supervisor_field_get_sf_float(positionFields[i]);
      position2 = wb_supervisor_field_get_sf_float(positionFields[i + 1]);
      ts_assert_double_in_delta(position, position2, 0.01,
                                "%s joint with and without parent body doesn't behave the same way at step %d.",
                                JOINT_NAMES[i / 2], step);
    }
    // check end point solid translation
    for (i = 0; i < JOINTS_COUNT; i += 2) {
      translation = wb_supervisor_field_get_sf_vec3f(translationFields[i]);
      translation2 = wb_supervisor_field_get_sf_vec3f(translationFields[i + 1]);
      ts_assert_doubles_in_delta(
        3, translation, translation2, 0.01,
        "%s joint with and without parent body doesn't have the same endPoint solid translation at step %d.",
        JOINT_NAMES[i / 2], step);
    }
    if (step > 60)
      break;
    step++;
  }
  while (wb_robot_step(time_step) != -1) {
    if (step > 70)
      break;
    step++;
  }

  for (i = 0; i < POSITION_COUNT; ++i) {
    position = wb_supervisor_field_get_sf_float(positionFields[i]);
    ts_assert_double_in_delta(position, 0.0, 0.01, "%s joint %d didn't return to initial position: expected %f, current %f.",
                              JOINT_NAMES[i / 2], 0.0, position);
  }

  free(positionFields);
  free(translationFields);
  ts_send_success();
  return EXIT_SUCCESS;
}
