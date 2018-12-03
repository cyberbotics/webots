#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <stdio.h>

#define TIME_STEP 64
#define NODE_NUMBER 8

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef nodes[NODE_NUMBER];
  double heights[NODE_NUMBER];

  int i;
  for (i = 0; i < NODE_NUMBER; ++i) {
    char name[8];
    sprintf(name, "SOLID%d", i);
    nodes[i] = wb_supervisor_node_get_from_def(name);
    heights[i] = wb_supervisor_node_get_position(nodes[i])[1];
  }

  wb_robot_step(100 * TIME_STEP);

  for (i = 0; i < NODE_NUMBER; ++i)
    ts_assert_double_in_delta(wb_supervisor_node_get_position(nodes[i])[1], heights[i], 0.01,
                              "Height of Solid '%d' is not correct (%lf != %lf).", i,
                              wb_supervisor_node_get_position(nodes[i])[1], heights[i]);

  ts_send_success();
  return EXIT_SUCCESS;
}
