#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();

  wb_robot_step(TIME_STEP);

  wb_supervisor_node_remove(wb_supervisor_node_get_self());

  wb_robot_step(TIME_STEP);

  return EXIT_SUCCESS;
}
