/*
 * Description:  Test supervisor function that resets its own controller
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();
  double time = wb_robot_get_time();

  if (time < 3.0) {
    WbNodeRef supervisor_node = wb_supervisor_node_get_self();
    wb_robot_step(TIME_STEP);

    // reset controller
    wb_supervisor_node_restart_controller(supervisor_node);
    while (wb_robot_step(TIME_STEP) != -1) {
    }
  } else {
    ts_setup(argv[0]);
    ts_send_success();
  }

  return EXIT_SUCCESS;
}
