/*
 * Description:  Test supervisor function that resets the robot controller
                 with a display without crashing Webots.
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef mybot = wb_supervisor_node_get_from_def("MYBOT");

  wb_robot_step(10 * TIME_STEP);

  wb_supervisor_node_restart_controller(mybot);

  wb_robot_step(10 * TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
