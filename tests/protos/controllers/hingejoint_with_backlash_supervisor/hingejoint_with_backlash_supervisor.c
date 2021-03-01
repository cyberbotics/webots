#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);
 
  int time_step = wb_robot_get_basic_time_step();

  WbNodeRef pendulum_robot = wb_supervisor_node_get_from_def("PENDULUM_ROBOT");
  while (wb_robot_step(time_step) != -1.0 && wb_robot_get_time() < 5.0) {
  }

  ts_send_success();
  return EXIT_SUCCESS;
}