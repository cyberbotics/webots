#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  wb_robot_step(atoi(argv[1]));

  ts_send_success();
  return EXIT_SUCCESS;
}
