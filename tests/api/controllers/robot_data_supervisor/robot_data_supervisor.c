#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  srand(time(NULL));
  int i;

  wb_robot_step(TIME_STEP);

  WbNodeRef robot_node;
  robot_node = wb_supervisor_node_get_from_def("ROBOT");
  WbFieldRef custom_data_field;
  custom_data_field = wb_supervisor_node_get_field(robot_node, "customData");

  // set the formula
  char buffer[8];
  unsigned int a = rand() % 100;
  unsigned int b = rand() % 100;
  sprintf(buffer, "%u + %u", a, b);
  wb_supervisor_field_set_sf_string(custom_data_field, buffer);

  // wait a few steps
  for (i = 0; i < 10; ++i)
    wb_robot_step(TIME_STEP);

  // get result
  int expected_result = a + b;
  int result = 0;
  sscanf(wb_supervisor_field_get_sf_string(custom_data_field), "%d", &result);

  ts_assert_int_equal(result, expected_result, "Result is wrong, expected %d + %d = %d instead of %d", a, b, a + b, result);

  ts_send_success();
  return EXIT_SUCCESS;
}
