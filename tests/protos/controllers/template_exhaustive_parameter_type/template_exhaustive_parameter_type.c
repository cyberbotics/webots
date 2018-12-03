#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  const char *template_result = wb_robot_get_model();
  ts_assert_string_equal(template_result, "success", "Unexpected template result: %s", template_result);

  const char *custom_data = wb_robot_get_custom_data();
  ts_assert_string_equal(custom_data, "custom data", "Unexpected custom data result: %s", custom_data);

  ts_send_success();
  return EXIT_SUCCESS;
}
