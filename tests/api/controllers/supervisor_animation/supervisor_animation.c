#include <unistd.h>
#include <webots/supervisor.h>
#include "../../../lib/file_utils.h"
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  remove("animation.html");
  remove("animation.json");
  remove("animation.x3d");
  remove("animation.css");

  bool success;

  success = wb_supervisor_animation_start_recording("animation.html");
  ts_assert_boolean_equal(success, "Animation start recording has failed.");

  int i;
  for (i = 0; i < 20; i++)
    wb_robot_step(TIME_STEP);

  success = wb_supervisor_animation_stop_recording();
  ts_assert_boolean_equal(success, "Animation stop recording has failed.");

  ts_assert_boolean_equal(file_exists("animation.html"), "animation.html not created.");
  ts_assert_boolean_equal(file_exists("animation.json"), "animation.json not created.");
  ts_assert_boolean_equal(file_exists("animation.x3d"), "animation.x3d not created.");
  ts_assert_boolean_equal(file_exists("animation.css"), "animation.css not created.");

  ts_send_success();
  return EXIT_SUCCESS;
}
