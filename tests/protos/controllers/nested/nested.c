#include <webots/camera.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);  // give the controller args

  WbDeviceTag top_camera = wb_robot_get_device("top camera");
  WbDeviceTag side_camera = wb_robot_get_device("side camera");

  ts_assert_int_equal(wb_camera_get_width(top_camera), 1, "Wrong top camera width");
  ts_assert_int_equal(wb_camera_get_height(top_camera), 1, "Wrong top camera height");
  ts_assert_int_equal(wb_camera_get_width(side_camera), 1, "Wrong side camera width");
  ts_assert_int_equal(wb_camera_get_height(side_camera), 1, "Wrong side camera height");

  wb_camera_enable(top_camera, TIME_STEP);
  wb_camera_enable(side_camera, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const unsigned char *top_values = wb_camera_get_image(top_camera);
  ts_assert_color_in_delta(wb_camera_image_get_red(top_values, 1, 0, 0), wb_camera_image_get_green(top_values, 1, 0, 0),
                           wb_camera_image_get_blue(top_values, 1, 0, 0), 255, 255, 255,  // background
                           1, "Unexpected camera top color. The top of the capsule should be invisible");

  const unsigned char *side_values = wb_camera_get_image(side_camera);
  ts_assert_color_in_delta(wb_camera_image_get_red(side_values, 1, 0, 0), wb_camera_image_get_green(side_values, 1, 0, 0),
                           wb_camera_image_get_blue(side_values, 1, 0, 0), 0, 0, 0,  // capsule
                           1, "Unexpected camera side color. The side of the capsule should be visible");

  ts_send_success();
  return EXIT_SUCCESS;
}
