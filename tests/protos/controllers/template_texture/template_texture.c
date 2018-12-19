#include <webots/camera.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);  // give the controller args

  WbDeviceTag camera1 = wb_robot_get_device("camera1");
  WbDeviceTag camera2 = wb_robot_get_device("camera2");

  ts_assert_int_equal(wb_camera_get_width(camera1), 1, "Wrong camera1 width");
  ts_assert_int_equal(wb_camera_get_height(camera1), 1, "Wrong camera1 height");
  ts_assert_int_equal(wb_camera_get_width(camera2), 1, "Wrong camera2 width");
  ts_assert_int_equal(wb_camera_get_height(camera2), 1, "Wrong camera2 height");

  wb_camera_enable(camera1, TIME_STEP);
  wb_camera_enable(camera2, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const unsigned char *image1 = wb_camera_get_image(camera1);
  ts_assert_color_in_delta(wb_camera_image_get_red(image1, 1, 0, 0), wb_camera_image_get_green(image1, 1, 0, 0),
                           wb_camera_image_get_blue(image1, 1, 0, 0), 0, 207, 0,  // TemplateTexture PROTO plane
                           1, "Unexpected camera1 color. The TemplateTexture PROTO should produce a green plane.");

  const unsigned char *image2 = wb_camera_get_image(camera2);
  ts_assert_color_in_delta(wb_camera_image_get_red(image2, 1, 0, 0), wb_camera_image_get_green(image2, 1, 0, 0),
                           wb_camera_image_get_blue(image2, 1, 0, 0), 255, 255, 255,  // background
                           1, "Unexpected camera side color. The side of the capsule should be visible");

  ts_send_success();
  return EXIT_SUCCESS;
}
