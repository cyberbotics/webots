#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

// Test mirror image in 3D scene with transparent recorded texture:
// if transparent textures are not handled properly the mirror image could be affected by the objects behind it.

// Camera and display overlays could be affected by the same issues.
// Visual test:
// even if the green object is rendered behind the overlays, the top area of the overlays' image should be white (199, 199,
// 199).

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  wb_robot_step(TIME_STEP);

  // check mirror image recorded by a camera
  const int width = wb_camera_get_width(camera);
  const unsigned char *image = wb_camera_get_image(camera);

  // check the top white  color
  int x = 30;
  int y = 20;
  int r = wb_camera_image_get_red(image, width, x, y);
  int g = wb_camera_image_get_green(image, width, x, y);
  int b = wb_camera_image_get_blue(image, width, x, y);
  ts_assert_color_in_delta(r, g, b, 180, 180, 180, 5, "The white color of the mirror image in the 3D scene is wrong.");

  // check the bottom red color
  x = 30;
  y = 50;
  r = wb_camera_image_get_red(image, width, x, y);
  g = wb_camera_image_get_green(image, width, x, y);
  b = wb_camera_image_get_blue(image, width, x, y);
  ts_assert_color_in_delta(r, g, b, 180, 156, 156, 5, "The red color of the mirror image in the 3D scene is wrong.");

  ts_send_success();
  return EXIT_SUCCESS;
}
