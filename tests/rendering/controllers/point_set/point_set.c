#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int expected_colors[4][3] = {{207, 0, 0}, {0, 207, 0}, {0, 0, 207}, {207, 0, 207}};

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const unsigned char *camera_image = wb_camera_get_image(camera);
  const int width = wb_camera_get_width(camera);
  const int height = wb_camera_get_height(camera);

  int i = 0;
  for (i = 0; i < 3; ++i) {
    int x = (i + 1) * width / 4.0;
    int y = height / 2.0;
    int red = wb_camera_image_get_red(camera_image, width, x, y);
    int green = wb_camera_image_get_green(camera_image, width, x, y);
    int blue = wb_camera_image_get_blue(camera_image, width, x, y);
    ts_assert_color_in_delta(red, green, blue, expected_colors[i][0], expected_colors[i][1], expected_colors[i][2], 2,
                             "Wrong point color when the 'Color' node is defined.");
  }

  WbNodeRef color_node = wb_supervisor_node_get_from_def("COLOR");
  wb_supervisor_node_remove(color_node);
  wb_robot_step(TIME_STEP);

  camera_image = wb_camera_get_image(camera);
  for (i = 0; i < 3; ++i) {
    int x = (i + 1) * width / 4.0;
    int y = height / 2.0;
    int red = wb_camera_image_get_red(camera_image, width, x, y);
    int green = wb_camera_image_get_green(camera_image, width, x, y);
    int blue = wb_camera_image_get_blue(camera_image, width, x, y);
    ts_assert_color_in_delta(
      red, green, blue, expected_colors[3][0], expected_colors[3][1], expected_colors[3][2], 2,
      "Point color should be equal to the material 'emissiveColor' when the 'Color' node is not defined.");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
