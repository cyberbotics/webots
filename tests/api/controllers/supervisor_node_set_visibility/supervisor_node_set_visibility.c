#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

static void quick_assert_color(WbDeviceTag camera, int px, int py, int expected, const char *error_msg) {
  const unsigned char *image = wb_camera_get_image(camera);
  int w = wb_camera_get_width(camera);

  ts_assert_color_in_delta(wb_camera_image_get_red(image, w, px, py), wb_camera_image_get_green(image, w, px, py),
                           wb_camera_image_get_blue(image, w, px, py), (expected >> 16) & 0xFF, (expected >> 8) & 0xFF,
                           expected & 0xFF, 1, error_msg);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag camera0 = wb_robot_get_device("camera0");
  WbDeviceTag camera1 = wb_robot_get_device("camera1");

  wb_camera_enable(camera0, TIME_STEP);
  wb_camera_enable(camera1, TIME_STEP);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera0, 31, 31, 0xFF0000, "Both cameras should see the red box at the beginning.\n");
  quick_assert_color(camera1, 31, 31, 0xFF0000, "Both cameras should see the red box at the beginning.\n");

  WbNodeRef camera_node = wb_supervisor_node_get_from_def("CAMERA0");
  WbNodeRef camera1_node = wb_supervisor_node_get_from_def("CAMERA1");
  WbNodeRef red_box_node = wb_supervisor_node_get_from_def("RED_BOX");
  WbNodeRef green_box_node = wb_supervisor_node_get_from_def("GREEN_BOX");

  wb_supervisor_node_set_visibility(red_box_node, camera_node, false);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera0, 31, 31, 0x00FF00, "Camera 0 should not see the red box after it was hidden from this camera.\n");
  quick_assert_color(camera1, 31, 31, 0xFF0000, "Camera 1 should still see the red box after it was hidden from camera 0.\n");

  wb_supervisor_node_set_visibility(red_box_node, camera_node, true);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera0, 31, 31, 0xFF0000,
                     "Both cameras should see the red box again after it was shown from camera 0.\n");
  quick_assert_color(camera1, 31, 31, 0xFF0000,
                     "Both cameras should see the red box again after it was shown from camera 0.\n");

  wb_supervisor_node_set_visibility(green_box_node, camera1_node, false);
  wb_supervisor_node_set_visibility(red_box_node, camera_node, false);
  wb_supervisor_node_remove(red_box_node);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera0, 31, 31, 0x00FF00, "Camera 0 should not see the red box after it has been removed.\n");
  quick_assert_color(camera1, 31, 31, 0x0000FF,
                     "Camera 1 should only see the background after red box has been removed and green box has been hidden.\n");

  ts_send_success();
  return EXIT_SUCCESS;
}
