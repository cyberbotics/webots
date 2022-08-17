#include <stdio.h>
#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64
#define NB_CAMERAS 8

static WbDeviceTag cameras[NB_CAMERAS];

void test_camera_color(int i, const int expected_color[3]) {
  int r, g, b;
  const int width = wb_camera_get_width(cameras[i]);

  const unsigned char *image = wb_camera_get_image(cameras[i]);
  r = wb_camera_image_get_red(image, width, 0, 0);
  g = wb_camera_image_get_green(image, width, 0, 0);
  b = wb_camera_image_get_blue(image, width, 0, 0);
  ts_assert_color_in_delta(r, g, b, expected_color[0], expected_color[1], expected_color[2], 5,
                           "Camera %d received color = [%d, %d, %d] but expected color = [%d, %d, %d]", i, r, g, b,
                           expected_color[0], expected_color[1], expected_color[2]);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  ts_set_test_name(wb_robot_get_world_path());

  wb_robot_step(TIME_STEP);

  char device_name[10];
  for (int i = 0; i < NB_CAMERAS; ++i) {
    sprintf(device_name, "camera%d", i);
    cameras[i] = wb_robot_get_device(device_name);
    wb_camera_enable(cameras[i], TIME_STEP);
  }

  wb_robot_step(TIME_STEP);

  // set the url of the last block using a supervisor
  WbNodeRef node_ref = wb_supervisor_node_get_from_def("PROTO_NODE");
  WbFieldRef field_ref = wb_supervisor_node_get_field(node_ref, "url_of_nested_shape_with_parameter");
  // set the url relative to the world since the field is visible (with random directory movements)
  wb_supervisor_field_set_mf_string(field_ref, 0, "../worlds/nonexistant_folder/../textures/black_texture.jpg");

  wb_robot_step(TIME_STEP);

  // ensure the texture map was loaded correctly
  const int expected_color[NB_CAMERAS][3] = {{0, 18, 203},  {35, 203, 0}, {6, 176, 203}, {203, 83, 0},
                                             {203, 0, 175}, {203, 0, 0},  {203, 196, 0}, {0, 0, 0}};
  for (int i = 0; i < NB_CAMERAS; ++i)
    test_camera_color(i, expected_color[i]);

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
