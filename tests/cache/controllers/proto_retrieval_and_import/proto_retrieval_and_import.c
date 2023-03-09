#include <stdio.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

void test_camera_color(int test, WbDeviceTag camera, const int expected_color[3]) {
  int r, g, b;
  const int width = wb_camera_get_width(camera);

  const unsigned char *image = wb_camera_get_image(camera);
  r = wb_camera_image_get_red(image, width, 32, 32);
  g = wb_camera_image_get_green(image, width, 32, 32);
  b = wb_camera_image_get_blue(image, width, 32, 32);
  ts_assert_color_in_delta(r, g, b, expected_color[0], expected_color[1], expected_color[2], 5,
                           "Test %d detected wrong color, received color = (%d, %d, %d) but expected color = (%d, %d, %d)",
                           test, r, g, b, expected_color[0], expected_color[1], expected_color[2]);
}

int main(int argc, const char *argv[]) {
  ts_setup(argv[0]);

  const int nb_tests = argc - 1;
  int expected_colors[nb_tests][3];

  for (int i = 1; i < argc; ++i)
    sscanf(argv[i], "%d %d %d", &expected_colors[i - 1][0], &expected_colors[i - 1][1], &expected_colors[i - 1][2]);

  // initialize devices
  WbDeviceTag cameras[nb_tests];
  char device_name[20];

  for (int i = 0; i < nb_tests; ++i) {
    sprintf(device_name, "camera%d", i);
    cameras[i] = wb_robot_get_device(device_name);
    wb_camera_enable(cameras[i], TIME_STEP);
  }

  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef children_field = wb_supervisor_node_get_field(root_node, "children");

  // import all variants of externally defined PROTO
  char vrml[100] = "Transform { children [ ShapeWithCustomTexture { url \"textures/blue_texture.jpg\" } ] }";
  wb_supervisor_field_import_mf_node_from_string(children_field, -1, vrml);
  strcpy(vrml, "Transform { translation 0 0 2 children [ ShapeWithAbsoluteTexture {} ] }");
  wb_supervisor_field_import_mf_node_from_string(children_field, -1, vrml);
  strcpy(vrml, "Transform { translation 0 0 4 children [ ShapeWithRelativeTexture {} ] }");
  wb_supervisor_field_import_mf_node_from_string(children_field, -1, vrml);

  wb_robot_step(TIME_STEP);

  // test textures are also loaded correctly
  for (int i = 0; i < nb_tests; ++i)
    test_camera_color(i, cameras[i], expected_colors[i]);

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
