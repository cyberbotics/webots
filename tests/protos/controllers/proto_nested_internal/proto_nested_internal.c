#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);  // give the controller args
  bool loadTest = (strcmp(argv[1], "proto_nested_fixed") == 0);
  bool colored = (strcmp(argv[1], "derived_proto_nested_internal_4") == 0);

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  ts_assert_int_equal(wb_camera_get_width(camera), 1, "Wrong camera width");
  ts_assert_int_equal(wb_camera_get_height(camera), 1, "Wrong camera height");

  WbNodeRef protoNode;
  WbFieldRef sizeField = NULL;
  if (!loadTest) {
    protoNode = wb_supervisor_node_get_from_def("TEST_PROTO");
    sizeField = wb_supervisor_node_get_field(protoNode, "boxSize");
  }

  wb_robot_step(TIME_STEP);

  int redComponent = colored ? 207 : 0;

  const unsigned char *values_before = wb_camera_get_image(camera);
  ts_assert_color_in_delta(wb_camera_image_get_red(values_before, 1, 0, 0), wb_camera_image_get_green(values_before, 1, 0, 0),
                           wb_camera_image_get_blue(values_before, 1, 0, 0), redComponent, 0, 0,  // box
                           1, "Unexpected camera color. The box should be visible.");

  if (loadTest) {
    ts_send_success();
    return EXIT_SUCCESS;
  }

  wb_robot_step(TIME_STEP);

  double new_size[3] = {0.4, 0.4, 1};
  wb_supervisor_field_set_sf_vec3f(sizeField, new_size);

  wb_robot_step(TIME_STEP);

  const unsigned char *values_after = wb_camera_get_image(camera);
  ts_assert_color_in_delta(wb_camera_image_get_red(values_after, 1, 0, 0), wb_camera_image_get_green(values_after, 1, 0, 0),
                           wb_camera_image_get_blue(values_after, 1, 0, 0), 255, 255, 255,  // background
                           1, "Unexpected camera color after resizing. The box should not be visible.");

  ts_send_success();
  return EXIT_SUCCESS;
}
