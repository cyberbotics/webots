#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);  // give the controller args

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  ts_assert_int_equal(wb_camera_get_width(camera), 1, "Wrong camera width");
  ts_assert_int_equal(wb_camera_get_height(camera), 1, "Wrong camera height");

  WbNodeRef protoNode = wb_supervisor_node_get_from_def("TEST_PROTO");
  WbFieldRef slotField = wb_supervisor_node_get_field(protoNode, "sensorsSlot");

  wb_robot_step(TIME_STEP);

  const unsigned char *valuesA = wb_camera_get_image(camera);
  ts_assert_color_in_delta(wb_camera_image_get_red(valuesA, 1, 0, 0), wb_camera_image_get_green(valuesA, 1, 0, 0),
                           wb_camera_image_get_blue(valuesA, 1, 0, 0), 255, 255, 255,  // background
                           1, "Unexpected camera color. The box should not be visible.");

  wb_robot_step(TIME_STEP);

  wb_supervisor_field_import_mf_node_from_string(
    slotField, 0, "DEF SHAPE Shape { appearance Appearance { material Material {} } geometry DEF BOX Box { size 1 1 1 } }");

  wb_robot_step(TIME_STEP);

  const unsigned char *valuesB = wb_camera_get_image(camera);
  ts_assert_color_in_delta(wb_camera_image_get_red(valuesB, 1, 0, 0), wb_camera_image_get_green(valuesB, 1, 0, 0),
                           wb_camera_image_get_blue(valuesB, 1, 0, 0), 0, 0, 0,  // box
                           1, "Unexpected camera color after importing node. The box should be visible.");

  wb_robot_step(TIME_STEP);

  WbNodeRef boxNode = wb_supervisor_node_get_from_def("BOX");
  WbFieldRef sizeField = wb_supervisor_node_get_field(boxNode, "size");
  double newSize[] = {0.5, 1, 1};
  wb_supervisor_field_set_sf_vec3f(sizeField, newSize);

  wb_robot_step(TIME_STEP);

  const unsigned char *valuesC = wb_camera_get_image(camera);
  ts_assert_color_in_delta(wb_camera_image_get_red(valuesC, 1, 0, 0), wb_camera_image_get_green(valuesC, 1, 0, 0),
                           wb_camera_image_get_blue(valuesC, 1, 0, 0), 255, 255, 255,  // background
                           1, "Unexpected camera color after resizing. The box should not be visible.");

  ts_send_success();
  return EXIT_SUCCESS;
}
