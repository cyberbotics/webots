/*
 * Regression test for issue #4075
 */

#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);  // give the controller args

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  WbNodeRef protoNode = wb_supervisor_node_get_from_def("TEST_PROTO");
  WbFieldRef colorField = wb_supervisor_node_get_field(protoNode, "color");

  wb_robot_step(TIME_STEP);

  const unsigned char *valuesA = wb_camera_get_image(camera);
  ts_assert_color_in_delta(wb_camera_image_get_red(valuesA, 1, 0, 0), wb_camera_image_get_green(valuesA, 1, 0, 0),
                           wb_camera_image_get_blue(valuesA, 1, 0, 0), 54, 54, 54,  // initial box color
                           1, "Unexpected initial camera color");

  double newColor[] = {0.0, 1.0, 0.0};
  wb_supervisor_field_set_sf_color(colorField, newColor);

  wb_robot_step(TIME_STEP);

  const unsigned char *valuesB = wb_camera_get_image(camera);
  printf("%d %d %d\n", wb_camera_image_get_red(valuesB, 1, 0, 0), wb_camera_image_get_blue(valuesB, 1, 0, 0),
         wb_camera_image_get_green(valuesB, 1, 0, 0));

  ts_assert_color_in_delta(wb_camera_image_get_red(valuesB, 1, 0, 0), wb_camera_image_get_green(valuesB, 1, 0, 0),
                           wb_camera_image_get_blue(valuesB, 1, 0, 0), 0, 207, 0,  // modified box color
                           1, "Unexpected camera color after changing box color to green.");

  ts_send_success();
  return EXIT_SUCCESS;
}
