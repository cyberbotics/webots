#include <webots/camera.h>
#include <webots/device.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

static double colorDisabled[3] = {0.0, 0.0, 0.0};
static double color0[3] = {0.8, 0.8, 0.8};
static double color1[3] = {0.0, 0.7, 0.5};
static double color2[3] = {0.7, 0.5, 0.7};
static double color3[3] = {0.25, 0.25, 0.7};

static void check_camera_image(WbDeviceTag tag, int width, const double *expected_color) {
  const unsigned char *image = wb_camera_get_image(tag);
  ts_assert_pointer_not_null((void *)image, "Cannot retrieve image pointer.");
  int red = wb_camera_image_get_red(image, width, 0, 0);
  int green = wb_camera_image_get_green(image, width, 0, 0);
  int blue = wb_camera_image_get_blue(image, width, 0, 0);
  int exp_red = 255.0 * expected_color[0] + 0.49;  // round double value to closest int
  int exp_green = 255.0 * expected_color[1] + 0.49;
  int exp_blue = 255.0 * expected_color[2] + 0.49;

  ts_assert_color_in_delta(red, green, blue, exp_red, exp_green, exp_blue, 20,
                           "Wrong color at time %f for camera %s: expected [%d %d %d] but received [%d %d %d]\n",
                           wb_robot_get_time(), wb_device_get_name(tag), exp_red, exp_green, exp_blue, red, green, blue);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef materialNode = wb_supervisor_node_get_from_def("MATERIAL");
  WbFieldRef colorField = wb_supervisor_node_get_field(materialNode, "diffuseColor");

  WbDeviceTag camera0 = wb_robot_get_device("camera0");
  WbDeviceTag camera1 = wb_robot_get_device("camera1");
  int width0 = wb_camera_get_width(camera0);
  int width1 = wb_camera_get_width(camera1);

  ts_assert_pointer_null((void *)wb_camera_get_image(camera0),
                         "Before camera enable and robot step, camera0 should have a NULL image.");
  ts_assert_pointer_null((void *)wb_camera_get_image(camera1),
                         "Before camera enable and robot step, camera1 should have a NULL image.");

  wb_camera_enable(camera0, TIME_STEP);
  wb_camera_enable(camera1, 2 * TIME_STEP);

  ts_assert_pointer_null((void *)wb_camera_get_image(camera0),
                         "After camera enable and before robot step, camera0 should have a NULL image.");
  ts_assert_pointer_null((void *)wb_camera_get_image(camera1),
                         "After camera enable and before robot step, camera1 should have a NULL image.");

  wb_robot_step(TIME_STEP);

  ts_assert_pointer_not_null((void *)wb_camera_get_image(camera0),
                             "After camera enable and after robot step, camera0 should have a valid image.");
  ts_assert_pointer_not_null((void *)wb_camera_get_image(camera1),
                             "After camera enable and after robot step, camera1 should have a valid image.");

  // camera0 updated
  check_camera_image(camera0, width0, color0);
  // camera1 not updated
  check_camera_image(camera1, width1, colorDisabled);

  wb_robot_step(TIME_STEP);

  // camera0 updated
  check_camera_image(camera0, width0, color0);
  // camera1 updated
  check_camera_image(camera1, width1, color0);

  wb_supervisor_field_set_sf_color(colorField, color1);

  wb_robot_step(TIME_STEP);

  // camera0 updated
  check_camera_image(camera0, width0, color1);
  // camera1 not updated
  check_camera_image(camera1, width1, color0);

  wb_robot_step(TIME_STEP);

  // camera0 updated
  check_camera_image(camera0, width0, color1);
  // camera1 updated
  check_camera_image(camera1, width1, color1);

  wb_supervisor_field_set_sf_color(colorField, color2);

  wb_robot_step(TIME_STEP);

  // camera0 updated
  check_camera_image(camera0, width0, color2);
  // camera1 not updated
  check_camera_image(camera1, width1, color1);

  wb_robot_step(TIME_STEP);

  // camera0 updated
  check_camera_image(camera0, width0, color2);
  // camera1 updated
  check_camera_image(camera1, width1, color2);

  wb_supervisor_field_set_sf_color(colorField, color3);

  wb_robot_step(TIME_STEP);

  // camera0 updated
  check_camera_image(camera0, width0, color3);
  // camera1 not updated
  check_camera_image(camera1, width1, color2);

  wb_robot_step(TIME_STEP);

  // camera0 updated
  check_camera_image(camera0, width0, color3);
  // camera1 updated
  check_camera_image(camera1, width1, color3);

  ts_send_success();
  return EXIT_SUCCESS;
}
