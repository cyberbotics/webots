#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

static void check_expected_colors(WbDeviceTag camera, int width, int height, bool mask) {
  int red, green, blue;
  int x, y;
  const unsigned char *image = wb_camera_get_image(camera);

  int samples_number = 4;
  int samples_positions[4][2] = {
    {width / 6 - height / 12, height / 4 - height / 8},  // first line, first square
    {width - height / 12, height / 4 - height / 8},      // first line, last square
    {width / 6 - height / 12, height - height / 8},      // last line, first square
    {width - height / 12, height - height / 8}           // last line, last square
  };
  int samples_expected_colors[4][3] = {
    {123, 71, 58},    // brown
    {156, 205, 195},  // cyan
    {207, 207, 207},  // white
    {44, 44, 44}      // dark gray
  };

  int i;
  for (i = 0; i < samples_number; i++) {
    x = samples_positions[i][0];
    y = samples_positions[i][1];
    red = wb_camera_image_get_red(image, width, x, y);
    green = wb_camera_image_get_green(image, width, x, y);
    blue = wb_camera_image_get_blue(image, width, x, y);
    if (mask) {
      ts_assert_color_in_delta(red, green, blue, 42, 140, 32, 1,
                               "Wrong color at (%d, %d), Received color = (%d, %d, %d), Expected color = (%d, %d, %d)", x, y,
                               red, green, blue, 42, 140, 32);
    } else {
      ts_assert_color_in_delta(
        red, green, blue, samples_expected_colors[i][0], samples_expected_colors[i][1], samples_expected_colors[i][2], 1,
        "Wrong color at (%d, %d), Received color = (%d, %d, %d), Expected color = (%d, %d, %d)", x, y, red, green, blue,
        samples_expected_colors[i][0], samples_expected_colors[i][1], samples_expected_colors[i][2]);
    }
  }
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef cameraNode = wb_supervisor_node_get_from_def("CAMERA");
  WbFieldRef noiseMaskUrlField = wb_supervisor_node_get_field(cameraNode, "noiseMaskUrl");

  WbDeviceTag camera = wb_robot_get_device("camera");
  ts_assert_boolean_equal(camera, "Camera device cannot be get.");

  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);

  ts_assert_boolean_equal(width >= 1 && height >= 1, "Unexpected dimension: %dx%d", width, height);

  wb_camera_enable(camera, TIME_STEP);

  wb_robot_step(TIME_STEP);

  // check reference image
  check_expected_colors(camera, width, height, false);

  // set opaque mask
  wb_supervisor_field_set_sf_string(noiseMaskUrlField, "textures/noise_mask_opaque.png");
  wb_robot_step(TIME_STEP);
  check_expected_colors(camera, width, height, true);

  // set transparent mask
  wb_supervisor_field_set_sf_string(noiseMaskUrlField, "textures/noise_mask_transparent.png");
  wb_robot_step(TIME_STEP);
  check_expected_colors(camera, width, height, false);

  ts_send_success();
  return EXIT_SUCCESS;
}
