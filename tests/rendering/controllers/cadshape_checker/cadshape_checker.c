#include <stdio.h>
#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64
#define NB_CAMERAS 30

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

void test_color_comparison(int i, int j) {
  int r1, g1, b1, r2, g2, b2;
  const int width = wb_camera_get_width(cameras[i]);

  const unsigned char *image1 = wb_camera_get_image(cameras[i]);
  const unsigned char *image2 = wb_camera_get_image(cameras[j]);
  r1 = wb_camera_image_get_red(image1, width, 0, 0);
  g1 = wb_camera_image_get_green(image1, width, 0, 0);
  b1 = wb_camera_image_get_blue(image1, width, 0, 0);

  r2 = wb_camera_image_get_red(image2, width, 0, 0);
  g2 = wb_camera_image_get_green(image2, width, 0, 0);
  b2 = wb_camera_image_get_blue(image2, width, 0, 0);

  ts_assert_color_in_delta(
    r1, g1, b1, r2, g2, b2, 2,
    "Color from camera %d does not coincide with color from camera %d (respectively [%d, %d, %d] and [%d, %d, %d])", i, j, r1,
    g1, b1, r2, g2, b2);
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

  // ensure the texture map was applied correctly both for collada and wavefront objects
  const int expected_color[6][3] = {{203, 35, 35}, {35, 38, 201}, {201, 195, 35}, {175, 57, 56}, {57, 58, 175}, {175, 169, 56}};
  for (int i = 0; i < 24; ++i) {
    const int color = (int)(i / 4);
    test_camera_color(i, expected_color[color]);
  }

  // ensure the mapping applied by CadShape is the same compared to using base nodes (Mesh + PBRAppearance)
  // collada CadShape and base node counterpart
  test_color_comparison(0, 24);
  test_color_comparison(4, 25);
  test_color_comparison(8, 26);
  // wavefront CadShape and base node counterpart
  test_color_comparison(12, 27);
  test_color_comparison(16, 28);
  test_color_comparison(20, 29);

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
