#include <stdio.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64
#define NB_TEXTURES 4

static WbDeviceTag cameras[NB_TEXTURES];

void test_camera_color(int camera, const int expected_color[3]) {
  int r, g, b;
  const int width = wb_camera_get_width(cameras[0]);

  const unsigned char *image = wb_camera_get_image(cameras[camera]);
  r = wb_camera_image_get_red(image, width, 32, 32);
  g = wb_camera_image_get_green(image, width, 32, 32);
  b = wb_camera_image_get_blue(image, width, 32, 32);
  ts_assert_color_in_delta(
    r, g, b, expected_color[0], expected_color[1], expected_color[2], 5,
    "Camera %d detected wrong texture color, received color = (%d, %d, %d) but expected color = (%d, %d, %d)", camera, r, g, b,
    expected_color[0], expected_color[1], expected_color[2]);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // initialize devices
  char device_name[20];

  for (int i = 0; i < NB_TEXTURES; ++i) {
    sprintf(device_name, "camera%d", i);
    cameras[i] = wb_robot_get_device(device_name);
    wb_camera_enable(cameras[i], TIME_STEP);
  }

  wb_robot_step(TIME_STEP);

  // test textures
  const int expected_blue_color[3] = {2, 11, 135};
  test_camera_color(0, expected_blue_color);

  const int expected_red_color[3] = {135, 2, 2};
  test_camera_color(1, expected_red_color);

  const int expected_yellow_color[3] = {203, 196, 0};
  test_camera_color(2, expected_yellow_color);

  const int expected_green_color[3] = {35, 203, 0};
  test_camera_color(3, expected_green_color);

  ts_send_success();
  return EXIT_SUCCESS;
}
