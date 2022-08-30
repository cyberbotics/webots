#include <stdio.h>
#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64
#define NB_CAMERAS 12

static const int red[3] = {203, 0, 0};
static const int green[3] = {35, 203, 0};
static const int blue[3] = {0, 18, 203};

static WbDeviceTag cameras[NB_CAMERAS];

void test_camera_color(int i, const int expected_color[3]) {
  const int width = wb_camera_get_width(cameras[i]);

  const unsigned char *image = wb_camera_get_image(cameras[i]);
  const int r = wb_camera_image_get_red(image, width, 0, 0);
  const int g = wb_camera_image_get_green(image, width, 0, 0);
  const int b = wb_camera_image_get_blue(image, width, 0, 0);
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

  // ensure the texture map was loaded correctly
  const int *expected_color[NB_CAMERAS][3] = {{red}, {green}, {red}, {green}, {red}, {green},
                                              {red}, {blue},  {red}, {green}, {red}, {blue}};
  for (int i = 0; i < NB_CAMERAS; ++i)
    test_camera_color(i, *expected_color[i]);

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
