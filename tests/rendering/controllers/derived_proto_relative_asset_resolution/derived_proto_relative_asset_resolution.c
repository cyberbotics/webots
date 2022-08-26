#include <stdio.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64
#define NB_DEVICES 7

static WbDeviceTag cameras[NB_DEVICES];
static WbDeviceTag sensors[NB_DEVICES];

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

void test_distance(int i, int expected_distance) {
  const double distance = wb_distance_sensor_get_value(sensors[i]);

  ts_assert_double_in_delta(distance, expected_distance, 2, "Distance value of sensor %d is %f but expected %f", i, distance,
                            expected_distance);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  ts_set_test_name(wb_robot_get_world_path());

  wb_robot_step(TIME_STEP);

  char device_name[10];
  for (int i = 0; i < NB_DEVICES; ++i) {
    // initialize cameras
    sprintf(device_name, "camera%d", i);
    cameras[i] = wb_robot_get_device(device_name);
    wb_camera_enable(cameras[i], TIME_STEP);
    // initialize distance sensors
    if (i < 4) {
      sprintf(device_name, "sensor%d", i);
      sensors[i] = wb_robot_get_device(device_name);
      wb_distance_sensor_enable(sensors[i], TIME_STEP);
    }
  }

  wb_robot_step(TIME_STEP);

  // ensure the texture and mesh was loaded correctly for derived PROTO with IS-chained parameters
  const int expected_color[NB_DEVICES][3] = {{203, 0, 0},   {35, 203, 0}, {0, 18, 203}, {203, 196, 0},
                                             {203, 0, 175}, {203, 85, 0}, {203, 196, 0}};
  const int expected_distance[NB_DEVICES] = {400, 300, 200, 100};
  for (int i = 0; i < NB_DEVICES; ++i) {
    if (i < 4)
      test_distance(i, expected_distance[i]);

    test_camera_color(i, expected_color[i]);
  }

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
