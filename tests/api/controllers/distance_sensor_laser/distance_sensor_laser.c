#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  double value;
  int chanel;

  WbDeviceTag ds0 = wb_robot_get_device("ds0");
  WbDeviceTag ds1 = wb_robot_get_device("ds1");
  WbDeviceTag camera = wb_robot_get_device("camera");

  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);
  wb_camera_enable(camera, TIME_STEP);

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds0);
  ts_assert_double_in_delta(value, 500.0, 0.001,
                            "The distance sensor in laser mode doesn't return the right distance when hitting an object.");

  value = wb_distance_sensor_get_value(ds1);
  ts_assert_double_in_delta(value, 1000.0, 0.001,
                            "The distance sensor in laser mode doesn't return the max range when missing an object.");

  wb_robot_step(TIME_STEP);

  const unsigned char *image = wb_camera_get_image(camera);
  chanel = wb_camera_image_get_red(image, 1, 0, 0);
  ts_assert_int_equal(chanel, 0xEF, "The laser beam of the distance sensor in laser mode isn't visible from a camera.");

  chanel = wb_camera_image_get_green(image, 1, 0, 0);
  ts_assert_int_equal(chanel, 0, "The laser beam of the distance sensor in laser mode isn't visible from a camera.");

  chanel = wb_camera_image_get_blue(image, 1, 0, 0);
  ts_assert_int_equal(chanel, 0, "The laser beam of the distance sensor in laser mode isn't visible from a camera.");

  ts_send_success();
  return EXIT_SUCCESS;
}
