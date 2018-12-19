#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  const unsigned char *image = NULL;
  int r, g, b;
  const double *values = NULL;

  ts_setup(argv[0]);

  int time_step = wb_robot_get_basic_time_step();
  WbDeviceTag camera = wb_robot_get_device("camera");
  WbDeviceTag compass = wb_robot_get_device("compass");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag ds = wb_robot_get_device("distance sensor");
  WbDeviceTag ts = wb_robot_get_device("touch sensor");

  wb_camera_enable(camera, 3 * time_step);
  wb_compass_enable(compass, 3 * time_step);
  wb_gps_enable(gps, 3 * time_step);
  wb_distance_sensor_enable(ds, 3 * time_step);
  wb_touch_sensor_enable(ts, 3 * time_step);

  // step 1 - 32 ms
  // step 2 - 64 ms
  // step 3 - 96 ms -> refresh sensor value
  // change robot position
  // step 4 - 128 ms -> sensor values not updated
  // step 5 - 160 ms
  wb_robot_step(5 * time_step);

  // check initial sensor values

  // camera
  image = wb_camera_get_image(camera);
  r = wb_camera_image_get_red(image, 1, 0, 0);
  g = wb_camera_image_get_green(image, 1, 0, 0);
  b = wb_camera_image_get_blue(image, 1, 0, 0);
  ts_assert_color_in_delta(r, g, b, 0x66, 0xB2, 0xFF, 5, "Initial camera measurement is wrong.");

  // compass
  values = wb_compass_get_values(compass);
  const double initialCompassValues[3] = {1.0, 0.0, 0.0};
  ts_assert_doubles_in_delta(3, values, initialCompassValues, 0.0001, "Initial compass measurement is wrong.");

  // distance sensor
  ts_assert_double_in_delta(wb_distance_sensor_get_value(ds), 1000.0, 0.001, "Initial distance sensor measurement is wrong.");

  // gps
  values = wb_gps_get_values(gps);
  const double initialGpsValues[3] = {0.100000, 0.025178, 0.100000};
  ts_assert_doubles_in_delta(3, values, initialGpsValues, 0.00001, "Initial GPS measurement is wrong.");

  // touch sensor
  ts_assert_double_in_delta(wb_touch_sensor_get_value(ts), 0.0, 0.001, "Initial touch sensor measurement is wrong.");

  wb_robot_step(time_step);

  // check sensor values after robot position changed

  // camera
  image = wb_camera_get_image(camera);
  r = wb_camera_image_get_red(image, 1, 0, 0);
  g = wb_camera_image_get_green(image, 1, 0, 0);
  b = wb_camera_image_get_blue(image, 1, 0, 0);
  ts_assert_color_in_delta(r, g, b, 207, 207, 207, 5, "Camera measurement after robot moved is wrong.");

  // compass
  values = wb_compass_get_values(compass);
  const double updatedCompassValues[3] = {0.003230, -0.034126, 0.999412};
  ts_assert_doubles_in_delta(3, values, updatedCompassValues, 0.00001,
                             "Compass measurement after robot moved is wrong (received: {%f, %f, %f}).", values[0], values[1],
                             values[2]);

  // distance sensor
  double value = wb_distance_sensor_get_value(ds);
  ts_assert_double_in_delta(value, 666.766, 0.001, "Distance sensor measurement after robot moved is wrong (received: %f).",
                            value);

  // gps
  values = wb_gps_get_values(gps);
  const double updatedGpsValues[3] = {-0.049331, 0.025861, 0.101667};
  ts_assert_doubles_in_delta(3, values, updatedGpsValues, 0.00001,
                             "GPS measurement after robot moved is wrong (received: {%f, %f, %f}).", values[0], values[1],
                             values[2]);

  // touch sensor
  value = wb_touch_sensor_get_value(ts);
  ts_assert_double_in_delta(value, 5.442, 0.001, "Touch sensor measurement after robot moved is wrong (received: %f).", value);

  ts_send_success();
  return EXIT_SUCCESS;
}
