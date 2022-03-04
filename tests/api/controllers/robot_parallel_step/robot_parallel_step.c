#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/touch_sensor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  const unsigned char *image = NULL;
  int r, g, b;
  const double *values = NULL;

  ts_setup(argv[0]);

  const int time_step = wb_robot_get_basic_time_step();
  WbDeviceTag camera = wb_robot_get_device("camera");
  WbDeviceTag compass = wb_robot_get_device("compass");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag ds = wb_robot_get_device("distance sensor");
  WbDeviceTag ts = wb_robot_get_device("touch sensor");

  wb_camera_enable(camera, time_step);
  wb_compass_enable(compass, time_step);
  wb_gps_enable(gps, time_step);
  wb_distance_sensor_enable(ds, time_step);
  wb_touch_sensor_enable(ts, time_step);

  WbNodeRef robot_node = wb_supervisor_node_get_from_def("ROBOT");
  WbFieldRef translation_field = wb_supervisor_node_get_field(robot_node, "translation");
  WbFieldRef rotation_field = wb_supervisor_node_get_field(robot_node, "rotation");
  wb_supervisor_field_enable_sf_tracking(rotation_field, time_step);

  // step 1 - 32 ms -> get initial sensor values, robot node/fields and update position
  wb_robot_step_begin(time_step);

  const double newRotation[4] = {0.0, 1.0, 0.0, 1.5708};
  const double newTranslation[3] = {-0.05, 0.0295, 0.1};
  wb_supervisor_field_set_sf_rotation(rotation_field, newRotation);
  wb_supervisor_field_set_sf_vec3f(translation_field, newTranslation);

  wb_robot_step_end();

  // step 2 - 64 ms -> check initial sensor values
  wb_robot_step_begin(time_step);

  // camera
  image = wb_camera_get_image(camera);
  r = wb_camera_image_get_red(image, 1, 0, 0);
  g = wb_camera_image_get_green(image, 1, 0, 0);
  b = wb_camera_image_get_blue(image, 1, 0, 0);

  ts_assert_color_in_delta(r, g, b, 102, 178, 255, 5, "Initial camera measurement is wrong.");

  // compass
  values = wb_compass_get_values(compass);
  const double initialCompassValues[3] = {1.0, 0.0, 0.0};
  ts_assert_doubles_in_delta(3, values, initialCompassValues, 0.0001, "Initial compass measurement is wrong.");

  // distance sensor
  ts_assert_double_in_delta(wb_distance_sensor_get_value(ds), 1000.0, 0.001, "Initial distance sensor measurement is wrong.");

  // gps
  values = wb_gps_get_values(gps);
  const double initialGpsValues[3] = {0.100000, 0.0194546, 0.100000};
  ts_assert_doubles_in_delta(3, values, initialGpsValues, 0.00001, "Initial GPS measurement is wrong.");

  // touch sensor
  ts_assert_double_in_delta(wb_touch_sensor_get_value(ts), 0.0, 0.001, "Initial touch sensor measurement is wrong.");

  // second pose update
  const double newRotation1[4] = {0.0, 0.0, 1.0, 0.0};
  const double newTranslation1[3] = {0.1, 0.0295, 0.1};
  wb_supervisor_field_set_sf_rotation(rotation_field, newRotation1);
  wb_supervisor_field_set_sf_vec3f(translation_field, newTranslation1);

  wb_robot_step_end();  // retrieve sensor values after step update

  // step 3 - 96 ms -> check camera values after first pose update (second is only visible by the sensor after the next
  // step_end)
  wb_robot_step_begin(time_step);

  image = wb_camera_get_image(camera);

  // intense computation between begin and end
  for (int i = 0; i < 1000000; i++) {
    r = wb_camera_image_get_red(image, 1, 0, 0);
    g = wb_camera_image_get_blue(image, 1, 0, 0);
    b = wb_camera_image_get_green(image, 1, 0, 0);
  }
  ts_assert_color_in_delta(r, g, b, 207, 207, 207, 5, "Camera measurement after first pose update is wrong.");

  wb_robot_step_end();

  // check camera value after second pose update
  image = wb_camera_get_image(camera);
  r = wb_camera_image_get_red(image, 1, 0, 0);
  g = wb_camera_image_get_green(image, 1, 0, 0);
  b = wb_camera_image_get_blue(image, 1, 0, 0);

  ts_assert_color_in_delta(r, g, b, 102, 178, 255, 5, "Camera measurement after second pose update is wrong.");

  ts_send_success();
  return EXIT_SUCCESS;
}
