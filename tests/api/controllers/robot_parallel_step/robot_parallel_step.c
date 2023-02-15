#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <unistd.h>

int main(int argc, char **argv) {
  const double *values = NULL;
  double distance = 0.0;

  ts_setup(argv[0]);

  const int time_step = wb_robot_get_basic_time_step();
  WbDeviceTag compass = wb_robot_get_device("compass");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag ds = wb_robot_get_device("distance sensor");

  wb_compass_enable(compass, time_step);
  wb_gps_enable(gps, time_step);
  wb_distance_sensor_enable(ds, time_step);

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

  // compass
  values = wb_compass_get_values(compass);
  const double initialCompassValues[3] = {1.0, 0.0, 0.0};
  ts_assert_doubles_in_delta(3, values, initialCompassValues, 0.0001, "Initial compass measurement is wrong.");

  // distance sensor
  distance = wb_distance_sensor_get_value(ds);
  ts_assert_double_in_delta(distance, 1000, 0.001, "Initial distance sensor measurement is wrong.");

  // gps
  values = wb_gps_get_values(gps);
  const double initialGpsValues[3] = {0.1, 0.0295, 0.1};
  ts_assert_doubles_in_delta(3, values, initialGpsValues, 0.00001, "Initial GPS measurement is wrong.");

  // second pose update
  const double newRotation1[4] = {0.0, 0.0, 1.0, 0.0};
  const double newTranslation1[3] = {0.1, 0.0295, 0.1};
  wb_supervisor_field_set_sf_rotation(rotation_field, newRotation1);
  wb_supervisor_field_set_sf_vec3f(translation_field, newTranslation1);

  wb_robot_step_end();  // retrieve sensor values after step update

  // step 3 - 96 ms -> check sensor values after first pose update (second is only visible by the sensor after the next
  // step_end)
  wb_robot_step_begin(time_step);

  // compass
  values = wb_compass_get_values(compass);
  const double newCompassValues1[3] = {0.0, 0.0, 1.0};
  ts_assert_doubles_in_delta(3, values, newCompassValues1, 0.0001, "Compass measurement after first pose update is wrong.");

  // distance sensor
  distance = wb_distance_sensor_get_value(ds);
  ts_assert_double_in_delta(distance, 650.0, 0.001, "Distance sensor measurement after first pose update is wrong.");

  // gps
  values = wb_gps_get_values(gps);
  const double newGpsValues1[3] = {-0.05, 0.0295, 0.1};
  ts_assert_doubles_in_delta(3, values, newGpsValues1, 0.00001, "GPS measurement after first pose update is wrong.");

  // Give webots plenty of time to finish the step so we can be sure we'll detect any unexpected tampering with our values.
  usleep(time_step * 1000 + 100000);

  // Do some intensive computation, just for good measure
  int result = 0xffff;
  for (int i = 0; i < 0xffff; i++)
    result &= ~i;
  ts_assert_int_equal(result, 0, "Computation produced unexpected result.");

  // compass
  values = wb_compass_get_values(compass);
  ts_assert_doubles_in_delta(3, values, newCompassValues1, 0.0001,
                             "Delayed compass measurement after first pose update is wrong.");

  // distance sensor
  distance = wb_distance_sensor_get_value(ds);
  ts_assert_double_in_delta(distance, 650.0, 0.001, "Delayed distance sensor measurement after first pose update is wrong.");

  // gps
  values = wb_gps_get_values(gps);
  ts_assert_doubles_in_delta(3, values, newGpsValues1, 0.00001, "Delayed GPS measurement after first pose update is wrong.");

  wb_robot_step_end();

  // check sensor values after second pose update

  // compass
  values = wb_compass_get_values(compass);
  const double newCompassValues2[3] = {1.0, 0.0, 0.0};
  ts_assert_doubles_in_delta(3, values, newCompassValues2, 0.0001, "Compass measurement after second pose update is wrong.");

  // distance sensor
  distance = wb_distance_sensor_get_value(ds);
  ts_assert_double_in_delta(distance, 1000, 0.001, "Distance sensor measurement after second pose update is wrong.");

  // gps
  values = wb_gps_get_values(gps);
  const double newGpsValues2[3] = {0.1, 0.0295, 0.1};
  ts_assert_doubles_in_delta(3, values, newGpsValues2, 0.00001, "GPS measurement after second pose update is wrong.");

  ts_send_success();
  return EXIT_SUCCESS;
}
