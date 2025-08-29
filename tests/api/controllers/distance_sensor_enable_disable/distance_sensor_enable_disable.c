#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  double value;
  double newPosition[3] = {-0.03, 0.0, 0.0};

  WbNodeRef robotNode = wb_supervisor_node_get_self();
  WbFieldRef translationField = wb_supervisor_node_get_field(robotNode, "translation");

  WbDeviceTag ds_generic = wb_robot_get_device("ds generic");
  WbDeviceTag ds_sonar = wb_robot_get_device("ds sonar");
  WbDeviceTag ds_infra_red = wb_robot_get_device("ds infra-red");
  WbDeviceTag ds_laser = wb_robot_get_device("ds laser");

  // test: get the value of a sensor which has not been enabled yet
  value = wb_distance_sensor_get_value(ds_generic);
  ts_assert_double_equal(value, NAN,
                         "The distance sensor in generic mode doesn't return NAN when it has not been enabled yet. %lf != %lf",
                         NAN, value);

  value = wb_distance_sensor_get_value(ds_sonar);
  ts_assert_double_equal(value, NAN, "The distance sensor in sonar mode doesn't return NAN when it has not been enabled yet.");

  value = wb_distance_sensor_get_value(ds_infra_red);
  ts_assert_double_equal(value, NAN,
                         "The distance sensor in infra-red mode doesn't return NAN when it has not been enabled yet.");

  value = wb_distance_sensor_get_value(ds_laser);
  ts_assert_double_equal(value, NAN, "The distance sensor in laser mode doesn't return NAN when it has not been enabled yet.");

  wb_distance_sensor_enable(ds_generic, 3 * TIME_STEP);
  wb_distance_sensor_enable(ds_sonar, TIME_STEP);
  wb_distance_sensor_enable(ds_infra_red, TIME_STEP);
  wb_distance_sensor_enable(ds_laser, TIME_STEP);

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds_sonar);
  ts_assert_double_in_delta(
    value, 500.0, 0.001,
    "The distance sensor in sonar mode doesn't return the right distance in the regular case. (%f but %f was expected)", value,
    500.0);

  value = wb_distance_sensor_get_value(ds_infra_red);
  ts_assert_double_in_delta(
    value, 500.0, 0.001,
    "The distance sensor in infra-red mode doesn't return the right distance in the regular case. (%f but %f was expected)",
    value, 500.0);

  value = wb_distance_sensor_get_value(ds_laser);
  ts_assert_double_in_delta(
    value, 500.0, 0.001,
    "The distance sensor in laser mode doesn't return the right distance in the regular case. (%f but %f was expected)", value,
    500.0);

  // test sensor value not available before first sampling period elapsed
  value = wb_distance_sensor_get_value(ds_generic);
  ts_assert_double_equal(value, NAN, "The distance sensor in generic mode should not yet be active during the first step.");

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds_generic);
  ts_assert_double_equal(value, NAN, "The distance sensor in generic mode should not yet be active during the second step.");

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds_generic);
  ts_assert_double_in_delta(
    value, 500.0, 0.001,
    "The distance sensor in generic mode doesn't return the right distance in the regular case. (%f but %f was expected)",
    value, 500.0);

  wb_supervisor_field_set_sf_vec3f(translationField, newPosition);

  // test: laser and sonar disabled -> last value returned
  //       infra-red enabled        -> return value changes next step
  //       generic enabled          -> return value changes in two steps
  wb_distance_sensor_disable(ds_sonar);
  wb_distance_sensor_disable(ds_laser);

  wb_robot_step(TIME_STEP);

  // test: get the value of an disabled sonar

  value = wb_distance_sensor_get_value(ds_sonar);
  ts_assert_double_in_delta(value, 500.0, 0.001,
                            "The distance sensor in sonar mode doesn't return the last measured distance when being disabled.");

  value = wb_distance_sensor_get_value(ds_laser);
  ts_assert_double_in_delta(value, 500.0, 0.001,
                            "The distance sensor in laser mode doesn't return the last measured distance when being disabled.");

  // test value updated
  value = wb_distance_sensor_get_value(ds_infra_red);
  ts_assert_double_in_delta(
    value, 800.0, 0.001,
    "The distance sensor in infra-red mode doesn't return the updated measured distance after translation.");

  // test value not updated yet
  value = wb_distance_sensor_get_value(ds_generic);
  ts_assert_double_in_delta(
    value, 500.0, 0.001,
    "The distance sensor in generic mode shouldn't update the measured distance the next step after translation.");

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds_generic);
  ts_assert_double_in_delta(
    value, 500.0, 0.001,
    "The distance sensor in generic mode shouldn't update the measured distance two steps after translation.");

  wb_robot_step(TIME_STEP);

  value = wb_distance_sensor_get_value(ds_generic);
  ts_assert_double_in_delta(
    value, 800.0, 0.001,
    "The distance sensor in generic mode doesn't return the updates measured distance three steps after translation.");

  ts_send_success();
  return EXIT_SUCCESS;
}
