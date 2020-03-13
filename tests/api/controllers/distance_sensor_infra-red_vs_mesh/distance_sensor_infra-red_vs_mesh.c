/*
 * Description:
 *
 * This controller tests infra-red DistanceSensor vs Meshes.
 */

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define N_DISTANCE_SENSORS 20

#define NO_OBSTACLE 1000.0

const double expected_values[N_DISTANCE_SENSORS] = {
  NO_OBSTACLE, 717.74, 384.29, 87.68,  NO_OBSTACLE, NO_OBSTACLE, NO_OBSTACLE, NO_OBSTACLE, NO_OBSTACLE, NO_OBSTACLE,
  653.52,      413.15, 476.33, 770.60, NO_OBSTACLE, 714.71,      420.45,      458.82,      699.19,      NO_OBSTACLE};

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag ds[N_DISTANCE_SENSORS];
  int i;
  char name[2] = "A";
  for (i = 0; i < N_DISTANCE_SENSORS; ++i) {
    ds[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
    name[0] += 1;
  }

  wb_robot_step(TIME_STEP);

  double dsV[N_DISTANCE_SENSORS];

  for (i = 0; i < N_DISTANCE_SENSORS; ++i)
    dsV[i] = wb_distance_sensor_get_value(ds[i]);

  for (i = 0; i < N_DISTANCE_SENSORS; ++i) {
    const double value = wb_distance_sensor_get_value(ds[i]);
    ts_assert_double_in_delta(
      value, expected_values[i], 10.0,
      "Distance sensor '%c' doesn't return the right distance when hitting an object "
      "(expected = %f, received = %f) %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
      'A' + i, expected_values[i], value, dsV[0], dsV[1], dsV[2], dsV[3], dsV[4], dsV[5], dsV[6], dsV[7], dsV[8], dsV[9],
      dsV[10], dsV[11], dsV[12], dsV[13], dsV[14], dsV[15], dsV[16], dsV[17], dsV[18], dsV[19]);
  }

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
