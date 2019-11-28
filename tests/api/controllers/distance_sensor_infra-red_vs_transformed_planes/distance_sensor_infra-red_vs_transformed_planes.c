/*
 * Description:
 *
 * This controller tests infra-red DistanceSensor vs transformed Plane collisions.
 * Several collisions against 3 transformed planes are tested.
 * Note: It's regular that the flipped plane gives no feedback.
 */

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define N_DISTANCE_SENSORS 10

#define OBSTACLE 833.3333
#define NO_OBSTACLE 1000.0

const double expected_values[N_DISTANCE_SENSORS] = {NO_OBSTACLE, OBSTACLE, OBSTACLE,    OBSTACLE,    NO_OBSTACLE,
                                                    OBSTACLE,    OBSTACLE, NO_OBSTACLE, NO_OBSTACLE, NO_OBSTACLE};

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

  for (i = 0; i < N_DISTANCE_SENSORS; ++i) {
    const double value = wb_distance_sensor_get_value(ds[i]);
    ts_assert_double_in_delta(value, expected_values[i], 0.01,
                              "Distance sensor '%c' doesn't return the right distance when hitting an object "
                              "(expected = %f, received = %f)",
                              'A' + i, expected_values[i], value);
  }

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
