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

const bool expected_detection[N_DISTANCE_SENSORS] = {true,  false, false, false, true, true,  true,  true,  true,  true,
                                                     false, false, false, false, true, false, false, false, false, true};

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
    const bool detection = wb_distance_sensor_get_value(ds[i]) < NO_OBSTACLE;
    ts_assert_boolean_equal(detection != expected_detection[i],
                            "Distance sensor '%c' doesn't return the right distance when hitting an object", 'A' + i);
  }

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
