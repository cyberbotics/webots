#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag ds[5];
  int i;
  char name[2] = "A";
  for (i = 0; i < 5; ++i) {
    ds[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
    name[0] += 1;
  }

  for (i = 0; i < 10; ++i) {
    printf("Values:\n");
    for (i = 0; i < 5; ++i) {
      double value = wb_distance_sensor_get_value(ds[i]);
      printf("%.1f - ", value);
    }
    printf("\n\n");

    wb_robot_step(TIME_STEP);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
