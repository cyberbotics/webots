#include <webots/display.h>
#include <webots/gps.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag display = wb_robot_get_device("display");
  WbDeviceTag gps = wb_robot_get_device("gps");

  // sending an image from libController to Webots using the Display device
  // imply that several packets are sent at the same time
  wb_display_image_load(display, "speedometer.png");

  // enabling a device after the packet
  wb_gps_enable(gps, TIME_STEP);

  // send the commands
  wb_robot_step(TIME_STEP);

  // get the results
  const double *values = wb_gps_get_values(gps);
  ts_assert_boolean_not_equal(values[0] != values[0],  // i.e. values[0] = NAN
                              "The device enable command has not been taken into account quickly enough.");

  ts_send_success();
  return EXIT_SUCCESS;
}
