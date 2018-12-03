#include <webots/gps.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag gps;
  int i;
  double new_york_latitude = 40.67;
  double new_york_altitude = 10;
  double new_york_longitude = -73.94;
  const char *expected_string = "-73° -56′ -23″";

  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  int coordinate_system = wb_gps_get_coordinate_system(gps);
  ts_assert_int_equal(coordinate_system, WB_GPS_WGS84_COORDINATE, "Wrong coordinate system returned");

  wb_robot_step(TIME_STEP);

  const double *position = wb_gps_get_values(gps);
  ts_assert_double_in_delta(position[0], new_york_latitude, 0.1, "The latitude measured by the GPS should be %f and not %f",
                            new_york_latitude, position[0]);
  ts_assert_double_in_delta(position[2], new_york_altitude, 0.1, "The altitude measured by the GPS should be %f and not %f",
                            new_york_altitude, position[1]);
  ts_assert_double_in_delta(position[1], new_york_longitude, 0.1, "The longitude measured by the GPS should be %f and not %f",
                            new_york_longitude, position[2]);

  for (i = 0; i < 20; ++i)
    wb_robot_step(TIME_STEP);

  const double speed = wb_gps_get_speed(gps);
  ts_assert_double_is_bigger(speed, 0.005, "The speed returned by the GPS should be bigger than 0.005m/s (%f returned)", speed);

  const char *coordinate_converted = wb_gps_convert_to_degrees_minutes_seconds(new_york_longitude);
  ts_assert_string_equal(coordinate_converted, expected_string,
                         "The coordinate '%lf' should be equal to '%s' in the convert degree/minutes/seconds and not '%s'",
                         new_york_longitude, expected_string, coordinate_converted);
  free((void *)coordinate_converted);

  ts_send_success();
  return EXIT_SUCCESS;
}
