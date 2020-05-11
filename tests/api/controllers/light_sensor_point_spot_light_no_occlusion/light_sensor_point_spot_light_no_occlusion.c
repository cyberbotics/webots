#include <webots/light_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

// expected sensor values
#define LS0_VALUE 279.21
#define LS1_VALUE 102.4

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag ls0, ls1;
  int i;
  double r;

  ls0 = wb_robot_get_device("ls0");
  ls1 = wb_robot_get_device("ls1");

  int lookup_table_size = wb_light_sensor_get_lookup_table_size(ls0);
  ts_assert_double_equal(lookup_table_size, 2, "Lookup table size returned is wrong (%d instead of 2)", lookup_table_size);
  const double *lookup_table = wb_light_sensor_get_lookup_table(ls0);
  ts_assert_double_equal(lookup_table[3], 10, "Lookup table (index 3) returned is wrong (%lf instead of 10)", lookup_table[3]);
  ts_assert_double_equal(lookup_table[5], 0, "Lookup table (index 5) returned is wrong (%lf instead of 0)", lookup_table[5]);

  r = wb_light_sensor_get_value(ls0);
  ts_assert_double_equal(
    r, NAN, "The value measured by the light sensor \"ls0\" should be NaN and not %g before the device is enabled", r);

  r = wb_light_sensor_get_value(ls1);
  ts_assert_double_equal(
    r, NAN, "The value measured by the light sensor \"ls1\" should be NaN and not %g before the device is enabled", r);

  wb_light_sensor_enable(ls0, TIME_STEP);
  wb_light_sensor_enable(ls1, 2 * TIME_STEP);

  r = wb_light_sensor_get_value(ls0);
  ts_assert_double_equal(
    r, NAN, "The value measured by the light sensor \"ls0\" should be NaN and not %g before a wb_robot_step is performed", r);

  r = wb_light_sensor_get_value(ls1);
  ts_assert_double_equal(
    r, NAN, "The value measured by the light sensor \"ls1\" should be NaN and not %g before a wb_robot_step is performed", r);

  wb_robot_step(TIME_STEP);

  r = wb_light_sensor_get_value(ls0);
  ts_assert_double_in_delta(r, LS0_VALUE, 0.001,
                            "The value measured by the light sensor \"ls0\" should be %g and not %g after 1 wb_robot_step(s)",
                            LS0_VALUE, r);

  r = wb_light_sensor_get_value(ls1);
  ts_assert_double_equal(
    r, NAN, "The value measured by the light sensor \"ls1\" should be NaN and not %g after 1 wb_robot_step is performed", r);

  for (i = 2; i <= 10; i++) {
    wb_robot_step(TIME_STEP);
    r = wb_light_sensor_get_value(ls0);
    ts_assert_double_in_delta(
      r, LS0_VALUE, 0.001, "The value measured by the light sensor \"ls0\" should be %g and not %g after %d wb_robot_step(s)",
      LS0_VALUE, r, i);
    r = wb_light_sensor_get_value(ls1);
    ts_assert_double_in_delta(
      r, LS1_VALUE, 0.001, "The value measured by the light sensor \"ls0\" should be %g and not %g after %d wb_robot_step(s)",
      LS1_VALUE, r, i);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
