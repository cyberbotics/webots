#include <webots/gyro.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag motor = wb_robot_get_device("rotational motor");
  WbDeviceTag gyro = wb_robot_get_device("gyro");

  wb_motor_set_position(motor, INFINITY);
  wb_motor_set_velocity(motor, 10.0);

  wb_gyro_enable(gyro, TIME_STEP);

  int i;
  for (i = 0; i < 40; i++)
    wb_robot_step(TIME_STEP);

  int lookup_table_size = wb_gyro_get_lookup_table_size(gyro);
  ts_assert_double_equal(lookup_table_size, 2, "Lookup table size returned is wrong (%d instead of 2)", lookup_table_size);
  const double *lookup_table = wb_gyro_get_lookup_table(gyro);
  ts_assert_double_equal(lookup_table[3], 1000, "Lookup table (index 3) returned is wrong (%lf instead of 1000)",
                         lookup_table[3]);
  ts_assert_double_equal(lookup_table[5], 0, "Lookup table (index 5) returned is wrong (%lf instead of 0)", lookup_table[5]);

  const double *values = wb_gyro_get_values(gyro);

  const double expected[] = {0.0, 10.0, 0.0};

  printf("expected = {%f, %f, %f}\n", expected[0], expected[1], expected[2]);
  printf("values = {%f, %f, %f}\n", values[0], values[1], values[2]);
  ts_assert_vec3_in_delta(values[0], values[1], values[2], expected[0], expected[1], expected[2], 0.0001,
                          "The Gyro doesn't return the right angular velocity."
                          " (expected = {%f, %f, %f}, received = {%f, %f, %f})",
                          expected[0], expected[1], expected[2], values[0], values[1], values[2]);

  ts_send_success();
  return EXIT_SUCCESS;
}
