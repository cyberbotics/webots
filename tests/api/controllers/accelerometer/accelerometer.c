#include <webots/accelerometer.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  enum { CENTER = 0, TX, TY, TZ, CENTER_OFFSET, TX_OFFSET, TY_OFFSET, TZ_OFFSET };
  WbDeviceTag accelerometer[8];
  const int N = sizeof(accelerometer) / sizeof(WbDeviceTag);

  accelerometer[CENTER] = wb_robot_get_device("center");
  accelerometer[TX] = wb_robot_get_device("tX");
  accelerometer[TY] = wb_robot_get_device("tY");
  accelerometer[TZ] = wb_robot_get_device("tZ");
  accelerometer[CENTER_OFFSET] = wb_robot_get_device("center offset");
  accelerometer[TX_OFFSET] = wb_robot_get_device("tX offset");
  accelerometer[TY_OFFSET] = wb_robot_get_device("tY offset");
  accelerometer[TZ_OFFSET] = wb_robot_get_device("tZ offset");
  for (int i = 0; i < N; i++)
    wb_accelerometer_enable(accelerometer[i], TIME_STEP);

  WbDeviceTag motor = wb_robot_get_device("motor");
  wb_motor_set_position(motor, INFINITY);
  wb_motor_set_velocity(motor, 3.1415927);  // 180° per second

  wb_robot_step(TIME_STEP * 5);

  int lookup_table_size = wb_accelerometer_get_lookup_table_size(accelerometer[CENTER]);
  ts_assert_double_equal(lookup_table_size, 2, "Lookup table size returned is wrong (%d instead of 2)", lookup_table_size);
  const double *lookup_table = wb_accelerometer_get_lookup_table(accelerometer[CENTER]);
  ts_assert_double_equal(lookup_table[3], 1000, "Lookup table (index 3) returned is wrong (%lf instead of 1000)",
                         lookup_table[3]);
  ts_assert_double_equal(lookup_table[5], 0, "Lookup table (index 5) returned is wrong (%lf instead of 0)", lookup_table[5]);

  const double *values[N];
  for (int i = 0; i < N; i++)
    values[i] = wb_accelerometer_get_values(accelerometer[i]);

  const double expected[][3] = {{0, 9.8, 0},     {-1, 9.8, -0.1}, {0, 9.8, 0},     {0, 9.8, -1},
                                {-1, 9.8, -0.1}, {-2, 9.8, -0.2}, {-1, 9.8, -0.1}, {-1, 9.8, -1.1}};
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < 3; j++)
      ts_assert_double_in_delta(values[i][j], expected[i][j], 0.1, "The Accelerometer doesn't return the right acceleration.");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
