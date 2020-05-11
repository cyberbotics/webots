#include <webots/inertial_unit.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define N 7

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  wb_robot_step(TIME_STEP);

  WbNodeRef plane = wb_supervisor_node_get_self();
  WbFieldRef rotation = wb_supervisor_node_get_field(plane, "rotation");

  WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);

  double problems[N][4] = {{0, 1, 0, 0},       {0, 1, 0, -1.5708}, {0, 1, 0, 1.5708}, {1, 0, 0, 1.5708},
                           {1, 0, 0, -1.5708}, {0, 0, 1, 0.7854},  {0, 0, 1, -0.7854}};

  double solutions[N][3] = {
    {0, 0, 0}, {0, 0, -1.5708}, {0, 0, 1.5708}, {1.5708, 0, 0}, {-1.5708, 0, 0}, {0, 0.7854, 0}, {0, -0.7854, 0},
  };

  int lookup_table_size = wb_inertial_unit_get_lookup_table_size(inertial_unit);
  ts_assert_double_equal(lookup_table_size, 2, "Lookup table size returned is wrong (%d instead of 2)", lookup_table_size);
  const double *lookup_table = wb_inertial_unit_get_lookup_table(inertial_unit);
  ts_assert_double_equal(lookup_table[3], 1000, "Lookup table (index 3) returned is wrong (%lf instead of 1000)",
                         lookup_table[3]);
  ts_assert_double_equal(lookup_table[5], 0, "Lookup table (index 5) returned is wrong (%lf instead of 0)", lookup_table[5]);

  int i;
  for (i = 0; i < N; i++) {
    wb_supervisor_field_set_sf_rotation(rotation, problems[i]);
    wb_robot_step(TIME_STEP);
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    ts_assert_doubles_in_delta(3, rpy, solutions[i], 0.0001,
                               "Getting roll-pitch-yaw failed after %d steps (received = {%f, %f, %f}, expected = {%f, %f, %f}",
                               i, rpy[0], rpy[1], rpy[2], solutions[i][0], solutions[i][1], solutions[i][2]);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
