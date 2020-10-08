#include <webots/compass.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag compass = wb_robot_get_device("compass");

  wb_compass_enable(compass, TIME_STEP);

  wb_robot_step(TIME_STEP);

  int lookup_table_size = wb_compass_get_lookup_table_size(compass);
  ts_assert_double_equal(lookup_table_size, 2, "Lookup table size returned is wrong (%d instead of 2)", lookup_table_size);
  const double *lookup_table = wb_compass_get_lookup_table(compass);
  ts_assert_double_equal(lookup_table[3], 1000, "Lookup table (index 3) returned is wrong (%lf instead of 1000)",
                         lookup_table[3]);
  ts_assert_double_equal(lookup_table[5], 0, "Lookup table (index 5) returned is wrong (%lf instead of 0)", lookup_table[5]);

  const double *values = wb_compass_get_values(compass);

  const double expected[] = {0.258819, 0, 0.965926};

  int i;
  for (i = 0; i < 3; i++)
    ts_assert_double_in_delta(values[i], expected[i], 0.0001, "The compass doesn't return the right north direction.");

  WbNodeRef worldinfo_node = wb_supervisor_node_get_from_def("WORLD_INFO");
  WbFieldRef coordinate_system_field = wb_supervisor_node_get_field(worldinfo_node, "coordinateSystem");

  wb_supervisor_field_set_sf_string(coordinate_system_field, "EUN");
  for (i = 0; i < 3; i++)
    wb_robot_step(TIME_STEP);

  values = wb_compass_get_values(compass);

  ts_assert_double_in_delta(values[0], -expected[2], 0.0001,
                            "The compass doesn't return the right north direction after changing the coordinate system.");
  ts_assert_double_in_delta(values[1], expected[1], 0.0001,
                            "The compass doesn't return the right north direction after changing the coordinate system.");
  ts_assert_double_in_delta(values[2], expected[0], 0.0001,
                            "The compass doesn't return the right north direction after changing the coordinate system.");

  ts_send_success();
  return EXIT_SUCCESS;
}
