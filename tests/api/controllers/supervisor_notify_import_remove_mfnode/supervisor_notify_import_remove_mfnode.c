#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbFieldRef root_children = wb_supervisor_node_get_field(wb_supervisor_node_get_root(), "children");

  const int node_count = 10;
  ts_assert_int_equal(wb_supervisor_field_get_count(root_children), node_count, "Intial number of root nodes is wrong");

  wb_robot_step(5 * TIME_STEP);

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children), node_count,
                      "Wrong number of root nodes after inserting SPHERE1.");

  wb_robot_step(2 * TIME_STEP);

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children), node_count - 1,
                      "Wrong number of root nodes after removing SPHERE1.");

  wb_robot_step(TIME_STEP);

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children), node_count,
                      "Wrong number of root nodes after inserting SPHERE2.");
  WbNodeRef sphere2_node = wb_supervisor_field_get_mf_node(root_children, 5);
  const char *def_name = wb_supervisor_node_get_def(sphere2_node);
  ts_assert_string_equal(def_name, "SPHERE2", "Wrong DEF name for top node at index 5: found %s, expected SPHERE2.", def_name);

  wb_robot_step(2 * TIME_STEP);

  ts_assert_int_equal(wb_supervisor_field_get_count(root_children), node_count - 1,
                      "Wrong number of root nodes after removing SPHERE2.");

  ts_send_success();
  return EXIT_SUCCESS;
}
