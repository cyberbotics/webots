#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // wb_supervisor_node_get_from_def tests
  WbNodeRef testA = wb_supervisor_node_get_from_def("TESTA");
  WbNodeRef testB = wb_supervisor_node_get_from_def("TESTA.TESTB");
  ts_assert_string_equal(wb_supervisor_node_get_def(testA), "TESTA", "testA's DEF name should be \"TESTA\"");
  ts_assert_string_equal(wb_supervisor_node_get_def(testB), "TESTB", "testB's DEF name should be \"TESTB\"");

  // check that root has empty DEF
  WbNodeRef root = wb_supervisor_node_get_root();
  ts_assert_string_equal(wb_supervisor_node_get_def(root), "", "The root node should have an empty DEF");

  // Scene tree traversal from root tests
  WbFieldRef rootChildren = wb_supervisor_node_get_field(root, "children");
  WbNodeRef testC = wb_supervisor_field_get_mf_node(rootChildren, 4);
  ts_assert_string_equal(wb_supervisor_node_get_def(testC), "TESTC", "testC's DEF name should be \"TESTC\"");

  // try traversing up to get DEF names
  WbNodeRef sup = wb_supervisor_node_get_self();
  WbNodeRef supParent = wb_supervisor_node_get_parent_node(sup);
  WbFieldRef supSiblings = wb_supervisor_node_get_field(supParent, "children");
  WbNodeRef testD = wb_supervisor_field_get_mf_node(supSiblings, 5);
  ts_assert_string_equal(wb_supervisor_node_get_def(testD), "TESTD", "testD's DEF name should be \"TESTD\"");

  // wb_supervisor_field_get_sf_node test
  WbNodeRef testEParent = wb_supervisor_field_get_mf_node(rootChildren, 0);
  WbFieldRef testEChildren = wb_supervisor_node_get_field(testEParent, "defaultDamping");
  WbNodeRef testE = wb_supervisor_field_get_sf_node(testEChildren);
  ts_assert_string_equal(wb_supervisor_node_get_def(testE), "TESTE", "testE's DEF name should be \"TESTE\"");

  // wb_supervisor_node_get_from_id test
  WbNodeRef testF = wb_supervisor_node_get_from_id(9);
  ts_assert_string_equal(wb_supervisor_node_get_def(testF), "TESTF", "testF's DEF name should be \"TESTF\"");

  ts_send_success();
  return EXIT_SUCCESS;
}
