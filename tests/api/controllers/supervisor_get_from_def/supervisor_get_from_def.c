#include <webots/robot.h>
#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 320

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef proto_node = wb_supervisor_node_get_from_def("SOLID_PROTO");
  ts_assert_pointer_not_null(proto_node, "wb_supervisor_node_get_from_def(\"SOLID_PROTO\") failed");

  WbNodeRef solid_node = wb_supervisor_node_get_from_def("SOLID_PROTO_PARAMETER");
  ts_assert_pointer_not_null(solid_node, "wb_supervisor_node_get_from_def(\"SOLID_PROTO_PARAMETER\") failed");

  WbNodeRef internal_group = wb_supervisor_node_get_from_def("INTERNAL_GROUP");
  ts_assert_pointer_null(internal_group, "wb_supervisor_node_get_from_def should not return internal nodes");

  internal_group = wb_supervisor_node_get_from_proto_def(proto_node, "INTERNAL_GROUP");
  ts_assert_pointer_not_null(internal_group, "wb_supervisor_node_get_from_proto_def should return internal nodes");

  WbNodeRef emitter_node = wb_supervisor_node_get_from_proto_def(proto_node, "EMITTER");
  ts_assert_pointer_null(emitter_node, "wb_supervisor_node_get_from_proto_def should not work for non-PROTO nodes");

  WbNodeRef bo_node = wb_supervisor_node_get_from_def("BO");
  ts_assert_pointer_not_null(bo_node, "wb_supervisor_node_get_from_def(\"BO\") failed");
  bool is_proto = wb_supervisor_node_is_proto(bo_node);
  ts_assert_boolean_not_equal(is_proto, "wb_supervisor_node_is_proto should return false for BO node");
  is_proto = wb_supervisor_node_is_proto(proto_node);
  ts_assert_boolean_equal(is_proto, "wb_supervisor_node_is_proto should return true for SOLID_PROTO node");
  is_proto = wb_supervisor_node_is_proto(internal_group);
  ts_assert_boolean_not_equal(is_proto, "wb_supervisor_node_is_proto should return false for INTERNAL_GROUP node");

  if (proto_node == solid_node)
    ts_assert_boolean_equal(false, "wb_supervisor_node_get_from_def(\"SOLID_PROTO_PARAMETER\") failed, reference to proto_node "
                                   "and solid_node should be equal.");

  const double *solid_position = wb_supervisor_node_get_position(solid_node);
  const double *proto_position = wb_supervisor_node_get_position(proto_node);

  ts_assert_doubles_in_delta(3, solid_position, proto_position, 0.000001,
                             "The position of both nodes should be equal at simulation start.");

  wb_robot_step(320);

  solid_position = wb_supervisor_node_get_position(solid_node);
  proto_position = wb_supervisor_node_get_position(proto_node);

  ts_assert_doubles_in_delta(3, solid_position, proto_position, 0.000001,
                             "The position of both nodes should be equal after some steps.");

  ts_send_success();
  return EXIT_SUCCESS;
}
