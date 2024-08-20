#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int i = 0;
  WbNodeRef node = wb_supervisor_node_get_from_def("NODE");
  WbProtoRef baseProto = wb_supervisor_proto_get_parent(wb_supervisor_node_get_proto(node));
  const int FIELD_COUNT = 8;
  WbFieldRef mfField[FIELD_COUNT];
  mfField[0] = wb_supervisor_node_get_field(node, "bool");
  mfField[1] = wb_supervisor_node_get_field(node, "int");
  mfField[2] = wb_supervisor_node_get_field(node, "float");
  mfField[3] = wb_supervisor_node_get_field(node, "vec2");
  mfField[4] = wb_supervisor_node_get_field(node, "vec3");
  mfField[5] = wb_supervisor_node_get_field(node, "rot");
  mfField[6] = wb_supervisor_node_get_field(node, "color");
  mfField[7] = wb_supervisor_node_get_field(node, "string");
  WbFieldRef mfProtoFields[FIELD_COUNT];
  mfProtoFields[0] = wb_supervisor_proto_get_field(baseProto, "bool");
  mfProtoFields[1] = wb_supervisor_proto_get_field(baseProto, "int");
  mfProtoFields[2] = wb_supervisor_proto_get_field(baseProto, "float");
  mfProtoFields[3] = wb_supervisor_proto_get_field(baseProto, "vec2");
  mfProtoFields[4] = wb_supervisor_proto_get_field(baseProto, "vec3");
  mfProtoFields[5] = wb_supervisor_proto_get_field(baseProto, "rot");
  mfProtoFields[6] = wb_supervisor_proto_get_field(baseProto, "color");
  mfProtoFields[7] = wb_supervisor_proto_get_field(baseProto, "string");
  int mfFieldCount[FIELD_COUNT];
  for (i = 0; i < FIELD_COUNT; ++i) {
    const int count = wb_supervisor_field_get_count(mfField[i]);
    mfFieldCount[i] = count;
    const int protoCount = wb_supervisor_field_get_count(mfProtoFields[i]);
    ts_assert_int_equal(protoCount, count,
                        "Size of proto field %d not correctly initialized: found %d, expected %d", i, count, count);
  }

  wb_robot_step(3 * TIME_STEP);

  for (i = 0; i < FIELD_COUNT; ++i) {
    wb_robot_step(TIME_STEP);
    const int count = wb_supervisor_field_get_count(mfField[i]);
    int increment = i < 7 ? 1 : 2;
    ts_assert_int_equal(count, mfFieldCount[i] + increment,
                        "Size of field %d not correctly updated after item inserted: found %d, expected %d", i, count,
                        mfFieldCount[i] + increment);
    const int protoCount = wb_supervisor_field_get_count(mfProtoFields[i]);
    ts_assert_int_equal(protoCount, mfFieldCount[i] + increment,
                        "Size of proto field %d not correctly updated after item inserted: found %d, expected %d", i,
                        protoCount, mfFieldCount[i] + increment);
  }

  for (i = FIELD_COUNT - 1; i >= 0; --i) {
    wb_robot_step(TIME_STEP);
    const int count = wb_supervisor_field_get_count(mfField[i]);
    int increment = i < 7 ? 0 : 1;
    ts_assert_int_equal(count, mfFieldCount[i] + increment,
                        "Size of field %d not correctly updated after item removed: found %d, expected %d", i, count,
                        mfFieldCount[i] + increment);
    const int protoCount = wb_supervisor_field_get_count(mfProtoFields[i]);
    ts_assert_int_equal(protoCount, mfFieldCount[i] + increment,
                        "Size of proto field %d not correctly updated after item removed: found %d, expected %d", i,
                        protoCount, mfFieldCount[i] + increment);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
