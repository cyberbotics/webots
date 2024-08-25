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
  WbProtoRef base_proto = wb_supervisor_node_get_proto(node);
  const int FIELD_COUNT = 8;
  WbFieldRef mf_field[FIELD_COUNT];
  mf_field[0] = wb_supervisor_node_get_field(node, "bool");
  mf_field[1] = wb_supervisor_node_get_field(node, "int");
  mf_field[2] = wb_supervisor_node_get_field(node, "float");
  mf_field[3] = wb_supervisor_node_get_field(node, "vec2");
  mf_field[4] = wb_supervisor_node_get_field(node, "vec3");
  mf_field[5] = wb_supervisor_node_get_field(node, "rot");
  mf_field[6] = wb_supervisor_node_get_field(node, "color");
  mf_field[7] = wb_supervisor_node_get_field(node, "string");
  WbFieldRef mf_proto_fields[FIELD_COUNT];
  mf_proto_fields[0] = wb_supervisor_proto_get_field(base_proto, "bool");
  mf_proto_fields[1] = wb_supervisor_proto_get_field(base_proto, "int");
  mf_proto_fields[2] = wb_supervisor_proto_get_field(base_proto, "float");
  mf_proto_fields[3] = wb_supervisor_proto_get_field(base_proto, "vec2");
  mf_proto_fields[4] = wb_supervisor_proto_get_field(base_proto, "vec3");
  mf_proto_fields[5] = wb_supervisor_proto_get_field(base_proto, "rot");
  mf_proto_fields[6] = wb_supervisor_proto_get_field(base_proto, "color");
  mf_proto_fields[7] = wb_supervisor_proto_get_field(base_proto, "string");
  int mf_field_count[FIELD_COUNT];
  for (i = 0; i < FIELD_COUNT; ++i) {
    const int count = wb_supervisor_field_get_count(mf_field[i]);
    mf_field_count[i] = count;
    const int proto_count = wb_supervisor_field_get_count(mf_proto_fields[i]);
    ts_assert_int_equal(proto_count, count, "Size of proto field %d not correctly initialized: found %d, expected %d", i, count,
                        count);
  }

  wb_robot_step(3 * TIME_STEP);

  for (i = 0; i < FIELD_COUNT; ++i) {
    wb_robot_step(TIME_STEP);
    const int count = wb_supervisor_field_get_count(mf_field[i]);
    int increment = i < 7 ? 1 : 2;
    ts_assert_int_equal(count, mf_field_count[i] + increment,
                        "Size of field %d not correctly updated after item inserted: found %d, expected %d", i, count,
                        mf_field_count[i] + increment);

    // The proto node should have been regenerated, so all the references should be invalidated
    ts_assert_pointer_null(wb_supervisor_proto_get_type_name(base_proto),
                           "Proto node should have been regenerated after field insertion.");
    ts_assert_int_equal(wb_supervisor_field_get_count(mf_proto_fields[i]), -1,
                        "Proto field %d should have been invalidated after field insertion.", i);
    base_proto = wb_supervisor_node_get_proto(node);
    // Note: this assumes that the fields are defined in the same order in the proto as they are in this test
    mf_proto_fields[i] = wb_supervisor_proto_get_field_by_index(base_proto, i);

    const int proto_count = wb_supervisor_field_get_count(mf_proto_fields[i]);
    ts_assert_int_equal(proto_count, mf_field_count[i] + increment,
                        "Size of proto field %d not correctly updated after item inserted: found %d, expected %d", i,
                        proto_count, mf_field_count[i] + increment);
    wb_robot_step(TIME_STEP);
  }

  for (i = FIELD_COUNT - 1; i >= 0; --i) {
    wb_robot_step(TIME_STEP);
    const int count = wb_supervisor_field_get_count(mf_field[i]);
    int increment = i < 7 ? 0 : 1;
    ts_assert_int_equal(count, mf_field_count[i] + increment,
                        "Size of field %d not correctly updated after item removed: found %d, expected %d", i, count,
                        mf_field_count[i] + increment);

    // The proto node should have been regenerated, so all the references should be invalidated
    ts_assert_pointer_null(wb_supervisor_proto_get_type_name(base_proto),
                           "Proto node should have been regenerated after field removal.");
    ts_assert_int_equal(wb_supervisor_field_get_count(mf_proto_fields[i]), -1,
                        "Proto field %d should have been invalidated after field removal.", i);
    base_proto = wb_supervisor_node_get_proto(node);
    // Note: this assumes that the fields are defined in the same order in the proto as they are in this test
    mf_proto_fields[i] = wb_supervisor_proto_get_field_by_index(base_proto, i);

    const int proto_count = wb_supervisor_field_get_count(mf_proto_fields[i]);
    ts_assert_int_equal(proto_count, mf_field_count[i] + increment,
                        "Size of proto field %d not correctly updated after item removed: found %d, expected %d", i,
                        proto_count, mf_field_count[i] + increment);
    wb_robot_step(TIME_STEP);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
