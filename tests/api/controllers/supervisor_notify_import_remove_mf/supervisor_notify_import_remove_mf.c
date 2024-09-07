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
  WbProtoRef proto = wb_supervisor_node_get_proto(node);
  const char *field_names[] = {"bool", "int", "float", "vec2", "vec3", "rot", "color", "string"};
  const int FIELD_COUNT = sizeof(field_names) / sizeof(field_names[0]);
  WbFieldRef mf_field[FIELD_COUNT];
  WbFieldRef mf_proto_fields[FIELD_COUNT];
  for (i = 0; i < FIELD_COUNT; ++i) {
    mf_field[i] = wb_supervisor_node_get_field(node, field_names[i]);
    mf_proto_fields[i] = wb_supervisor_proto_get_field(proto, field_names[i]);
  }

  int mf_field_count[FIELD_COUNT];
  for (i = 0; i < FIELD_COUNT; ++i) {
    const int count = wb_supervisor_field_get_count(mf_field[i]);
    mf_field_count[i] = count;
    const int proto_count = wb_supervisor_field_get_count(mf_proto_fields[i]);
    ts_assert_int_equal(proto_count, count, "Size of proto field %d not correctly initialized: found %d, expected %d", i,
                        proto_count, count);
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
    ts_assert_string_equal(wb_supervisor_proto_get_type_name(proto), "",
                           "Proto node should have been regenerated after field insertion.");
    ts_assert_int_equal(wb_supervisor_field_get_count(mf_proto_fields[i]), -1,
                        "Proto field %d should have been invalidated after field insertion.", i);
    proto = wb_supervisor_node_get_proto(node);
    mf_proto_fields[i] = wb_supervisor_proto_get_field(proto, field_names[i]);

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
    ts_assert_string_equal(wb_supervisor_proto_get_type_name(proto), "",
                           "Proto node should have been regenerated after field removal.");
    ts_assert_int_equal(wb_supervisor_field_get_count(mf_proto_fields[i]), -1,
                        "Proto field %d should have been invalidated after field removal.", i);
    proto = wb_supervisor_node_get_proto(node);
    mf_proto_fields[i] = wb_supervisor_proto_get_field(proto, field_names[i]);

    const int proto_count = wb_supervisor_field_get_count(mf_proto_fields[i]);
    ts_assert_int_equal(proto_count, mf_field_count[i] + increment,
                        "Size of proto field %d not correctly updated after item removed: found %d, expected %d", i,
                        proto_count, mf_field_count[i] + increment);
    wb_robot_step(TIME_STEP);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
