/*
 * Description:  Test Supervisor API
 *               This file tests the basic functionalities of the proto api
 *               such as reading the proto hierarchy and verifying the fields
 *               of the proto nodes.
 *               Because this test shares the world with supervisor_proto_fields,
 *               which tests the ability to modify the fields of a proto node,
 *               it should not assume or modify the field values.
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

struct FieldDefinition {
  const char *name;
  WbFieldType type;
};

static void assert_hierarchy_correct(WbProtoRef proto, const char **expected_hierarchy, const char *node_name) {
  for (int i = 0; expected_hierarchy[i]; ++i) {
    const char *actual_name = wb_supervisor_proto_get_type_name(proto);
    ts_assert_pointer_not_null(actual_name, "(%s) Hierarchy mismatch! Expected \"%s\", but got NULL", node_name,
                               expected_hierarchy[i]);
    ts_assert_string_equal(actual_name, expected_hierarchy[i], "(%s) Hierarchy mismatch! Expected \"%s\", but got \"%s\"",
                           node_name, expected_hierarchy[i], actual_name);

    if (expected_hierarchy[i + 1])
      ts_assert_boolean_equal(wb_supervisor_proto_is_derived(proto),
                              "(%s) wb_supervisor_proto_is_derived() should return true for \"%s\"", node_name, actual_name);
    else
      ts_assert_boolean_not_equal(wb_supervisor_proto_is_derived(proto),
                                  "(%s) wb_supervisor_proto_is_derived() should return false for \"%s\"", node_name,
                                  actual_name);

    proto = wb_supervisor_proto_get_parent(proto);
  }
  ts_assert_pointer_null(proto, "(%s) Hierarchy mismatch! Expected NULL, but got \"%s\"", node_name,
                         wb_supervisor_proto_get_type_name(proto));
}

static void assert_fields_correct(WbProtoRef proto, const struct FieldDefinition **expected_fields, const char *node_name) {
  for (int i = 0; expected_fields[i]; ++i) {
    ts_assert_pointer_not_null(proto,
                               "(%s) Expected more fields, but proto is null. The test should've failed when checking the "
                               "hierarchy. This likely means that the expected values are incorrectly configured.",
                               node_name);

    int number_of_fields = 0;
    while (expected_fields[i][number_of_fields].type != WB_NO_FIELD)
      ++number_of_fields;

    const char *proto_name = wb_supervisor_proto_get_type_name(proto);

    const int actual_number_of_fields = wb_supervisor_proto_get_number_of_fields(proto);
    ts_assert_int_equal(actual_number_of_fields, number_of_fields,
                        "(%s) Wrong number of fields in proto \"%s\". Expected %d, but got %d", node_name, proto_name,
                        number_of_fields, actual_number_of_fields);

    for (int j = 0; j < number_of_fields; ++j) {
      const char *expected_name = expected_fields[i][j].name;
      const WbFieldType expected_type = expected_fields[i][j].type;

      WbFieldRef field = wb_supervisor_proto_get_field_by_index(proto, j);
      ts_assert_pointer_not_null(
        field, "(%s) Field \"%d\" not found in proto \"%s\" despite the reported number of fields being correct", node_name, j,
        proto_name);

      const char *actual_name = wb_supervisor_field_get_name(field);
      ts_assert_string_equal(actual_name, expected_name, "(%s) Field mismatch in proto \"%s\"! Expected \"%s\", but got \"%s\"",
                             node_name, proto_name, expected_name, actual_name);

      const WbFieldType actual_type = wb_supervisor_field_get_type(field);
      ts_assert_int_equal(actual_type, expected_type,
                          "(%s) Field \"%s\" in proto \"%s\" has wrong type! Expected %d, but got %d", node_name, expected_name,
                          proto_name, expected_type, actual_type);
    }

    proto = wb_supervisor_proto_get_parent(proto);
  }
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // Check null values
  ts_assert_pointer_null(wb_supervisor_node_get_proto(NULL), "wb_supervisor_node_get_proto(NULL) should return NULL");
  ts_assert_string_equal(wb_supervisor_proto_get_type_name(NULL), "",
                         "wb_supervisor_proto_get_type_name(NULL) should return NULL");
  ts_assert_boolean_not_equal(wb_supervisor_proto_is_derived(NULL), "wb_supervisor_proto_is_derived(NULL) should return false");
  ts_assert_pointer_null(wb_supervisor_proto_get_parent(NULL), "wb_supervisor_proto_get_parent(NULL) should return NULL");
  ts_assert_int_equal(wb_supervisor_proto_get_number_of_fields(NULL), -1,
                      "wb_supervisor_proto_get_number_of_fields(NULL) should return -1");
  ts_assert_pointer_null(wb_supervisor_proto_get_field_by_index(NULL, 0),
                         "wb_supervisor_proto_get_field_by_index(NULL, 0) should return NULL");
  ts_assert_pointer_null(wb_supervisor_proto_get_field(NULL, "name"),
                         "wb_supervisor_proto_get_field(NULL, \"name\") should return NULL");

  // This array keeps track of the names of the proto types we expect at each level of the hierarchy,
  // starting with the proto type of the main hierarchy node, and ending with NULL.
  const char *expected_hierarchy[] = {"SolidProtoHierarchy", "SolidProtoHierarchyInternal", "SolidProtoHierarchyBase", NULL};
  // This array keeps track of the fields we expect at each level of the hierarchy.
  // Each element in this array is an array that represents a single proto type
  // Each of these arrays contains the fields we expect to find in that type, in the order we expect them,
  // terminated with a field of type WB_NO_FIELD.
  const struct FieldDefinition *expected_fields[] = {(const struct FieldDefinition[]){// SolidProtoHierarchy
                                                                                      {"translationStep", WB_SF_FLOAT},
                                                                                      {"internalTranslationStep", WB_SF_FLOAT},
                                                                                      {"internalField1", WB_SF_FLOAT},
                                                                                      {"internalField2", WB_SF_FLOAT},
                                                                                      {"ucField4", WB_SF_FLOAT},
                                                                                      {NULL, WB_NO_FIELD}},
                                                     (const struct FieldDefinition[]){// SolidProtoInternal
                                                                                      {"translationStep", WB_SF_FLOAT},
                                                                                      {"rotationStep", WB_SF_FLOAT},
                                                                                      {"children", WB_MF_NODE},
                                                                                      {"ucField1", WB_SF_FLOAT},
                                                                                      {"ucField2", WB_SF_FLOAT},
                                                                                      {"ucField3", WB_SF_FLOAT},
                                                                                      {NULL, WB_NO_FIELD}},
                                                     (const struct FieldDefinition[]){// SolidProtoBase
                                                                                      {"translationStep", WB_SF_FLOAT},
                                                                                      {"rotationStep", WB_SF_FLOAT},
                                                                                      {"children", WB_MF_NODE},
                                                                                      {"ucField1", WB_SF_FLOAT},
                                                                                      {NULL, WB_NO_FIELD}},
                                                     NULL};

  WbNodeRef hierarchy = wb_supervisor_node_get_from_def("HIERARCHY");
  ts_assert_pointer_not_null(hierarchy, "Hierarchy node not found");
  WbNodeRef internal_node = wb_supervisor_node_get_from_proto_def(hierarchy, "INTERNAL_NODE");
  ts_assert_pointer_not_null(internal_node, "Internal node not found");

  WbProtoRef hierarchy_proto = wb_supervisor_node_get_proto(hierarchy);
  WbProtoRef internal_proto = wb_supervisor_node_get_proto(internal_node);

  assert_hierarchy_correct(hierarchy_proto, expected_hierarchy, "HIERARCHY");
  assert_fields_correct(hierarchy_proto, expected_fields, "HIERARCHY");
  // Because the main hierarchy node and the internal node are both of type SolidProtoInternal,
  // we can just skip the first element of both arrays when validating the internal node
  assert_hierarchy_correct(internal_proto, expected_hierarchy + 1, "INTERNAL_NODE");
  assert_fields_correct(internal_proto, expected_fields + 1, "INTERNAL_NODE");

  ts_send_success();
  return EXIT_SUCCESS;
}
