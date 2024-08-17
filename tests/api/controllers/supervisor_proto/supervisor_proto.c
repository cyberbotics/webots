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

static void assert_hierarchy_correct(WbProtoRef proto, const char **expected_hierarchy) {
  int i = 0;
  while (expected_hierarchy[i]) {
    const char *actual_name = wb_supervisor_proto_get_type_name(proto);
    ts_assert_pointer_not_null(actual_name, "Hierarchy mismatch! Expected \"%s\", but got NULL", expected_hierarchy[i]);
    ts_assert_string_equal(actual_name, expected_hierarchy[i], "Hierarchy mismatch! Expected \"%s\", but got %s", expected_hierarchy[i], actual_name);
    proto = wb_supervisor_proto_get_parent(proto);
    i++;
  }
  ts_assert_pointer_null(proto, "Hierarchy mismatch! Expected NULL, but got \"%s\"", wb_supervisor_proto_get_type_name(proto));
}

static void assert_fields_correct(WbProtoRef proto, const struct FieldDefinition ***expected_fields) {
  int i = 0;
  while (expected_fields[i]) {
    ts_assert_pointer_not_null(proto, "Expected more fields, but proto is null. The test should've failed when checking the hierarchy. This likely means that the expected hierarchy is incorrectly configured.");

    int number_of_fields = 0;
    while (expected_fields[i][number_of_fields])
      number_of_fields++;

    const char *proto_name = wb_supervisor_proto_get_type_name(proto);

    const int actual_number_of_fields = wb_supervisor_proto_get_number_of_parameters(proto);
    ts_assert_int_equal(actual_number_of_fields, number_of_fields, "Wrong number of fields in proto \"%s\". Expected %d, but got %d", proto_name, number_of_fields, actual_number_of_fields);

    for (int j = 0; j < number_of_fields; j++) {
      const char *expected_name = expected_fields[i][j]->name;
      const WbFieldType expected_type = expected_fields[i][j]->type;

      WbFieldRef field = wb_supervisor_proto_get_parameter_by_index(proto, j);
      ts_assert_pointer_not_null(field, "Field \"%d\" not found in proto \"%s\" despite the reported number of parameters being correct", j, proto_name);

      const char *actual_name = wb_supervisor_field_get_name(field);
      ts_assert_string_equal(actual_name, expected_name, "Field mismatch in proto \"%s\"! Expected \"%s\", but got \"%s\"", proto_name, expected_name, actual_name);

      const WbFieldType actual_type = wb_supervisor_field_get_type(field);
      ts_assert_int_equal(actual_type, expected_type, "Field \"%s\" in proto \"%s\" has wrong type! Expected %d, but got %d", expected_name, proto_name, expected_type, actual_type);
    }

    proto = wb_supervisor_proto_get_parent(proto);
  }
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // Check null values
  ts_assert_pointer_null(wb_supervisor_proto_get_type_name(NULL), "wb_supervisor_proto_get_type_name(NULL) should return NULL");
  ts_assert_pointer_null(wb_supervisor_proto_get_parent(NULL), "wb_supervisor_proto_get_parent(NULL) should return NULL");
  ts_assert_int_equal(wb_supervisor_proto_get_number_of_parameters(NULL), 0, "wb_supervisor_proto_get_number_of_parameters(NULL) should return 0");
  ts_assert_pointer_null(wb_supervisor_proto_get_parameter_by_index(NULL, 0), "wb_supervisor_proto_get_parameter_by_index(NULL, 0) should return NULL");
  ts_assert_pointer_null(wb_supervisor_proto_get_parameter_by_name(NULL, "name"), "wb_supervisor_proto_get_parameter_by_name(NULL, \"name\") should return NULL");

  // This array keeps track of the names of the proto types we expect at each level of the hierarchy,
  // starting with the proto type of the main hierarchy node, and ending with NULL.
  const char *expected_hierarchy[] = {"SolidProtoHierarchy", "SolidProtoInternal", "SolidProtoBase", NULL};
  // This array keeps track of the fields we expect at each level of the hierarchy.
  // Each element in this array is an array that represents a single proto type/
  // Each of these arrays contains the fields we expect to find in that type, in the order we expect them,
  // terminated with a NULL pointer.
  const struct FieldDefinition **expected_fields[] = {
    (struct FieldDefinition*[]){ // SolidProtoHierarchy
      FieldDefinition("translationStep", WB_SF_FLOAT),
      FieldDefinition("children", WB_MF_NODE),
      FieldDefinition("internalTranslationStep", WB_SF_FLOAT),
      FieldDefinition("internalField1", WB_SF_FLOAT),
      FieldDefinition("internalField2", WB_SF_FLOAT),
      FieldDefinition("ucField1", WB_SF_FLOAT),
      NULL
    },
    (struct FieldDefinition*[]){ // SolidProtoInternal
      FieldDefinition("translationStep", WB_SF_FLOAT),
      FieldDefinition("rotationStep", WB_SF_FLOAT),
      FieldDefinition("children", WB_MF_NODE),
      FieldDefinition("ucField1", WB_SF_FLOAT),
      FieldDefinition("ucField2", WB_SF_FLOAT),
      NULL
    },
    (struct FieldDefinition*[]){ // SolidProtoBase
      FieldDefinition("translationStep", WB_SF_FLOAT),
      FieldDefinition("rotationStep", WB_SF_FLOAT),
      FieldDefinition("children", WB_MF_NODE),
      NULL
    },
    NULL
  };

  WbNodeRef hierarchy = wb_supervisor_node_get_from_def("HIERARCHY");
  WbNodeRef internal_node = wb_supervisor_node_get_from_def("INTERNAL_HIERARCHY");

  WbProtoRef hierarchy_proto = wb_supervisor_node_get_proto(hierarchy);
  WbProtoRef internal_proto = wb_supervisor_node_get_proto(internal_node);

  assert_hierarchy_correct(hierarchy_proto, expected_hierarchy);
  assert_fields_correct(hierarchy_proto, expected_fields);
  // Because the main hierarchy node and the internal node are both of type SolidProtoInternal,
  // we can just skip the first element of both arrays when validating the internal node
  assert_hierarchy_correct(internal_proto, expected_hierarchy + 1);
  assert_fields_correct(internal_proto, expected_fields + 1);

  ts_send_success();
  return EXIT_SUCCESS;
}
