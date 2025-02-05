/*
 * Description:  Test Supervisor API
 *               This file tests the ability to read and set the values of a proto node's fields.
 *               For simplicity, all fields are SFFloats. This test only needs to check that the
 *               appropriate access restrictions are in place for proto fields. The actual get/set
 *               functionality for other types of fields is tested in supervisor_field.
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

#define NUMBER_OF_MAIN_FIELDS 7
#define NUMBER_OF_INTERNAL_FIELDS 6
#define NUMBER_OF_HIERARCHY_LEVELS 4

// Field/Proto references may become invalid if any nodes are regenerated. This function provides an easy way to re-retrieve
// them.
// At the time of writing, the protos being used are not templated, so they shouldn't be regenerated. However, this function
// is still useful for future-proofing and for segmenting the retrieval code from the main test.
static void retrieve_fields(const char *main_field_names[NUMBER_OF_MAIN_FIELDS][NUMBER_OF_HIERARCHY_LEVELS],
                            const char *internal_field_names[NUMBER_OF_INTERNAL_FIELDS][NUMBER_OF_HIERARCHY_LEVELS],
                            WbNodeRef hierarchy, WbFieldRef actual_main_fields[NUMBER_OF_MAIN_FIELDS],
                            WbFieldRef actual_internal_fields[NUMBER_OF_INTERNAL_FIELDS],
                            WbFieldRef main_fields[NUMBER_OF_MAIN_FIELDS][NUMBER_OF_HIERARCHY_LEVELS],
                            WbFieldRef internal_fields[NUMBER_OF_INTERNAL_FIELDS][NUMBER_OF_HIERARCHY_LEVELS]) {
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; ++i) {
    const char *name = main_field_names[i][0];

    if (!name) {
      // No top-level field exists
      actual_main_fields[i] = NULL;
      continue;
    }

    WbFieldRef field = wb_supervisor_node_get_field(hierarchy, name);
    ts_assert_pointer_not_null(field, "Field \"%s\" not found", name);
    actual_main_fields[i] = wb_supervisor_field_get_actual_field(field);
  }

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; ++i) {
    const char *name = internal_field_names[i][0];

    if (!name) {
      // No top-level field exists
      actual_internal_fields[i] = NULL;
      continue;
    }

    // All of the actual fields are part of the hierarchy node in the scene tree
    WbFieldRef field = wb_supervisor_node_get_field(hierarchy, name);
    ts_assert_pointer_not_null(field, "Field \"%s\" not found", name);
    actual_internal_fields[i] = wb_supervisor_field_get_actual_field(field);
  }

  WbProtoRef hierarchy_proto = wb_supervisor_node_get_proto(hierarchy);
  ts_assert_pointer_not_null(hierarchy_proto, "Hierarchy proto not found");

  // The first level of the internal hierarchy is the same as the main hierarchy
  // We handle it here, before we start messing with hierarchy_proto
  const char *proto_type = wb_supervisor_proto_get_type_name(hierarchy_proto);
  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; ++i) {
    const char *field_name = internal_field_names[i][0];
    if (field_name) {
      WbFieldRef field = wb_supervisor_proto_get_field(hierarchy_proto, field_name);
      ts_assert_pointer_not_null(field, "Field \"%s\" not found in proto of type \"%s\"", field_name, proto_type);
      internal_fields[i][0] = field;
    } else
      // The field does not exist at this level of the hierarchy
      internal_fields[i][0] = NULL;
  }

  // Fetch all the internal fields in the main hierarchy
  for (int i = 0; i < NUMBER_OF_HIERARCHY_LEVELS - 1; ++i) {
    ts_assert_pointer_not_null(
      hierarchy_proto, "Hierarchy level %d does not exist. Try running the supervisor_proto test for more information.", i);
    proto_type = wb_supervisor_proto_get_type_name(hierarchy_proto);
    for (int j = 0; j < NUMBER_OF_MAIN_FIELDS; ++j) {
      const char *name = main_field_names[j][i];
      if (name) {
        WbFieldRef field = wb_supervisor_proto_get_field(hierarchy_proto, name);
        ts_assert_pointer_not_null(field, "Field \"%s\" not found in proto of type \"%s\"", name, proto_type);
        main_fields[j][i] = field;
      } else
        // The field does not exist at this level of the hierarchy
        main_fields[j][i] = NULL;
    }
    hierarchy_proto = wb_supervisor_proto_get_parent(hierarchy_proto);
  }

  // The last expected hierarchy level refers to the base node
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; ++i) {
    const char *name = main_field_names[i][NUMBER_OF_HIERARCHY_LEVELS - 1];
    if (name) {
      WbFieldRef field = wb_supervisor_node_get_base_node_field(hierarchy, name);
      ts_assert_pointer_not_null(field, "Field \"%s\" not found in base hierarchy node", name);
      main_fields[i][NUMBER_OF_HIERARCHY_LEVELS - 1] = field;
    } else
      // The field does not exist at this level of the hierarchy
      main_fields[i][NUMBER_OF_HIERARCHY_LEVELS - 1] = NULL;
  }

  WbNodeRef internal_node = wb_supervisor_node_get_from_proto_def(hierarchy, "INTERNAL_NODE");
  ts_assert_pointer_not_null(internal_node, "Internal node not found");
  WbProtoRef internal_proto = wb_supervisor_node_get_proto(internal_node);
  ts_assert_pointer_not_null(internal_proto, "Internal node proto not found");

  // Fetch all the internal fields in the internal node hierarchy
  for (int i = 1; i < NUMBER_OF_HIERARCHY_LEVELS - 1; ++i) {
    ts_assert_pointer_not_null(
      internal_proto, "Hierarchy level %d does not exist. Try running the supervisor_proto test for more information.", i);
    proto_type = wb_supervisor_proto_get_type_name(internal_proto);
    for (int j = 0; j < NUMBER_OF_INTERNAL_FIELDS; ++j) {
      const char *name = internal_field_names[j][i];
      if (name) {
        WbFieldRef field = wb_supervisor_proto_get_field(internal_proto, name);
        ts_assert_pointer_not_null(field, "Field \"%s\" not found in proto of type \"%s\"", name, proto_type);
        internal_fields[j][i] = field;
      } else
        // The field does not exist at this level of the hierarchy
        internal_fields[j][i] = NULL;
    }
    internal_proto = wb_supervisor_proto_get_parent(internal_proto);
  }

  // The last expected hierarchy level refers to the base node
  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; ++i) {
    const char *name = internal_field_names[i][NUMBER_OF_HIERARCHY_LEVELS - 1];
    if (name) {
      WbFieldRef field = wb_supervisor_node_get_base_node_field(internal_node, name);
      ts_assert_pointer_not_null(field, "Field \"%s\" not found in base internal node", name);
      internal_fields[i][NUMBER_OF_HIERARCHY_LEVELS - 1] = field;
    } else
      // The field does not exist at this level of the hierarchy
      internal_fields[i][NUMBER_OF_HIERARCHY_LEVELS - 1] = NULL;
  }
}

// Checks that a field has the expected type, actual field, and value
static void check_field(WbFieldRef field, WbFieldRef actual_field, double expected_value, const char *name,
                        const char *check_name) {
  ts_assert_pointer_not_null(field, "(%s) Field \"%s\" not found", check_name, name);
  const int field_type = wb_supervisor_field_get_type(field);
  ts_assert_int_equal(field_type, WB_SF_FLOAT, "(%s) Field \"%s\" is not an SFFloat. Got type: %d", check_name, name,
                      field_type);
  ts_assert_boolean_equal(wb_supervisor_field_get_actual_field(field) == actual_field,
                          "(%s) Field \"%s\" actual field mismatch", check_name, name);
  const double actual_value = wb_supervisor_field_get_sf_float(field);
  ts_assert_double_equal(actual_value, expected_value, "(%s) Field \"%s\" value mismatch! Expected %f, but got %f", check_name,
                         name, expected_value, actual_value);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // This array keeps track of the names of the fields we want to test at each level of the main node hierarchy.
  // Each element in this array is an array that represents a single field, which may exist at many levels of the hierarchy.
  // Each of these arrays contains the name for that field at each level of the hierarchy, from the main hierarchy node
  // up to AND INCLUDING the base node type. All levels in the hierarchy are assumed to be present in the array.
  // If a field does not exist at a specific level in the hierarchy, it should be listed as NULL
  const char *main_field_names[NUMBER_OF_MAIN_FIELDS][NUMBER_OF_HIERARCHY_LEVELS] = {
    {"translationStep", "translationStep", "translationStep", "translationStep"},
    {"internalField1", "ucField2", NULL, NULL},
    {"ucField4", NULL, NULL, NULL},
    {NULL, "rotationStep", "rotationStep", "rotationStep"},
    {NULL, "ucField1", "ucField1", NULL},
    {NULL, "ucField3", NULL, NULL},
    {NULL, NULL, NULL, "radarCrossSection"},
  };
  // This array is the same as the above array, except, with the exception of the first entry in each sub-array,
  // all fields are assumed to be present in the in internal node rather than the main hierarchy node.
  const char *internal_field_names[NUMBER_OF_INTERNAL_FIELDS][NUMBER_OF_HIERARCHY_LEVELS] = {
    {"internalTranslationStep", "translationStep", "translationStep", "translationStep"},
    {"internalField2", "ucField2", NULL, NULL},
    {NULL, "rotationStep", "rotationStep", "rotationStep"},
    {NULL, "ucField1", "ucField1", NULL},
    {NULL, "ucField3", NULL, NULL},
    {NULL, NULL, NULL, "radarCrossSection"},
  };

  WbNodeRef hierarchy = wb_supervisor_node_get_from_def("HIERARCHY");
  ts_assert_pointer_not_null(hierarchy, "Hierarchy node not found");

  WbFieldRef actual_main_fields[NUMBER_OF_MAIN_FIELDS];
  WbFieldRef actual_internal_fields[NUMBER_OF_INTERNAL_FIELDS];
  WbFieldRef main_fields[NUMBER_OF_MAIN_FIELDS][NUMBER_OF_HIERARCHY_LEVELS];
  WbFieldRef internal_fields[NUMBER_OF_INTERNAL_FIELDS][NUMBER_OF_HIERARCHY_LEVELS];

  retrieve_fields(main_field_names, internal_field_names, hierarchy, actual_main_fields, actual_internal_fields, main_fields,
                  internal_fields);

  // Check that all the wb_supervisor_field_get_actual_field returns its input if the field is already in the scene tree
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; ++i)
    if (actual_main_fields[i])
      check_field(actual_main_fields[i], actual_main_fields[i], 0.01, main_field_names[i][0], "Initial value [main field]");

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; ++i)
    if (actual_internal_fields[i])
      check_field(actual_internal_fields[i], actual_internal_fields[i], 0.01, internal_field_names[i][0],
                  "Initial value [internal field]");

  // Check that wb_supervisor_field_get_actual_field returns the correct field (or NULL) in all other cases, and that internal
  // fields are read-only
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; ++i)
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; ++j)
      if (main_fields[i][j]) {
        check_field(main_fields[i][j], actual_main_fields[i], 0.01, main_field_names[i][j], "Initial value [main field]");
        wb_supervisor_field_set_sf_float(main_fields[i][j], 1.01);  // Should have no effect
      }

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; ++i)
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; ++j)
      if (internal_fields[i][j]) {
        check_field(internal_fields[i][j], actual_internal_fields[i], 0.01, internal_field_names[i][j],
                    "Initial value [internal field]");
        wb_supervisor_field_set_sf_float(internal_fields[i][j], 1.01);  // Should have no effect
      }

  wb_robot_step(TIME_STEP);

  // Nothing should have changed, but just in case, re-retrieve the fields
  retrieve_fields(main_field_names, internal_field_names, hierarchy, actual_main_fields, actual_internal_fields, main_fields,
                  internal_fields);

  // Check that no fields were modified
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; ++i) {
    if (actual_main_fields[i])
      check_field(actual_main_fields[i], actual_main_fields[i], 0.01, main_field_names[i][0],
                  "Update read-only field [main field]");
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; ++j)
      if (main_fields[i][j])
        check_field(main_fields[i][j], actual_main_fields[i], 0.01, main_field_names[i][j],
                    "Update read-only field [main field]");
  }

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; ++i) {
    if (actual_internal_fields[i])
      check_field(actual_internal_fields[i], actual_internal_fields[i], 0.01, internal_field_names[i][0],
                  "Update read-only field [internal field]");
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; ++j)
      if (internal_fields[i][j])
        check_field(internal_fields[i][j], actual_internal_fields[i], 0.01, internal_field_names[i][j],
                    "Update read-only field [internal field]");
  }

  // Update the fields
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; ++i)
    if (actual_main_fields[i])
      wb_supervisor_field_set_sf_float(actual_main_fields[i], i);

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; ++i)
    if (actual_internal_fields[i])
      wb_supervisor_field_set_sf_float(actual_internal_fields[i], i);

  wb_robot_step(TIME_STEP);

  // The nodes may have been regenerated. Update all the relevant references
  retrieve_fields(main_field_names, internal_field_names, hierarchy, actual_main_fields, actual_internal_fields, main_fields,
                  internal_fields);

  // Check that the fields were updated correctly
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; ++i) {
    if (actual_main_fields[i])
      check_field(actual_main_fields[i], actual_main_fields[i], i, main_field_names[i][0], "Update field [main field]");
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; ++j)
      if (main_fields[i][j])
        check_field(main_fields[i][j], actual_main_fields[i], actual_main_fields[i] ? i : 0.01, main_field_names[i][j],
                    "Update field [main field]");
  }

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; ++i) {
    if (actual_internal_fields[i])
      check_field(actual_internal_fields[i], actual_internal_fields[i], i, internal_field_names[i][0],
                  "Update field [internal field]");
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; ++j)
      if (internal_fields[i][j])
        check_field(internal_fields[i][j], actual_internal_fields[i], actual_internal_fields[i] ? i : 0.01,
                    internal_field_names[i][j], "Update field [internal field]");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
