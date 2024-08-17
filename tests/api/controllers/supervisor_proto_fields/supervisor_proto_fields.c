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

// Field/Proto references may become invalid if any nodes are regenerated. This function provides an easy way to re-retrieve them.
static void retrieve_fields(const char *main_field_names[NUMBER_OF_MAIN_FIELDS][NUMBER_OF_HIERARCHY_LEVELS],
    const char *internal_field_names[NUMBER_OF_INTERNAL_FIELDS][NUMBER_OF_HIERARCHY_LEVELS],
    WbNodeRef hierarchy, WbNodeRef internal_node, WbFieldRef actual_main_parameters[NUMBER_OF_MAIN_FIELDS],
    WbFieldRef actual_internal_parameters[NUMBER_OF_INTERNAL_FIELDS],
    WbFieldRef main_fields[NUMBER_OF_MAIN_FIELDS][NUMBER_OF_HIERARCHY_LEVELS],
    WbFieldRef internal_fields[NUMBER_OF_INTERNAL_FIELDS][NUMBER_OF_HIERARCHY_LEVELS]) {
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; i++) {
    const char *name = main_field_names[i][0];

    if(!name) {
      // No top-level parameter exists
      actual_main_parameters[i] = NULL;
      continue;
    }

    WbFieldRef field = wb_supervisor_node_get_field(hierarchy, name);
    ts_assert_pointer_not_null(field, "Parameter \"%s\" not found", name);
    actual_main_parameters[i] = wb_supervisor_field_get_actual_parameter(field);
  }

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; i++) {
    const char *name = internal_field_names[i][0];

    if(!name) {
      // No top-level parameter exists
      actual_internal_parameters[i] = NULL;
      continue;
    }

    // All of the actual parameters are part of the hierarchy node in the scene tree
    WbFieldRef field = wb_supervisor_node_get_field(hierarchy, name);
    ts_assert_pointer_not_null(field, "Parameter \"%s\" not found", name);
    actual_internal_parameters[i] = wb_supervisor_field_get_actual_parameter(field);
  }

  WbProtoRef hierarchy_proto = wb_supervisor_node_get_proto(hierarchy);
  ts_assert_pointer_not_null(hierarchy_proto, "Hierarchy proto not found");

  // The first level of the internal hierarchy is the same as the main hierarchy
  // We handle it here, before we start messing with hierarchy_proto
  const char *proto_type = wb_supervisor_proto_get_type_name(hierarchy_proto);
  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; i++) {
    const char *field_name = internal_field_names[i][0];
    if (field_name) {
      WbFieldRef field = wb_supervisor_proto_get_parameter(hierarchy_proto, field_name);
      ts_assert_pointer_not_null(field, "Field \"%s\" not found in proto of type \"%s\"", field_name, proto_type);
      internal_fields[i][0] = field;
    } else
      // The field does not exist at this level of the hierarchy
      internal_fields[i][0] = NULL;
  }

  // Fetch all the internal fields in the main hierarchy
  for (int i = 0; i < NUMBER_OF_HIERARCHY_LEVELS; i++) {
    ts_assert_pointer_not_null(hierarchy_proto, "Hierarchy level %d does not exist. Try running the supervisor_proto test for more information.", i);
    proto_type = wb_supervisor_proto_get_type_name(hierarchy_proto);
    for (int j = 0; j < NUMBER_OF_MAIN_FIELDS; j++) {
      const char *name = main_field_names[j][i];
      if (name) {
        WbFieldRef field = wb_supervisor_proto_get_parameter(hierarchy_proto, name);
        ts_assert_pointer_not_null(field, "Field \"%s\" not found in proto of type \"%s\"", name, proto_type);
        main_fields[j][i] = field;
      } else
        // The field does not exist at this level of the hierarchy
        main_fields[j][i] = NULL;
    }
    hierarchy_proto = wb_supervisor_proto_get_parent(hierarchy_proto);
  }

  WbProtoRef internal_proto = wb_supervisor_node_get_proto(internal_node);
  ts_assert_pointer_not_null(hierarchy_proto, "Internal node proto not found");

  // Fetch all the internal fields in the internal node hierarchy
  for (int i = 0; i < NUMBER_OF_HIERARCHY_LEVELS; i++) {
    ts_assert_pointer_not_null(internal_proto, "Hierarchy level %d does not exist. Try running the supervisor_proto test for more information.", i);
    proto_type = wb_supervisor_proto_get_type_name(internal_proto);
    for (int j = 0; j < NUMBER_OF_INTERNAL_FIELDS; j++) {
      const char *name = internal_field_names[j][i];
      if (name) {
        WbFieldRef field = wb_supervisor_proto_get_parameter(internal_proto, name);
        ts_assert_pointer_not_null(field, "Field \"%s\" not found in proto of type \"%s\"", name, proto_type);
        internal_fields[j][i] = field;
      } else
        // The field does not exist at this level of the hierarchy
        internal_fields[j][i] = NULL;
    }
    internal_proto = wb_supervisor_proto_get_parent(internal_proto);
  }
}

// Checks that a field has the expected type, actual parameter, and value
// If try_set is true, it will also attempt to set the field to a different value (which should have no effect)
static void check_field(WbFieldRef field, WbFieldRef actual_field, double expected_value, bool try_set, const char *name, const char *check_name) {
  ts_assert_pointer_not_null(field, "(%s) Field \"%s\" not found", check_name, name);
  const int field_type = wb_supervisor_field_get_type(field);
  ts_assert_int_equal(field_type, WB_SF_FLOAT, "(%s) Field \"%s\" is not an SFFloat. Got type: %d", check_name, name, field_type);
  ts_assert_boolean_equal(wb_supervisor_field_get_actual_parameter(field) == actual_field, "(%s) Field \"%s\" actual parameter mismatch", check_name, name);
  const double actual_value = wb_supervisor_field_get_sf_float(field);
  ts_assert_double_equal(actual_value, expected_value, "(%s) Field \"%s\" value mismatch! Expected %f, but got %f", check_name, name, expected_value, actual_value);

  if (try_set) {
    // The field should be read-only, so setting it should have no effect
    wb_supervisor_field_set_sf_float(field, expected_value + 1.0);
  }
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
  WbNodeRef internal_node = wb_supervisor_node_get_from_def("INTERNAL_HIERARCHY");

  WbFieldRef actual_main_parameters[NUMBER_OF_MAIN_FIELDS];
  WbFieldRef actual_internal_parameters[NUMBER_OF_INTERNAL_FIELDS];
  WbFieldRef main_fields[NUMBER_OF_MAIN_FIELDS][NUMBER_OF_HIERARCHY_LEVELS];
  WbFieldRef internal_fields[NUMBER_OF_INTERNAL_FIELDS][NUMBER_OF_HIERARCHY_LEVELS];

  retrieve_fields(main_field_names, internal_field_names, hierarchy, internal_node,
                  actual_main_parameters, actual_internal_parameters, main_fields, internal_fields);

  // Check that all the wb_supervisor_field_get_actual_parameter returns its input if the field is already in the scene tree
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; i++)
    if (actual_main_parameters[i])
      check_field(actual_main_parameters[i], actual_main_parameters[i], 0.01, false, main_field_names[i][0], "Initial value [main parameter]");

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; i++)
    if (actual_internal_parameters[i])
      check_field(actual_internal_parameters[i], actual_internal_parameters[i], 0.01, false, internal_field_names[i][0], "Initial value [internal parameter]");

  // Check that wb_supervisor_field_get_actual_parameter returns the correct field (or NULL) in all other cases, and that internal fields are read-only
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; i++)
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; j++)
      if (main_fields[i][j])
        check_field(main_fields[i][j], actual_main_parameters[i], 0.01, true, main_field_names[i][j], "Initial value [main field]");

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; i++)
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; j++)
      if (internal_fields[i][j])
        check_field(internal_fields[i][j], actual_internal_parameters[i], 0.01, true, internal_field_names[i][j], "Initial value [internal field]");

  wb_robot_step(TIME_STEP);

  // Nothing should have changed, but just in case, re-retrieve the fields
  internal_node = wb_supervisor_node_get_from_def("INTERNAL_HIERARCHY");
  retrieve_fields(main_field_names, internal_field_names, hierarchy, internal_node,
                  actual_main_parameters, actual_internal_parameters, main_fields, internal_fields);

  // Check that no fields were modified
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; i++) {
    check_field(actual_main_parameters[i], actual_main_parameters[i], i, false, main_field_names[i][0], "Update read-only field [main parameter]");
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; j++)
      if (main_field_names[i][j])
        check_field(main_fields[i][j], actual_main_parameters[i], i, false, main_field_names[i][j], "Update read-only field [main field]");
  }

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; i++) {
    check_field(actual_internal_parameters[i], actual_internal_parameters[i], i, false, internal_field_names[i][0], "Update read-only field [internal parameter]");
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; j++)
      if (main_field_names[i][j])
        check_field(internal_fields[i][j], actual_internal_parameters[i], i, false, internal_field_names[i][j], "Update read-only field [internal field]");
  }

  // Update the fields
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; i++)
    if (actual_main_parameters[i])
      wb_supervisor_field_set_sf_float(actual_main_parameters[i], i);

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; i++)
    if (actual_internal_parameters[i])
      wb_supervisor_field_set_sf_float(actual_internal_parameters[i], i);

  wb_robot_step(TIME_STEP);

  // The nodes may have been regenerated. Update all the relevant references
  internal_node = wb_supervisor_node_get_from_def("INTERNAL_HIERARCHY");
  retrieve_fields(main_field_names, internal_field_names, hierarchy, internal_node,
                  actual_main_parameters, actual_internal_parameters, main_fields, internal_fields);

  // Check that the fields were updated correctly
  for (int i = 0; i < NUMBER_OF_MAIN_FIELDS; i++) {
    if (actual_main_parameters[i])
      check_field(actual_main_parameters[i], actual_main_parameters[i], i, false, main_field_names[i][0], "Update field [main parameter]");
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; j++)
      if (main_fields[i][j])
        check_field(main_fields[i][j], actual_main_parameters[i], i, false, main_field_names[i][j], "Update field [main field]");
  }

  for (int i = 0; i < NUMBER_OF_INTERNAL_FIELDS; i++) {
    if (actual_internal_parameters[i])
      check_field(actual_internal_parameters[i], actual_internal_parameters[i], i, false, internal_field_names[i][0], "Update field [internal parameter]");
    for (int j = 0; j < NUMBER_OF_HIERARCHY_LEVELS; j++)
      if (internal_fields[i][j])
        check_field(internal_fields[i][j], actual_internal_parameters[i], i, false, internal_field_names[i][j], "Update field [internal field]");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
