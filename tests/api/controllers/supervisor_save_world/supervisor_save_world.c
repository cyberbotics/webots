#include <webots/supervisor.h>

#include "../../../lib/file_utils.h"
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <stdio.h>
#include <string.h>

#define TARGET_WORLD_NAME "test.wbt"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = wb_robot_get_basic_time_step();

  remove_file(TARGET_WORLD_NAME);
  ts_assert_boolean_not_equal(file_exists(TARGET_WORLD_NAME), "Impossible to remove the target world file.");

  WbNodeRef sphere = wb_supervisor_node_get_from_def("SPHERE");
  ts_assert_pointer_not_null(sphere, "Cannot find the sphere object.");
  WbFieldRef sphere_radius_field = wb_supervisor_node_get_field(sphere, "radius");
  ts_assert_pointer_not_null(sphere_radius_field, "Cannot find the sphere radius field.");
  wb_supervisor_field_set_sf_float(sphere_radius_field, 0.2);

  WbNodeRef viewpoint = wb_supervisor_node_get_from_def("VIEWPOINT");
  ts_assert_pointer_not_null(viewpoint, "Cannot find the viewpoint object.");
  WbFieldRef viewpoint_orientation_field = wb_supervisor_node_get_field(viewpoint, "orientation");
  const double *orientation = wb_supervisor_field_get_sf_rotation(viewpoint_orientation_field);
  double expected_orientation[4];
  memcpy(&expected_orientation, orientation, 4 * sizeof(double));
  WbFieldRef viewpoint_position_field = wb_supervisor_node_get_field(viewpoint, "position");
  const double *position = wb_supervisor_field_get_sf_vec3f(viewpoint_position_field);
  double expected_position[3];
  memcpy(&expected_position, position, 3 * sizeof(double));

  wb_robot_step(time_step);

  ts_assert_double_equal(wb_supervisor_field_get_sf_float(sphere_radius_field), 0.2, "Cannot set the sphere radius field");

  bool success = wb_supervisor_world_save(TARGET_WORLD_NAME);

  ts_assert_boolean_equal(success, "Issue when saving the world.");
  ts_assert_boolean_equal(file_exists(TARGET_WORLD_NAME), "The target world file doesn't exist.");

  // check that the field values saved in target filematch the origin ones

  FILE *file = fopen(TARGET_WORLD_NAME, "r");
  ts_assert_pointer_not_null(file, "The target world file cannot be open.");

  bool radius_found = false;
  bool orientation_found = false;
  bool position_found = false;
  char line[150];
  while (!feof(file) && fgets(line, 150, file) != NULL) {
    char *first = strtok(line, " ");
    if (strcmp(first, "orientation") == 0) {
      double value[4];
      int i = 0;
      while (i < 4) {
        char *token = strtok(NULL, " ");
        ts_assert_pointer_not_null(token, "Invalid 'orientation' value in the target file.");
        value[i] = atof(token);
        i++;
      }
      ts_assert_doubles_equal(4, value, expected_orientation, "Wrong 'orientation' value in the target file.\n");
      orientation_found = true;
    } else if (strcmp(first, "position") == 0) {
      double value[3];
      int i = 0;
      while (i < 3) {
        char *token = strtok(NULL, " ");
        ts_assert_pointer_not_null(token, "Wrong 'position' value in the target file.");
        value[i] = atof(token);
        i++;
      }
      ts_assert_doubles_equal(3, value, expected_position, "Wrong 'position' value in the target file.\n");
      position_found = true;
    } else if (strcmp(first, "radius") == 0) {
      char *token = strtok(NULL, " ");
      ts_assert_pointer_not_null(token, "Invalid 'radius' value in the target file.");
      double value = atof(token);
      ts_assert_double_equal(value, 0.2, "Wrong 'radius' value in the target file: expected %f, found %f\n", 0.2, value);
      radius_found = true;
    }
  }
  fclose(file);

  ts_assert_boolean_equal(orientation_found, "The target file doesn't contain the viewpoint 'orientation' field definition.");
  ts_assert_boolean_equal(position_found, "The target file doesn't contain the viewpoint 'position' field definition.");
  ts_assert_boolean_equal(radius_found, "The target file doesn't contain the sphere radius modification.");

  ts_send_success();
  return EXIT_SUCCESS;
}
