#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <float.h>

static const char *temp_world_file_path = "../../worlds/floating_point_precision_temp.wbt";

// singular floating point values to be tested.
#define N_VALUES 12
static const double values[N_VALUES] = {
  // perfect 0.0
  0.0,
  // The 4 first values of this list are there to check that no noise digits are present when storing the file.
  0.1, 0.2, 0.3,
  // 7 decimals (Webots < R2018a max precision)
  123456.0, 0.123456,
  // 15 decimals (= DBL_DIG) at minimum should be ensured (Webots >= R2018a max precision)
  123456789012345.0, 0.123456789012345,
  // +/- double epsilon
  DBL_EPSILON, -DBL_EPSILON,
  // min/max double/float: we don't use DBL_MAX as it forbidden by Webots because it causes ODE failures.
  DBL_MIN, FLT_MAX};

int main(int argc, char **argv) {
  wb_robot_init();
  sprintf(ts_test_name, "floating_point_precision");

  int time_step = (int)wb_robot_get_basic_time_step();

  // Be sure DBL_EPSILON is correct.
  // https://stackoverflow.com/questions/1566198/how-to-portably-get-dbl-epsilon-in-c-c
  ts_assert_boolean_equal((1.0 + DBL_EPSILON) != 1.0 && (1.0 + DBL_EPSILON / 2) == 1.0, "DBL_EPSILON is not correct.");

  int i, c;
  char name_buffer[32];
  char import_buffer[1024];
  const char *custom_data = wb_robot_get_custom_data();

  WbNodeRef root = wb_supervisor_node_get_root();
  WbFieldRef root_children = wb_supervisor_node_get_field(root, "children");

  if (strcmp(custom_data, "init") == 0) {
    // case: the initial world is loaded.

    // 1. Create a Transform for each floating point values, and store the value into the Transform.translation field.
    for (i = 0; i < N_VALUES; ++i) {
      printf("%g\n", values[i]);
      sprintf(name_buffer, "TEST_%d", i);
      sprintf(import_buffer, "DEF %s Pose {}", name_buffer);
      wb_supervisor_field_import_mf_node_from_string(root_children, -1, import_buffer);
      WbNodeRef test_node = wb_supervisor_node_get_from_def(name_buffer);
      WbFieldRef test_translation = wb_supervisor_node_get_field(test_node, "translation");
      const double test_translation_values[3] = {values[i], values[i], values[i]};
      wb_supervisor_field_set_sf_vec3f(test_translation, test_translation_values);
    }

    // 2. Save and reload the world.
    wb_robot_set_custom_data("saved");  // to differentiate the initial and the saved world.
    wb_robot_step(time_step);
    wb_supervisor_world_save(temp_world_file_path);
    wb_supervisor_world_load(temp_world_file_path);

  } else if (strcmp(custom_data, "saved") == 0) {
    // case: the saved world is reloaded.

    ts_setup_done = true;
    ts_notify_controller_status(true);

    // 1. Check that the floating point values are the same as the loaded ones.
    for (i = 0; i < N_VALUES; ++i) {
      sprintf(name_buffer, "TEST_%d", i);
      WbNodeRef test_node = wb_supervisor_node_get_from_def(name_buffer);
      WbFieldRef test_translation = wb_supervisor_node_get_field(test_node, "translation");
      const double *test_translation_values = wb_supervisor_field_get_sf_vec3f(test_translation);
      for (c = 0; c < 3; ++c)
        ts_assert_double_equal(test_translation_values[c], values[i],
                               "Issue when getting value[%d][%d] (get = %g, expected = %g)", i, c, test_translation_values[c],
                               values[i]);
    }

    // 2. Check that noise digits are not present in the first 4 tests (by parsing the generated .wbt file).
    //    i.e. Read the generated .wbt file line by line, and check that the first 3 translations contains exactly 0.1, 0.2 and
    //    0.3
    FILE *fp = fopen(temp_world_file_path, "r");
    char line[1024];
    ts_assert_pointer_not_null(fp, "Cannot read %s.", temp_world_file_path);
    int translation_counter = 0;
    char b1[32], b2[32], b3[32], expected_value[32];
    while (fgets(line, 1024, fp)) {
      int read = sscanf(line, "  translation %s %s %s", b1, b2, b3);
      if (read == 3) {  // case: the current line is a translation line parsed successfully.
        translation_counter++;
        if (translation_counter > 3)
          break;  // the first 3 translations have been read successfully.

        sprintf(expected_value, "0.%d", translation_counter);  // Forge the expected value: 0.1, 0.2, etc.

        ts_assert_string_equal(b1, expected_value, "Unexpected TEST_%d.translation[0] (expected: %s, get: %s)",
                               translation_counter, expected_value, b1);
        ts_assert_string_equal(b2, expected_value, "Unexpected TEST_%d.translation[1] (expected: %s, get: %s)",
                               translation_counter, expected_value, b2);
        ts_assert_string_equal(b3, expected_value, "Unexpected TEST_%d.translation[2] (expected: %s, get: %s)",
                               translation_counter, expected_value, b3);
      }
    }
    fclose(fp);
    ts_assert_int_equal(translation_counter - 1, 3, "Cannot read 3 TEST_?.translation in %s (read: %d)", temp_world_file_path,
                        translation_counter);

    ts_send_success();
    return EXIT_SUCCESS;
  }

  wb_robot_cleanup();

  return 0;
}
