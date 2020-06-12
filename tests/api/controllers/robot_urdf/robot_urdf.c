#include <webots/robot.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define MAX_LINE_LENGTH (10 * 1024)
#define TIME_STEP 16

bool are_files_equals(FILE *file_1, FILE *file_2) {
  char line_1[MAX_LINE_LENGTH];
  char line_2[MAX_LINE_LENGTH];
  char *line_status_1;
  char *line_status_2;

  do {
    line_status_1 = fgets(line_1, MAX_LINE_LENGTH, file_1);
    line_status_2 = fgets(line_2, MAX_LINE_LENGTH, file_2);
    if (strcmp(line_1, line_2))
      return false;
  } while (line_status_1 || line_status_2);

  return true;
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // Write URDF to file
  FILE *f_urdf = fopen("robot.urdf", "w");
  const char *urdf = wb_robot_get_urdf("");
  fprintf(f_urdf, "%s", urdf);
  fclose(f_urdf);

  // Save output of `check_urdf` to file
  const int urdf_check_status = system("check_urdf robot.urdf > result.txt");
  FILE *f_res = fopen("result.txt", "r");
  fseek(f_res, 0L, SEEK_END);
  const int file_size = ftell(f_res);
  if ((urdf_check_status >> 8) != 127 && file_size > 0) {
    // `check_urdf` command is available
    // Verify output from `check_urdf`
    rewind(f_res);
    char result_string[MAX_LINE_LENGTH];
    bool success_word_found = false;

    while (fgets(result_string, MAX_LINE_LENGTH, f_res))
      if (strstr(result_string, "Successfully Parsed XML"))
        success_word_found = true;
    ts_assert_boolean_equal(success_word_found, "URDF verification failed");
  }

  // Compare URDF output to ground truth
  f_urdf = fopen("robot.urdf", "r");
  FILE *f_urdf_ref = fopen("reference_robot.urdf", "r");
  ts_assert_boolean_equal(are_files_equals(f_urdf_ref, f_urdf), "Reference file and exported files are not the same");
  fclose(f_urdf);
  fclose(f_urdf_ref);

  ts_send_success();
  return EXIT_SUCCESS;
}
