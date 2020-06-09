#include <webots/robot.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define MAX_RESULT_LENGTH (10 * 1024)
#define TIME_STEP 16

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // Write URDF to file
  FILE *f;
  f = fopen("robot.urdf", "w");
  const char *urdf = wb_robot_get_urdf("");
  fprintf(f, "%s", urdf);
  fclose(f);

  // Save output of `check_urdf` to file
  int urdf_check_status = system("check_urdf robot.urdf > result.txt");
  ts_assert_boolean_not_equal(urdf_check_status, "`check_urdf` failed to be executed");

  // Verify output from `check_urdf`
  char result_string[MAX_RESULT_LENGTH];
  bool success_word_found = false;
  f = fopen("result.txt", "r");
  while (fgets(result_string, MAX_RESULT_LENGTH, f))
    if (strstr(result_string, "Successfully Parsed XML"))
      success_word_found = true;
  ts_assert_boolean_equal(success_word_found, "URDF verification failed");

  ts_send_success();
  return EXIT_SUCCESS;
}
