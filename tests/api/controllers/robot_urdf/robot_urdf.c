#include <webots/device.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <sys/wait.h>
#endif

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define MAX_LINE_LENGTH (10 * 1024)
#define MAX_OUTPUT_LENGTH (10 * 1024)
#define TIME_STEP 16

// Returns the number of the first line that differs (0 if the files are identical), and copies this line of each file.
int compare_files(FILE *file_1, FILE *file_2, char *line_1, char *line_2) {
  int count = 0;
  while (true) {
    count++;
    const bool end_1 = fgets(line_1, MAX_LINE_LENGTH, file_1) == NULL;
    const bool end_2 = fgets(line_2, MAX_LINE_LENGTH, file_2) == NULL;
    if (end_1)
      strcpy(line_1, "<end of file>");
    if (end_2)
      strcpy(line_2, "<end of file>");
    line_1[strcspn(line_1, "\r\n")] = '\0';
    line_2[strcspn(line_2, "\r\n")] = '\0';
    if (end_1 && end_2)
      return 0;
    if (end_1 || end_2 || strcmp(line_1, line_2))
      return count;
  }
}

int main(int argc, char **argv) {
  char reference_filename[64], generated_filename[64], check_filename[64], test[256];
  snprintf(generated_filename, sizeof(generated_filename), "%s.urdf", argv[1]);
  snprintf(reference_filename, sizeof(reference_filename), "%s_reference.urdf", argv[1]);
  // the robots of the test world run at the same time, so each one needs its own `check_urdf` output file
  snprintf(check_filename, sizeof(check_filename), "%s_check_urdf.txt", argv[1]);
  snprintf(test, sizeof(test), "%s_%s", argv[0], argv[1]);

  ts_setup(test);

  // URDF describes the robot at the zero position of its joints: moving the joints must not change the exported file
  if (argc > 2 && strcmp(argv[2], "move") == 0) {
    for (int i = 0; i < wb_robot_get_number_of_devices(); ++i) {
      const WbDeviceTag tag = wb_robot_get_device_by_index(i);
      if (wb_device_get_node_type(tag) == WB_NODE_ROTATIONAL_MOTOR)
        wb_motor_set_position(tag, 0.5);
      else if (wb_device_get_node_type(tag) == WB_NODE_LINEAR_MOTOR)
        wb_motor_set_position(tag, 0.05);
    }
    for (int i = 0; i < 50; ++i)
      wb_robot_step(TIME_STEP);
  }

  // Write URDF to file
  FILE *f_urdf = fopen(generated_filename, "w");
  const char *urdf = wb_robot_get_urdf("");
  fprintf(f_urdf, "%s", urdf);
  fclose(f_urdf);
#ifndef _WIN32  // check_urdf is not available on Windows
  char command[512];
  // Save the output of `check_urdf` (stdout and stderr) to file
  snprintf(command, sizeof(command), "check_urdf \"%s\" > \"%s\" 2>&1", generated_filename, check_filename);
  const int urdf_check_status = system(command);
  if (urdf_check_status == -1)
    ts_send_error_and_exit("Failed to run the command '%s'", command);
  FILE *f_res = fopen(check_filename, "r");
  if (!f_res)
    ts_send_error_and_exit("Cannot open '%s' file", check_filename);
  if (WIFEXITED(urdf_check_status) && WEXITSTATUS(urdf_check_status) == 127) {
    fclose(f_res);
    ts_send_error_and_exit("check_urdf command is missing. "
                           "Install with 'apt install liburdfdom-tools' on Ubuntu or 'brew install urdfdom' on MacOS.");
  }
  // Verify output from `check_urdf`
  char result_string[MAX_LINE_LENGTH];
  bool success_word_found = false;
  while (fgets(result_string, sizeof(result_string), f_res))
    if (strstr(result_string, "Successfully Parsed XML"))
      success_word_found = true;
  if (urdf_check_status != 0 || !success_word_found) {
    rewind(f_res);
    char *buffer = malloc(MAX_OUTPUT_LENGTH + 1);
    const size_t n = buffer ? fread(buffer, 1, MAX_OUTPUT_LENGTH, f_res) : 0;
    if (buffer)
      buffer[n] = '\0';
    fclose(f_res);
    ts_send_error_and_exit("URDF verification of '%s' failed: %s", generated_filename, buffer ? buffer : "");
  }
  fclose(f_res);
#endif

  // Compare URDF output to ground truth
  f_urdf = fopen(generated_filename, "r");
  FILE *f_urdf_ref = fopen(reference_filename, "r");
  if (!f_urdf || !f_urdf_ref)
    ts_send_error_and_exit("Cannot open '%s' or '%s'", generated_filename, reference_filename);
  char reference_line[MAX_LINE_LENGTH], exported_line[MAX_LINE_LENGTH];
  const int line = compare_files(f_urdf_ref, f_urdf, reference_line, exported_line);
  fclose(f_urdf);
  fclose(f_urdf_ref);
  if (line != 0)
    ts_send_error_and_exit("Reference file '%s' and exported file '%s' differ at line %d:\nreference: %s\nexported:  %s\n"
                           "Full exported URDF:\n%s",
                           reference_filename, generated_filename, line, reference_line, exported_line, urdf);

  ts_send_success();
  return EXIT_SUCCESS;
}
