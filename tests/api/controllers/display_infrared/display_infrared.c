#include <webots/display.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

#define BLACK 0x00000000
#define WHITE 0x00FFFFFF

#define XSIZE 20
#define YSIZE 20
#define DIMENSION (XSIZE * YSIZE)
#define MAX_NAME 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag display = wb_robot_get_device("display");
  ts_assert_boolean_equal(display != 0, "Unable to find the display device");

  int i, j;
  char name[MAX_NAME];
  WbDeviceTag ds[DIMENSION];
  for (i = 0; i < XSIZE; i++) {
    for (j = 0; j < YSIZE; j++) {
      int index = j * XSIZE + i;
      sprintf(name, "ds%d-%d", i, j);
      ds[index] = wb_robot_get_device(name);
      ts_assert_boolean_equal(ds[index] != 0, "Unable to find the distance sensor [%d, %d] device", i, j);
      wb_distance_sensor_enable(ds[index], TIME_STEP);
    }
  }

  int display_width = wb_display_get_width(display);
  ts_assert_int_equal(display_width, XSIZE, "Display has a wrong width");
  int display_height = wb_display_get_height(display);
  ts_assert_int_equal(display_width, XSIZE, "Display has a wrong height");

  wb_display_set_color(display, BLACK);
  wb_display_fill_rectangle(display, 0, 0, display_width, display_height);
  wb_display_set_color(display, WHITE);
  wb_display_fill_oval(display, 5, 10, 4, 8);
  wb_display_fill_oval(display, 15, 10, 4, 8);

  wb_robot_step(TIME_STEP);

  const char *result_filename = "data.txt";
  FILE *result_file = fopen(result_filename, "w");
  ts_assert_pointer_not_null(result_file, "Unable to open result data file");
  FILE *expected_file = fopen("expected_data.txt", "r");
  ts_assert_pointer_not_null(expected_file, "Unable to open expected data file");

  for (j = 0; j < YSIZE; j++) {
    for (i = 0; i < XSIZE; i++) {
      double expected;
      int ret = fscanf(expected_file, "%lf", &expected);
      ts_assert_int_equal(ret, 1, "fscanf failed");
      int index = j * XSIZE + i;
      double value = 0.5 * wb_distance_sensor_get_value(ds[index]);
      fprintf(result_file, "%lf ", value);

      ts_assert_double_in_delta(value, expected, 0.01, "Measurement [%d, %d] failed. Expected=%lf Received=%lf", i, j, expected,
                                value);
    }
    fprintf(result_file, "\n");
  }

  fclose(expected_file);
  fclose(result_file);

  // debug
  /*
  char command[256];
  sprintf(command, "python ~/develop/webotsd/src/util/show_floating_point_data.py -f %s -w %d -h %d", result_filename, XSIZE,
          YSIZE);
  system(command);
  */

  ts_send_success();
  return EXIT_SUCCESS;
}
