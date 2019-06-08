#include <webots/robot.h>
#include <webots/supervisor.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  if (argv[1][0] == 'S')
    ts_setup(argv[0]);
  else {
    srand(time(NULL) + (int)argv[1][0]);
    wb_robot_init();
  }
  WbNodeRef material;
  WbFieldRef field;
  material = wb_supervisor_node_get_from_def("MATERIAL");
  field = wb_supervisor_node_get_field(material, "emissiveColor");
  typedef struct {
    double red;
    double green;
    double blue;
  } Color;
  Color red = {1, 0, 0}, blue = {0, 0, 1}, color;
  int i, j;
  for (i = 0; i < 100; i++) {
    if (argv[1][0] != 'S') {  // load the CPU with randomly complex computation to perturbate system process scheduling
      double x = 0.0;
      int max = (rand() * rand()) % 10000000;
      for (j = 0; j < max; j++)
        x += sqrt((double)j) / 10000.0;
    }
    switch (argv[1][0]) {
      case 'R':
        wb_supervisor_field_set_sf_color(field, (double *)&red);
        break;
      case 'B':
        wb_supervisor_field_set_sf_color(field, (double *)&blue);
        break;
      case 'S':
        color = *(Color *)wb_supervisor_field_get_sf_color(field);
        if (i > 1)
          ts_assert_vec3_equal(color.red, color.green, color.blue, 0, 0, 1, "Wrong value read: %d %d %d - should be 1 0 0",
                               (int)color.red, (int)color.green, (int)color.blue);
        break;
    }
    wb_robot_step(TIME_STEP);
  }
  if (argv[1][0] == 'S')
    ts_send_success();
  return EXIT_SUCCESS;
}
