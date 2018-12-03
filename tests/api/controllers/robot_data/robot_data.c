#include <webots/robot.h>

#include <stdio.h>
#include <string.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();
  wb_robot_step(TIME_STEP);

  // wait until something is written in the data field by the supervisor
  while (strlen(wb_robot_get_custom_data()) == 0)
    wb_robot_step(TIME_STEP);

  // get formula, resolve it and write answer in the data field
  int a, b;
  char c;
  sscanf(wb_robot_get_custom_data(), "%d %c %d", &a, &c, &b);
  char result[8];
  sprintf(result, "%d", a + b);
  wb_robot_set_custom_data(result);

  wb_robot_step(TIME_STEP);

  wb_robot_cleanup();

  return 0;
}
