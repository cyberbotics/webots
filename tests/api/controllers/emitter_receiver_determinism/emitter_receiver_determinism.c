#include <webots/emitter.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  char message[2];
  snprintf(message, sizeof(message), "%s", argv[1]);
  srand(time(NULL) + (int)argv[1][0]);
  int i, j;
  for (i = 0; i < 100; i++) {
    wb_emitter_send(emitter, message, strlen(message) + 1);
    double x = 0.0;
    int max = (rand() * rand()) % 10000000;
    for (j = 0; j < max; j++)  // load the CPU with randomly complex computation to perturbate system process scheduling
      x += sqrt((double)j) / 10000.0;
    wb_robot_step(TIME_STEP);
  }
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
