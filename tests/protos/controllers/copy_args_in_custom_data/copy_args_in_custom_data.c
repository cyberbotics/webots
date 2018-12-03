#include <webots/robot.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();

  wb_robot_set_custom_data(argv[1]);

  while (wb_robot_step(TIME_STEP) != -1) {
  };

  wb_robot_cleanup();

  return 0;
}
