#include <webots/emitter.h>
#include <webots/robot.h>

static int time_step = 0;
static WbDeviceTag e_radio = 0;
static WbDeviceTag e_infra_red = 0;
static int message[1];

static int step() {
  message[0] = wb_robot_get_time() * 1000;
  wb_emitter_send(e_radio, message, sizeof(message));
  wb_emitter_send(e_infra_red, message, sizeof(message));
  return wb_robot_step(time_step);
}

int main(int argc, char **argv) {
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();
  e_radio = wb_robot_get_device("emitter radio");
  e_infra_red = wb_robot_get_device("emitter infra-red");

  // stabilize the system
  step();
  step();
  step();

  // Receiver moves
  step();

  step();

  wb_robot_cleanup();

  return 0;
}
