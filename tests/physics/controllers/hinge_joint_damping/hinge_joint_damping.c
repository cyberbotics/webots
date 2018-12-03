#include <webots/receiver.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);

  int n_success = 0;

  while (wb_robot_step(time_step) != -1) {
    if (wb_robot_get_time() > 2.0)
      ts_assert_boolean_equal(0, "Time-out reached before collisions are detected");

    int queue_length = wb_receiver_get_queue_length(receiver);
    if (queue_length > 0) {
      const char *buffer = wb_receiver_get_data(receiver);
      // printf("Received: %s\n", buffer);

      char name[64];
      double time;
      int ret = sscanf(buffer, "%s : collision at %lf", name, &time);
      ts_assert_int_equal(ret, 2, "fscanf failed");

      // printf("%d %s %lf\n", ret, name, time);

      if ((strcmp(name, "r1") == 0 && time == 0.48) || (strcmp(name, "r2") == 0 && time == 0.672) ||
          (strcmp(name, "r3") == 0 && time == 1.024) || (strcmp(name, "r4") == 0 && time == 1.024))
        n_success++;
      else
        ts_assert_boolean_equal(0, "Received unexpected data (name = \"%s\", time = %lf)", name, time);

      if (n_success == 4)
        break;

      wb_receiver_next_packet(receiver);
    }
  };

  ts_send_success();
  return EXIT_SUCCESS;
}
