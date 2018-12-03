#include <webots/receiver.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);
  char previous = 'B';
  int i;
  for (i = 0; i < 100; i++) {
    while (wb_receiver_get_queue_length(receiver) > 0) {
      const char *data = wb_receiver_get_data(receiver);
      ts_assert_int_not_equal(data[0], previous, "Received same message twice: '%c'.", previous);
      previous = data[0];
      wb_receiver_next_packet(receiver);
    }
    wb_robot_step(TIME_STEP);
  }
  ts_send_success();
  return EXIT_SUCCESS;
}
