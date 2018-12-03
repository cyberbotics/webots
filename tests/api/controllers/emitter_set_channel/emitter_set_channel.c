/*
 * Description:   Test set_channel function of Emitter device.
 *                It should be possible to change the channel just before
 *                sending a message without needing to wait for the next step.
 */

#include <string.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

const char *MESSAGE1 = "Hello1";
const char *MESSAGE2 = "Hello2";

// controller for RECEIVER1 Robot
void receiver1_controller() {
  int length = 0;

  WbDeviceTag receiver = wb_robot_get_device("receiver1");
  wb_receiver_enable(receiver, TIME_STEP);

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  // STEP 2
  // one message is received
  length = wb_receiver_get_queue_length(receiver);
  ts_assert_double_in_delta(length, 1, 0.5, "Receiver1 received %d message(s) instead of 1 at step 2.", length);
  const char *msg1 = (const char *)wb_receiver_get_data(receiver);
  ts_assert_string_equal(msg1, MESSAGE1, "Receiver1 received the wrong message at step 2: '%s' instead of '%s'.", msg1,
                         MESSAGE1);
  wb_receiver_next_packet(receiver);

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  // STEP 4
  // one message is received
  length = wb_receiver_get_queue_length(receiver);
  ts_assert_double_in_delta(length, 1, 0.5, "Receiver1 received %d message(s) instead of 1 at step 4.", length);
  const char *msg2 = (const char *)wb_receiver_get_data(receiver);
  ts_assert_string_equal(msg2, MESSAGE1, "Receiver1 received the wrong message at step 4: '%s' instead of '%s'.", msg2,
                         MESSAGE1);
  wb_receiver_next_packet(receiver);
}

// controller for RECEIVER2 Robot
void receiver2_controller() {
  int length = 0;

  WbDeviceTag receiver = wb_robot_get_device("receiver2");
  wb_receiver_enable(receiver, TIME_STEP);

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  // STEP 2
  // no message is received
  length = wb_receiver_get_queue_length(receiver);
  ts_assert_double_in_delta(length, 0, 0.5, "Receiver2 received %d message(s) instead of 0 at step 2.", length);

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  // STEP 4
  // one message is received
  length = wb_receiver_get_queue_length(receiver);
  ts_assert_double_in_delta(length, 1, 0.5, "Receiver2 received %d message(s) instead of 1 at step 4.", length);
  const char *msg = (const char *)wb_receiver_get_data(receiver);
  ts_assert_string_equal(msg, MESSAGE2, "Receiver2 received the wrong message at step 4: '%s' instead of '%s'.", msg, MESSAGE2);
  wb_receiver_next_packet(receiver);
}

// controller for EMITTER Robot
void emitter_controller() {
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  printf("Device tag %d\n", emitter);
  wb_robot_step(TIME_STEP);

  // STEP 1
  // send message on channel 1
  wb_emitter_set_channel(emitter, 1);
  wb_emitter_send(emitter, MESSAGE1, strlen(MESSAGE1) + 1);

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  // STEP 3
  // send message on channel 2
  wb_emitter_set_channel(emitter, 2);
  wb_emitter_send(emitter, MESSAGE2, strlen(MESSAGE2) + 1);

  // send message on channel 1
  wb_emitter_set_channel(emitter, 1);
  wb_emitter_send(emitter, MESSAGE1, strlen(MESSAGE1) + 1);

  wb_robot_step(TIME_STEP);
}

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  const char *name = wb_robot_get_name();
  if (strcmp(name, "robot_emitter") == 0)
    emitter_controller();
  else if (strcmp(name, "robot_receiver1") == 0) {
    receiver1_controller();
    wb_robot_step(TIME_STEP);  // avoid collision of simultaneous success messages on stdout
  } else if (strcmp(name, "robot_receiver2") == 0) {
    receiver2_controller();
    wb_robot_step(2 * TIME_STEP);  // avoid collision of simultaneous success messages on stdout
  }
  ts_send_success();
  return EXIT_SUCCESS;
}
