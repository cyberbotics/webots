/*
 * Description:  Test Emitters and Receivers with mixed transmission type.
 *               This file contains some tests of the emission/reception of data:
 *                 - disabled / ensabled receivers
 *                 - communicate only with device with the same transmission type
 *                 - communicate on different channels
 */

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);
  WbDeviceTag emitter, receiver;
  int queueLength;
  const char *buffer;

  int robotID = -1;

  // messages
  const char *msgString0 = "Hello";
  const char *msgString1 = "Radio";
  const char *msgString2 = "Serial";
  const char *msgString3 = "Infra-Red";

  if (strcmp(wb_robot_get_name(), "robot0") == 0) {
    emitter = wb_robot_get_device("emitter0");    // serial,    channel 2
    receiver = wb_robot_get_device("receiver0");  // radio,     channel -1
    robotID = 0;
  } else if (strcmp(wb_robot_get_name(), "robot1") == 0) {
    emitter = wb_robot_get_device("emitter1");  // infra-red, channel -1
    robotID = 1;
  } else if (strcmp(wb_robot_get_name(), "robot2") == 0) {
    emitter = wb_robot_get_device("emitter2");  // radio,     channel -1
    robotID = 2;
  } else if (strcmp(wb_robot_get_name(), "robot3") == 0) {
    receiver = wb_robot_get_device("receiver3");  // serial,    channel 2
    robotID = 3;
  } else if (strcmp(wb_robot_get_name(), "robot4") == 0) {
    receiver = wb_robot_get_device("receiver4");  // infra-red, channel 2
    robotID = 4;
  }

  // TEST 1: receivers disabled
  //         emitter0 -> msgString0 (serial,    channel 2)
  //         emitter1 -> msgString0 (infra-red, channel -1)
  //         emitter2 -> msgString0 (radio,     channel -1)
  wb_robot_step(TIME_STEP);
  switch (robotID) {
    case 0:
      wb_emitter_send(emitter, msgString0, strlen(msgString0) + 1);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 0, "Radio receiver should receive %d not %d packets when disabled", 0, queueLength);
      break;

    case 1:
      wb_emitter_send(emitter, msgString0, strlen(msgString0) + 1);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_emitter_send(emitter, msgString0, strlen(msgString0) + 1);
      wb_robot_step(TIME_STEP);
      break;

    case 3:
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 0, "Serial receiver should receive %d not %d packets when disabled", 0, queueLength);
      break;

    case 4:
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 0, "Infra-red receiver should receive %d not %d packets when disabled", 0, queueLength);
      break;

    default:
      break;
  }

  // TEST 2: receivers enabled
  //         emitter0 -> msgString2 (serial,    channel 2)
  //         emitter1 -> msgString3 (infra-red, channel -1)
  //         emitter2 -> msgString1 (radio,     channel -1)
  wb_robot_step(TIME_STEP);
  switch (robotID) {
    case 0:
      wb_receiver_enable(receiver, TIME_STEP);
      wb_emitter_send(emitter, msgString2, strlen(msgString2) + 1);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 1, "Radio receiver should receive %d not %d packets when enabled", 1, queueLength);

      buffer = wb_receiver_get_data(receiver);
      ts_assert_string_equal(buffer, msgString1, "Radio receiver should receive a packet containing \"%s\" not \"%s\"",
                             msgString1, buffer);
      wb_receiver_next_packet(receiver);
      wb_robot_step(TIME_STEP);
      break;

    case 1:
      wb_emitter_send(emitter, msgString3, strlen(msgString3) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_emitter_send(emitter, msgString1, strlen(msgString1) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 3:
      wb_receiver_enable(receiver, 2 * TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 0, "Serial receiver should receive %d not %d packets one step after being enabled", 0,
                          queueLength);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 1, "Serial receiver should receive %d not %d packets two steps after being enabled", 1,
                          queueLength);

      buffer = wb_receiver_get_data(receiver);
      ts_assert_string_equal(buffer, msgString2, "Serial receiver should receive a packet containing \"%s\" not \"%s\"",
                             msgString2, buffer);
      wb_receiver_next_packet(receiver);
      break;

    case 4:
      wb_receiver_enable(receiver, TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 1, "Infra-red receiver should receive %d not %d packets when enabled", 1, queueLength);

      buffer = wb_receiver_get_data(receiver);
      ts_assert_string_equal(buffer, msgString3, "Infra-red receiver should receive a packet containing \"%s\" not \"%s\"",
                             msgString3, buffer);
      wb_receiver_next_packet(receiver);
      wb_robot_step(TIME_STEP);
      break;

    default:
      break;
  }

  // TEST 3: Radio receiver channel = 0; serial receiver = 10; infra-red receiver disabled
  //         emitter0 -> msgString2 (serial,    channel 2)
  //         emitter1 -> msgString3 (infra-red, channel -1)
  //         emitter2 -> msgString1 (radio,     channel -1)
  wb_robot_step(TIME_STEP);
  switch (robotID) {
    case 0:
      wb_receiver_set_channel(receiver, 0);
      wb_emitter_send(emitter, msgString2, strlen(msgString2) + 1);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 1, "Radio receiver should receive %d not %d packets when channel set to 0", 1,
                          queueLength);

      buffer = wb_receiver_get_data(receiver);
      ts_assert_string_equal(buffer, msgString1,
                             "Radio receiver should receive a packet containing \"%s\" not \"%s\" when channel set to 0",
                             msgString1, buffer);
      wb_receiver_next_packet(receiver);
      wb_robot_step(TIME_STEP);
      break;

    case 1:
      wb_emitter_send(emitter, msgString3, strlen(msgString3) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_emitter_send(emitter, msgString1, strlen(msgString1) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 3:
      wb_receiver_set_channel(receiver, 10);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 0, "Serial receiver should receive %d not %d packets when channel set to 10", 0,
                          queueLength);
      wb_receiver_set_channel(receiver, 2);
      break;

    case 4:
      wb_receiver_disable(receiver);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 0, "Infra-red receiver should receive %d not %d packets when re-disabled", 0,
                          queueLength);
      wb_robot_step(TIME_STEP);
      break;

    default:
      break;
  }

  // TEST 4: Radio receiver channel = 0; serial receiver = 2; infra-red receiver disabled
  //         emitter2 -> msgString2 (serial, channel 2)
  //         emitter2 -> msgString1 (radio, channel 0)
  wb_robot_step(TIME_STEP);
  switch (robotID) {
    case 0:
      wb_emitter_send(emitter, msgString2, strlen(msgString2) + 1);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 1, "Radio receiver should receive %d not %d packets when channel set to 0", 1,
                          queueLength);

      buffer = wb_receiver_get_data(receiver);
      ts_assert_string_equal(buffer, msgString1,
                             "Radio receiver should receive a packet containing \"%s\" not \"%s\" when channel set to 0",
                             msgString1, buffer);
      wb_receiver_next_packet(receiver);
      wb_robot_step(TIME_STEP);
      break;

    case 1:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_emitter_set_channel(emitter, 0);
      wb_emitter_send(emitter, msgString1, strlen(msgString1) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 3:
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 0,
                          "Serial receiver should receive %d not %d packets during same step when the message is sent", 0,
                          queueLength);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(queueLength, 0, "Serial receiver should receive %d not %d packets one step after the message is sent",
                          0, queueLength);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(receiver);
      ts_assert_int_equal(
        queueLength, 1, "Serial receiver should receive %d not %d packets two steps after the message is sent", 1, queueLength);
      wb_receiver_next_packet(receiver);
      break;

    case 4:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    default:
      break;
  }
  wb_robot_step(robotID * TIME_STEP);  // avoid collision of simultaneous success messages on stdout
  ts_send_success();
  return EXIT_SUCCESS;
}
