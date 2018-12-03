/*
 * Description:  Test aperture field of Emitters and Receivers with
 *               infra-red transmission type.
 *               This file contains some tests of the reception of messages
 *               with different aperture values of both emitter and receiver.
 */

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);
  WbDeviceTag communication;
  int queueLength;

  int robotID = -1;
  const char *msg = "Hello!";

  if (strcmp(wb_robot_get_name(), "robot0") == 0) {
    communication = wb_robot_get_device("emitter0");  // aperture PI/2
    wb_emitter_set_channel(communication, 2);
    robotID = 0;
  } else if (strcmp(wb_robot_get_name(), "robot1") == 0) {
    communication = wb_robot_get_device("emitter1");  // aperture -1
    wb_emitter_set_channel(communication, 1);
    robotID = 1;
  } else if (strcmp(wb_robot_get_name(), "robot2") == 0) {
    communication = wb_robot_get_device("receiver2");  // aperture 0.5
    wb_receiver_set_channel(communication, 1);
    wb_receiver_enable(communication, TIME_STEP);
    robotID = 2;
  } else if (strcmp(wb_robot_get_name(), "robot3") == 0) {
    communication = wb_robot_get_device("receiver3");  // aperture 0.5
    wb_receiver_set_channel(communication, 2);
    wb_receiver_enable(communication, TIME_STEP);
    robotID = 3;
  }

  // CASE 1: message received
  wb_robot_step(TIME_STEP);
  switch (robotID) {
    case 0:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 1:
      wb_emitter_send(communication, msg, strlen(msg) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 1,
                          "Receiver \"receiver%d\" should receive %d not %d packets when emitter and receiver are inside the "
                          "respective emission/reception cones",
                          robotID, 1, queueLength);
      wb_receiver_next_packet(communication);
      break;

    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    default:
      break;
  }

  // CASE 2: emitter aperture too small
  wb_robot_step(TIME_STEP);
  switch (robotID) {
    case 0:
      wb_emitter_send(communication, msg, strlen(msg) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;
    case 1:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;
    case 2:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(
        queueLength, 0,
        "Receiver \"receiver%d\" should receive %d not %d packets when receiver is not located in emitter emission cone",
        robotID, 0, queueLength);
      break;

    default:
      break;
  }

  // CASE 3: receiver aperture too small
  wb_robot_step(TIME_STEP);
  switch (robotID) {
    case 0:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 1:
      wb_emitter_set_channel(communication, 2);
      wb_emitter_send(communication, msg, strlen(msg) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(
        queueLength, 0,
        "Receiver \"receiver%d\" should receive %d not %d packets when emitter is not located in receiver reception cone",
        robotID, 0, queueLength);
      break;

    default:
      break;
  }

  // CASE 3: emitter and receiver aperture too small
  wb_robot_step(TIME_STEP);
  switch (robotID) {
    case 0:
      wb_emitter_set_channel(communication, 1);
      wb_emitter_send(communication, msg, strlen(msg) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 1:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(
        queueLength, 0,
        "Receiver \"receiver%d\" should receive %d not %d packets when emitter is not located in receiver reception cone",
        robotID, 0, queueLength);
      break;

    case 3:
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
