/*
 * Description:  Test Emitters and Receivers with serial transmission type.
 *               This file contains some tests of the reception of messages:
 *                 - disabled / ensabled receivers
 *                 - different channels, emitter/receiver broadcast
 *                 - multiple emitters/receivers
 *                 - send and receive on the same robot
 *                 - reduced emitter/receiver buffer size
 *                 - reduced emitter range
 */

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);
  WbDeviceTag communication = 0, receiver0 = 0;
  const char *buffer;
  const double *dbuffer;
  const int *ibuffer;
  int queueLength, samplingPeriod, dataSize, bufferSize, channel;
  double signalStrength, range;

  int robotID = -1;

  // channels
  const int channel1 = 1;
  const int channel2 = 7;
  const int channel3 = 24;
  const int channelAll = -1;

  // messages
  const char *msgString1 = "Hello!";
  const char *msgString2 = "Do you received the message?";
  const char *msgString3 =
    "A very long message that should be larger than the buffer size and cause a buffer overflow if not handled properly.";
  double *msgDouble4 = (double *)malloc(4 * (sizeof(double)));
  msgDouble4[0] = 9.67;
  msgDouble4[1] = 8.75;
  msgDouble4[2] = 0.00;
  msgDouble4[3] = 1.61;
  int *msgInt5 = (int *)malloc(2 * (sizeof(int)));
  msgInt5[0] = 2;
  msgInt5[1] = 33;

  if (strcmp(wb_robot_get_name(), "robot0") == 0) {
    communication = wb_robot_get_device("emitter0");
    receiver0 = wb_robot_get_device("receiver0");
    wb_emitter_set_channel(communication, channel1);
    robotID = 0;
  } else if (strcmp(wb_robot_get_name(), "robot1") == 0) {
    communication = wb_robot_get_device("emitter1");
    wb_emitter_set_channel(communication, channel2);
    robotID = 1;
  } else if (strcmp(wb_robot_get_name(), "robot2") == 0) {
    communication = wb_robot_get_device("receiver2");
    wb_receiver_set_channel(communication, channel1);
    robotID = 2;
  } else if (strcmp(wb_robot_get_name(), "robot3") == 0) {
    communication = wb_robot_get_device("receiver3");
    wb_receiver_set_channel(communication, channel3);
    robotID = 3;
  }

  wb_robot_step(TIME_STEP);
  // CASE 1: Receivers not enabled
  // e0: channel1, e1: channel2, r0: channelALL (disabled), r2: channel1 (disabled), r3: channel3 (disabled)
  // communication:

  switch (robotID) {
    case 0:
    case 1:
      wb_emitter_send(communication, msgString1, strlen(msgString1) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;
    case 2:
    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 0,
                          "Case 1 - Receiver \"receiver%d\" should not receive %d packets not %d packets when disabled",
                          robotID, 0, queueLength);

      wb_receiver_enable(communication, TIME_STEP);

      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(
        queueLength, 0,
        "Case 1 - Receiver \"receiver%d\" should not receive %d packets not %d packets before the step after it is enabled",
        robotID, 0, queueLength);
      break;
    default:
      break;
  }

  wb_robot_step(TIME_STEP);

  // CASE 2: Receivers enabled
  // e0: channel1, e1: channel2, r0: channelALL (disabled), r2: channel1, r3: channel3
  // communication: e0 -> r2, msgString1
  switch (robotID) {
    case 0:
      wb_emitter_send(communication, msgString1, strlen(msgString1) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;
    case 1:
      wb_emitter_send(communication, msgString2, strlen(msgString2) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;
    case 2:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 1, "Case 2 - Receiver \"receiver%d\" should receive 1 packet not %d packets",
                          robotID - 2, queueLength);

      // check packet content
      buffer = wb_receiver_get_data(communication);
      ts_assert_string_equal(buffer, msgString1,
                             "Case 2 - Receiver \"receiver%d\" should receive packet contaning \"%c\" not \"%c\"", msgString1,
                             buffer);

      wb_receiver_next_packet(communication);
      break;

    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 0, "Case 2 - Receiver \"receiver%d\" should receive 0 packet not %d packets",
                          robotID - 2, queueLength);

      wb_receiver_set_channel(communication, channel2);
      break;

    default:
      break;
  }

  wb_robot_step(TIME_STEP);
  // CASE 3: Multiple messages sent by an emitter and data size checks
  // e0: channel1, e1: channel2, r0: channelALL (disabled), r2: channel1, r3: channel2
  // communication: e0 -> r2, msgString1
  //                e0 -> r2, msgInt5
  //                e1 -> r3, msgString2
  switch (robotID) {
    case 0:
      wb_emitter_send(communication, msgString1, strlen(msgString1) + 1);
      wb_emitter_send(communication, msgInt5, 2 * sizeof(int));
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 1:
      wb_emitter_send(communication, msgString3, strlen(msgString3) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 2, "Case 3 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          2, queueLength);

      // check packets content
      buffer = wb_receiver_get_data(communication);
      dataSize = wb_receiver_get_data_size(communication);
      ts_assert_boolean_equal(dataSize == (strlen(msgString1) + 1),
                              "Case 3 - Receiver \"receiver%d\" should receive a first packet with size %d not %d", robotID,
                              (strlen(msgString1) + 1), dataSize);
      ts_assert_string_equal(buffer, msgString1,
                             "Case 3 - Receiver \"receiver%d\" should receive a first packet contaning \"%c\" not \"%c\"",
                             robotID, msgString1, buffer);

      wb_receiver_next_packet(communication);

      dataSize = wb_receiver_get_data_size(communication);
      ibuffer = (const int *)wb_receiver_get_data(communication);
      ts_assert_boolean_equal(dataSize == (2 * sizeof(int)),
                              "Case 3 - Receiver \"receiver%d\" should receive a second packet with size %d not %d", robotID,
                              2 * sizeof(int), dataSize);
      ts_assert_boolean_equal(ibuffer[0] == msgInt5[0] && ibuffer[1] == msgInt5[1],
                              "Case 3 - Receiver \"receiver%d\" should receive a second packet contaning [%d, %d] not [%d, %d]",
                              robotID, msgInt5[0], msgInt5[1], ibuffer[0], ibuffer[1]);

      wb_receiver_next_packet(communication);

      break;

    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 1, "Case 3 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          1, queueLength);

      // check packet content
      buffer = wb_receiver_get_data(communication);
      dataSize = wb_receiver_get_data_size(communication);
      ts_assert_boolean_equal(dataSize == (strlen(msgString3) + 1),
                              "Case 3 - Receiver \"receiver%d\" should receive packet with size %d not %d", robotID,
                              (strlen(msgString3) + 1), dataSize);
      ts_assert_string_equal(buffer, msgString3,
                             "Case 3 - Receiver \"receiver%d\" should receive packet contaning \"%c\" not \"%c\"", robotID,
                             msgString3, buffer);

      wb_receiver_next_packet(communication);

      wb_receiver_set_channel(communication, channel1);
      break;
    default:
      break;
  }

  wb_robot_step(TIME_STEP);
  // CASE 4: Multiple messages sent by an emitter to multiple receivers and get/set channel  check
  // e0: channel1, e1: channel2, r0: channelALL (disabled), r2: channel1, r3: channel1
  // communication: e0 -> r2, msgString1
  //                e0 -> r2, msgDouble5
  //                e0 -> r3, msgString1
  //                e0 -> r3, msgDouble5

  switch (robotID) {
    case 0:
      wb_emitter_send(communication, msgString1, strlen(msgString1) + 1);
      wb_emitter_send(communication, msgDouble4, 4 * sizeof(double));

      channel = wb_emitter_get_channel(communication);
      ts_assert_boolean_equal(channel == channel1, "Case 4 - Emitter \"emiter%d\" should be set to channel %d not %d", robotID,
                              channel1, channel);

      wb_emitter_set_channel(communication, channel3);
      channel = wb_emitter_get_channel(communication);
      ts_assert_boolean_equal(channel == channel3, "Case 4 - Emitter \"emitter%d\" should be set to channel %d not %d", robotID,
                              channel3, channel);

      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;
    case 1:
      wb_emitter_send(communication, msgString2, strlen(msgString2) + 1);
      wb_emitter_set_channel(communication, channel3);

      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 2, "Case 4 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          2, queueLength);

      // check packets content
      buffer = wb_receiver_get_data(communication);
      ts_assert_string_equal(buffer, msgString1,
                             "Case 4 - Receiver \"receiver%d\" should receive a first packet contaning \"%c\" not \"%c\"",
                             robotID, msgString1, buffer);

      wb_receiver_next_packet(communication);

      dbuffer = (double *)wb_receiver_get_data(communication);
      ts_assert_doubles_in_delta(
        4, dbuffer, msgDouble4, 0.001,
        "Case 4 - Receiver \"receiver%d\" should receive a second packet contaning [%f, %f, %f, %f] not [%f, %f, %f, %f]",
        robotID, msgDouble4[0], msgDouble4[1], msgDouble4[2], msgDouble4[3], dbuffer[0], dbuffer[1], dbuffer[2], dbuffer[3]);

      wb_receiver_next_packet(communication);

      if (robotID == 3) {
        channel = wb_receiver_get_channel(communication);
        ts_assert_boolean_equal(channel == channel1,
                                "Case 4 - Receiver \"receiver%d\" should be set to channel %d not %d before changing it",
                                robotID, channel1, channel);

        wb_receiver_set_channel(communication, channel3);
        channel = wb_receiver_get_channel(communication);
        ts_assert_boolean_equal(channel == channel3,
                                "Case 4 - Receiver \"receiver%d\" should be set to channel %d not %d after changing it",
                                robotID, channel3, channel);

      } else {
        channel = wb_receiver_get_channel(communication);
        ts_assert_boolean_equal(channel == channel1, "Case 4 - Receiver \"receiver%d\" should be set to channel %d not %d",
                                robotID, channel1, channel);
      }
      break;

    default:
      break;
  }

  wb_robot_step(TIME_STEP);
  // CASE 5: Multiple packets sent to a receiver by multiple emitters
  // e0: channel3, e1: channel3, r0: channelALL (disabled), r2: channel1, r3: channel3
  // communication: e0 -> r3, msgString1
  //                e1 -> r3, msgString2
  //                e1 -> r3, msgString3
  switch (robotID) {
    case 0:
      wb_emitter_send(communication, msgString1, strlen(msgString1) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 1:
      wb_robot_step(TIME_STEP);
      wb_emitter_send(communication, msgString2, strlen(msgString2) + 1);
      wb_emitter_send(communication, msgString3, strlen(msgString3) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 0, "Case 5 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          0, queueLength);

      break;

    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 2, "Case 5 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          2, queueLength);

      // check packets content
      buffer = wb_receiver_get_data(communication);

      ts_assert_string_equal(buffer, msgString1,
                             "Case 5 - Receiver \"receiver%d\" should receive a first packet contaning \"%c\" not \"%c\"",
                             robotID, msgString1, buffer);

      wb_receiver_next_packet(communication);

      buffer = wb_receiver_get_data(communication);
      ts_assert_string_equal(buffer, msgString2,
                             "Case 5 - Receiver \"receiver%d\" should receive a second packet contaning \"%c\" not \"%c\"",
                             robotID, msgString2, buffer);

      wb_receiver_next_packet(communication);
      // 3. message was too big and the buffer does not have enough space -> destroyed

      break;

    default:
      break;
  }

  wb_robot_step(TIME_STEP);
  // CASE 6: Emitter broadcast and check signal strength and emitter direction
  // e0: channelALL, e1: channel3, r0: channelALL (disabled), r2: channel1, r3: channel3
  // communication: e0 -> r2, msgString1
  //                e0 -> r3, msgString1
  //                e1 -> r3, msgString2
  switch (robotID) {
    case 0:
      wb_emitter_set_channel(communication, channelAll);
      wb_emitter_send(communication, msgString1, strlen(msgString1) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 1:
      wb_robot_step(TIME_STEP);
      wb_emitter_send(communication, msgString2, strlen(msgString2) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2: {
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 1, "Case 6 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          1, queueLength);

      // check packet content
      buffer = wb_receiver_get_data(communication);
      ts_assert_string_equal(buffer, msgString1,
                             "Case 6 - Receiver \"receiver%d\" should receive packet contaning \"%c\" not \"%c\"", robotID,
                             msgString1, buffer);

      // check emitter direction
      dbuffer = wb_receiver_get_emitter_direction(communication);
      const double expected[3] = {-0.7071, 0.0, -0.7071};
      ts_assert_doubles_in_delta(
        3, dbuffer, expected, 0.0001,
        "Case 6 - Receiver \"receiver%d\" should receive packet from the emitter in direction [%f, %f, %f] not [%f, %f, %f]",
        robotID, expected[0], expected[1], expected[2], dbuffer[0], dbuffer[1], dbuffer[2]);

      // check signal strength
      signalStrength = wb_receiver_get_signal_strength(communication);
      ts_assert_double_in_delta(signalStrength, 1.3889, 0.0001,
                                "Case 6 - Receiver \"receiver%d\" should receive packet with signal strength %f not %f",
                                robotID, 1.3889, signalStrength);

      wb_receiver_next_packet(communication);
      wb_receiver_set_channel(communication, channelAll);

      break;
    }
    case 3: {
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 2, "Case 6 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          2, queueLength);

      // check packets content
      buffer = wb_receiver_get_data(communication);
      ts_assert_string_equal(buffer, msgString1,
                             "Case 6 - Receiver \"receiver%d\" should receive a first packet contaning \"%c\" not \"%c\"",
                             robotID, msgString1, buffer);

      // check emitter direction
      dbuffer = wb_receiver_get_emitter_direction(communication);
      const double expected[3] = {-1.0, 0.0, 0.0};
      ts_assert_doubles_in_delta(3, dbuffer, expected, 0.0001,
                                 "Case 6 - Receiver \"receiver%d\" should receive a first packet from the emitter in direction "
                                 "[%f, %f, %f] not [%f, %f, %f]",
                                 robotID, expected[0], expected[1], expected[2], dbuffer[0], dbuffer[1], dbuffer[2]);

      // check signal strength
      signalStrength = wb_receiver_get_signal_strength(communication);
      ts_assert_double_in_delta(signalStrength, 2.7778, 0.0001,
                                "Case 6 - Receiver \"receiver%d\" should receive packet with signal strength %f not %f",
                                robotID, 2.7778, signalStrength);

      wb_receiver_next_packet(communication);

      // check packets content
      buffer = wb_receiver_get_data(communication);
      ts_assert_string_equal(buffer, msgString2,
                             "Case 6 - Receiver \"receiver%d\" should receive a second packet contaning \"%c\" not \"%c\"",
                             robotID, msgString2, buffer);

      // check emitter direction
      dbuffer = wb_receiver_get_emitter_direction(communication);
      const double expected1[3] = {-0.7071, 0.0, +0.7071};
      ts_assert_doubles_in_delta(3, dbuffer, expected1, 0.0001,
                                 "Case 6 - Receiver \"receiver%d\" should receive a second packet from the emitter in "
                                 "direction [%f, %f, %f] not [%f, %f, %f]",
                                 robotID, expected1[0], expected1[1], expected1[2], dbuffer[0], dbuffer[1], dbuffer[2]);

      // check signal strength
      signalStrength = wb_receiver_get_signal_strength(communication);
      ts_assert_double_in_delta(signalStrength, 1.3889, 0.0001,
                                "Case 6 - Receiver \"receiver%d\" should receive packet with signal strength %f not %f",
                                robotID, 1.3889, signalStrength);

      wb_receiver_next_packet(communication);

      wb_receiver_disable(communication);
      break;
    }
    default:
      break;
  }

  wb_robot_step(TIME_STEP);
  // CASE 7: Disable receiver and check sending and receiving on the same robot
  // e0: channelALL, e1: channel3, r0: channelALL, r2: channelALL, r3: channel3 (disabled)
  // communication: e0 -> r2, msgString1
  //                e1 -> r2, msgString2
  //                e1 -> r0, msgString2
  switch (robotID) {
    case 0:
      wb_receiver_enable(receiver0, TIME_STEP);

      wb_emitter_send(communication, msgString1, strlen(msgString1) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);

      queueLength = wb_receiver_get_queue_length(receiver0);

      // Not possible to send and receive data from the same robot
      ts_assert_int_equal(queueLength, 1, "Case 7 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          1, queueLength);

      buffer = wb_receiver_get_data(receiver0);
      ts_assert_string_equal(buffer, msgString2,
                             "Case 7 - Receiver \"receiver%d\" should receive packet contaning \"%c\" not \"%c\"", robotID,
                             msgString2, buffer);

      wb_receiver_next_packet(receiver0);

      samplingPeriod = wb_receiver_get_sampling_period(receiver0);
      ts_assert_int_equal(samplingPeriod, TIME_STEP,
                          "Case 7 - Receiver \"receiver%d\" should have a sampling period %d not %d when enabled", robotID,
                          TIME_STEP, samplingPeriod);

      wb_receiver_disable(receiver0);

      break;

    case 1:
      wb_robot_step(TIME_STEP);
      wb_emitter_send(communication, msgString2, strlen(msgString2) + 1);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 2, "Case 7 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          2, queueLength);

      buffer = wb_receiver_get_data(communication);
      ts_assert_string_equal(buffer, msgString1,
                             "Case 7 - Receiver \"receiver%d\" should receive a first packet contaning \"%c\" not \"%c\"",
                             robotID, msgString1, buffer);

      wb_receiver_next_packet(communication);

      buffer = wb_receiver_get_data(communication);
      ts_assert_string_equal(buffer, msgString2,
                             "Case 7 - Receiver \"receiver%d\" should receive a second packet contaning \"%c\" not \"%c\"",
                             robotID, msgString2, buffer);

      wb_receiver_next_packet(communication);

      samplingPeriod = wb_receiver_get_sampling_period(communication);
      ts_assert_int_equal(samplingPeriod, TIME_STEP,
                          "Case 7 - Receiver \"receiver%d\" should have a sampling period %d not %d when enabled", TIME_STEP,
                          robotID, samplingPeriod);

      break;

    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 0,
                          "Case 7 - Receiver \"receiver%d\" should receive %d packet not %d packets when disabled", robotID, 0,
                          queueLength);

      samplingPeriod = wb_receiver_get_sampling_period(communication);
      ts_assert_int_equal(samplingPeriod, 0,
                          "Case 7 - Receiver \"receiver%d\" should have a sampling period %d not %d when disabled", robotID, 0,
                          samplingPeriod);

      break;

    default:
      break;
  }

  wb_robot_step(TIME_STEP);
  // CASE 8: Check emitter buffer size
  // e0: channelALL, e1: channel3, r0: channelALL (disabled), r2: channelALL, r3: channel3 (disabled)
  // communication: e1 -> r2, msgString2
  switch (robotID) {
    case 0:
      bufferSize = wb_emitter_get_buffer_size(communication);
      ts_assert_int_equal(bufferSize, 100, "Case 8 - Emitter \"emitter%d\" should have a buffer size %d not %d", robotID, 100,
                          bufferSize);

      // bigger than the buffer -> not sent
      // wb_emitter_send(communication, msgString3, strlen(msgString3) + 1);

      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      wb_receiver_enable(receiver0, TIME_STEP);
      break;

    case 1:
      bufferSize = wb_emitter_get_buffer_size(communication);
      ts_assert_int_equal(bufferSize, -1, "Case 8 - Emitter \"emitter%d\" should have a buffer size %d not %d", robotID, 4096,
                          bufferSize);

      wb_emitter_send(communication, msgString2, strlen(msgString2) + 1);

      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 1, "Case 8 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          1, queueLength);

      buffer = wb_receiver_get_data(communication);
      ts_assert_string_equal(buffer, msgString2,
                             "Case 8 - Receiver \"receiver%d\" should receive a first packet contaning \"%c\" not \"%c\"",
                             robotID, msgString2, buffer);

      wb_receiver_next_packet(communication);
      break;

    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 0,
                          "Case 8 - Receiver \"receiver%d\" should receive %d packet not %d packets when disabled", robotID, 0,
                          queueLength);

      wb_receiver_enable(communication, TIME_STEP);
      break;

    default:
      break;
  }

  wb_robot_step(TIME_STEP);
  // CASE 9: Check emitter range
  // e0: channelALL, e1: channel3, r0: channelALL , r2: channelALL, r3: channel3
  // communication: e1 -> r0, msgString3
  //                e1 -> r2, msgString3
  switch (robotID) {
    case 0:
      range = wb_emitter_get_range(communication);
      ts_assert_double_in_delta(range, -1, 0.001, "Case 9 - Emitter \"emitter%d\" should have range %d not %d", robotID, -1,
                                range);

      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);

      queueLength = wb_receiver_get_queue_length(receiver0);
      ts_assert_int_equal(queueLength, 1, "Case 9 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          1, queueLength);

      buffer = wb_receiver_get_data(receiver0);
      ts_assert_string_equal(buffer, msgString2,
                             "Case 9 - Receiver \"receiver%d\" should receive a first packet contaning \"%c\" not \"%c\"",
                             robotID, msgString2, buffer);

      wb_receiver_next_packet(receiver0);

      break;

    case 1:
      range = wb_emitter_get_range(communication);
      ts_assert_double_in_delta(range, 1.3, 0.001,
                                "Case 9 - Emitter \"emitter%d\" should have range %d not %d before changing it", robotID, 1.3,
                                range);

      wb_emitter_set_range(communication, 2.5);

      range = wb_emitter_get_range(communication);
      ts_assert_double_in_delta(
        range, 2.0, 0.001,
        "Case 9 - Emitter \"emitter%d\" should have range %f not %f if setting a value greater than max range %f", robotID, 2.0,
        range, 2.0);

      wb_emitter_set_range(communication, 0.7);

      range = wb_emitter_get_range(communication);
      ts_assert_double_in_delta(
        range, 0.7, 0.001, "Case 9 - Emitter \"emitter%d\" should have range %f not %f after changing it", robotID, 0.7, range);

      wb_emitter_send(communication, msgString2, strlen(msgString2) + 1);

      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      break;

    case 2:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(queueLength, 1, "Case 9 - Receiver \"receiver%d\" should receive %d packets not %d packets", robotID,
                          1, queueLength);

      buffer = wb_receiver_get_data(communication);
      ts_assert_string_equal(buffer, msgString2,
                             "Case 9 - Receiver \"receiver%d\" should receive a first packet contaning \"%c\" not \"%c\"",
                             robotID, msgString2, buffer);

      wb_receiver_next_packet(communication);
      break;

    case 3:
      wb_robot_step(TIME_STEP);
      wb_robot_step(TIME_STEP);
      queueLength = wb_receiver_get_queue_length(communication);
      ts_assert_int_equal(
        queueLength, 0,
        "Case 9 - Receiver \"receiver%d\" should receive %d packet not %d packets when distance larger that valid range",
        robotID, 0, queueLength);

      break;

    default:
      break;
  }
  wb_robot_step(robotID * TIME_STEP);  // avoid collision of simultaneous success messages on stdout
  ts_send_success();
  return EXIT_SUCCESS;
}
