/*
 * Description:  Test Emitters and Receivers in a physics plugin.
 *               This test is based on the communication between emitter and receivers
 *               controlled in this controller file, and the emitter and receiver of the
 *               physics plugin 'emitter_receiver_physics.c'.
 *               The correct reception of the message is checked by sending back a message
 *               to the controller containing the received values.
 *
 *               This file contains some tests of the emission of messages:
 *                 - communication with radio/infra-red emitter
 *                 - emitter/receiver broadcast
 *                 - communication from physics plugin to physics plugin
 *                 - reduced emitter range
 *                 - reduced emitter/receiver aperture
 */

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag emitter_radio, emitter_infra_red, emitter_infra_red_reduced;
  WbDeviceTag receiver_radio, receiver_infra_red, receiver_infra_red_reduced;
  int queueLength;
  const char *buffer;
  bool valid;
  int i;

  // messages
  double msg1[1] = {0.13};
  double msg2[2] = {0.24, 0.27};
  double msg7[7] = {1, 2, 3, 4, 5, 6, 7};

  emitter_radio = wb_robot_get_device("emitter_radio");                            // radio,     channel 0
  receiver_radio = wb_robot_get_device("receiver_radio");                          // radio,     channel 1
  emitter_infra_red = wb_robot_get_device("emitter_infra-red");                    // infra-red, channel 0
  receiver_infra_red = wb_robot_get_device("receiver_infra-red");                  // infra-red, channel 2 aperture -1
  emitter_infra_red_reduced = wb_robot_get_device("emitter_infra-red_reduced");    // infra-red, channel 0 aperture 0
  receiver_infra_red_reduced = wb_robot_get_device("receiver_infra-red_reduced");  // infra-red, channel 2 aperture 0
  wb_receiver_enable(receiver_radio, TIME_STEP);
  wb_receiver_enable(receiver_infra_red, TIME_STEP);
  wb_receiver_disable(receiver_infra_red_reduced);

  // CASE 1: send message to the physics plugin from radio emitter
  wb_emitter_send(emitter_radio, msg1, 1 * sizeof(double));

  // wait for answer
  i = 0;
  valid = false;
  queueLength = 0;
  while (!valid && i < 5) {
    wb_robot_step(TIME_STEP);
    queueLength = wb_receiver_get_queue_length(receiver_radio);
    if (queueLength > 0)
      valid = true;
    i++;
  }

  ts_assert_boolean_equal(valid, "No answer received from the physics plugin when emitting on channel 1");

  // check queueLength of radio receiver
  ts_assert_int_equal(queueLength, 1, "Radio receiver should receive %d packets not %d packets after emitting msg1", 1,
                      queueLength);

  // check received data
  buffer = wb_receiver_get_data(receiver_radio);
  ts_assert_string_equal(
    buffer, "The value sent is: 0.13",
    "Radio receiver should receive packet sent by the physics emitter containing \"%s\" not \"%s\" after emitting msg1",
    "The value sent is: 0.13", buffer);
  wb_receiver_next_packet(receiver_radio);

  // check queueLength of infra-red receiver
  queueLength = wb_receiver_get_queue_length(receiver_infra_red);
  ts_assert_int_equal(queueLength, 0, "Infra-red receiver should receive %d packets not %d packets after emitting msg1", 0,
                      queueLength);

  // CASE 2: send message to the physics plugin from infra-red emitter
  wb_robot_step(TIME_STEP);
  wb_emitter_send(emitter_infra_red, msg2, 2 * sizeof(double));

  // wait for answer
  i = 0;
  valid = false;
  queueLength = 0;
  while (!valid && i < 5) {
    wb_robot_step(TIME_STEP);
    queueLength = wb_receiver_get_queue_length(receiver_infra_red);
    if (queueLength > 0)
      valid = true;
    i++;
  }

  ts_assert_boolean_equal(valid, "No answer received from the physics plugin when emitting msg2");

  // check queueLength of infra-red receiver
  queueLength = wb_receiver_get_queue_length(receiver_infra_red);
  ts_assert_int_equal(queueLength, 1, "Infra-red receiver should receive %d packets not %d packets after emitting msg2", 1,
                      queueLength);

  // check received data
  buffer = wb_receiver_get_data(receiver_infra_red);
  ts_assert_string_equal(
    buffer, "The value sent is: [0.24, 0.27]",
    "Infra-red receiver should receive packet sent by the physics emitter containing \"%s\" not \"%s\" after emitting msg2",
    "The value sent is: [0.24, 0.27]", buffer);
  wb_receiver_next_packet(receiver_infra_red);

  // check queueLength of infra-red receiver
  queueLength = wb_receiver_get_queue_length(receiver_radio);
  ts_assert_int_equal(queueLength, 0, "Radio receiver should receive %d packets not %d packets after emitting msg2", 0,
                      queueLength);

  // CASE 3: send message on channel -1 from infra-red emitter
  wb_robot_step(TIME_STEP);
  wb_emitter_set_channel(emitter_infra_red, -1);
  wb_emitter_send(emitter_infra_red, msg2, 2 * sizeof(double));

  // wait for answer
  i = 0;
  valid = false;
  queueLength = 0;
  while (!valid && i < 20) {
    wb_robot_step(TIME_STEP);
    queueLength = wb_receiver_get_queue_length(receiver_infra_red);
    if (queueLength > 0)
      valid = true;
    i++;
  }

  ts_assert_boolean_not_equal(
    valid,
    "Infra-red receiver should receive 0 packets, i.e. they physics plugin should not receive any packets sent on channel -1");

  // CASE 4: send message on channel 2 from infra-red emitter
  wb_robot_step(TIME_STEP);
  wb_emitter_set_channel(emitter_infra_red, 3);
  wb_emitter_send(emitter_infra_red, msg2, 2 * sizeof(double));

  // wait for answer
  i = 0;
  valid = false;
  queueLength = 0;
  while (!valid && i < 20) {
    wb_robot_step(TIME_STEP);
    queueLength = wb_receiver_get_queue_length(receiver_infra_red);
    if (queueLength > 0)
      valid = true;
    i++;
  }

  ts_assert_boolean_not_equal(
    valid,
    "Infra-red receiver should receive 0 packets, i.e. they physics plugin should not receive any packets sent on channel > 0");

  // CASE 5: send message to physics plugin from radio emitter
  //         test sending a message from physics plugin to physics plugin
  wb_robot_step(TIME_STEP);
  wb_emitter_send(emitter_radio, msg7, 7 * sizeof(double));

  // wait for answer
  i = 0;
  valid = false;
  queueLength = 0;
  while (!valid && i < 20) {
    wb_robot_step(TIME_STEP);
    queueLength = wb_receiver_get_queue_length(receiver_radio);
    if (queueLength > 0)
      valid = true;
    i++;
  }

  ts_assert_boolean_not_equal(
    valid,
    "Infra-red receiver should receive 0 packets, i.e. they physics plugin should not be able to send a message to itself");

  // CASE 6: reduced emitter range
  wb_emitter_set_range(emitter_radio, 0.0);
  wb_emitter_send(emitter_radio, msg1, 1 * sizeof(double));

  // wait for answer
  i = 0;
  valid = false;
  queueLength = 0;
  while (!valid && i < 5) {
    wb_robot_step(TIME_STEP);
    queueLength = wb_receiver_get_queue_length(receiver_radio);
    if (queueLength > 0)
      valid = true;
    i++;
  }

  ts_assert_boolean_equal(valid, "No answer received from the physics plugin when emitting with range 0.0");

  // check queueLength of radio receiver
  ts_assert_int_equal(queueLength, 1,
                      "Radio receiver should receive %d packets not %d packets after emitting msg1  with range 0.0", 1,
                      queueLength);

  // check received data
  buffer = wb_receiver_get_data(receiver_radio);
  ts_assert_string_equal(buffer, "The value sent is: 0.13",
                         "Radio receiver should receive packet sent by the physics emitter containing \"%s\" not \"%s\" after "
                         "emitting msg1 with range 0.0",
                         "The value sent is: 0.13", buffer);
  wb_receiver_next_packet(receiver_radio);

  // CASE 7: reduced emitter aperture
  wb_emitter_send(emitter_infra_red_reduced, msg2, 2 * sizeof(double));

  // wait for answer
  i = 0;
  valid = false;
  queueLength = 0;
  while (!valid && i < 5) {
    wb_robot_step(TIME_STEP);
    queueLength = wb_receiver_get_queue_length(receiver_infra_red);
    if (queueLength > 0)
      valid = true;
    i++;
  }

  ts_assert_boolean_equal(valid, "No answer received from the physics plugin when emitting with aperture 0.0");

  // check queueLength of infra-red reduced receiver
  ts_assert_int_equal(queueLength, 1,
                      "Infra-red receiver should receive %d packets not %d packets after emitting msg2 with aperture 0.0", 1,
                      queueLength);

  // check received data
  buffer = wb_receiver_get_data(receiver_infra_red);
  ts_assert_string_equal(buffer, "The value sent is: [0.24, 0.27]",
                         "Infra-red receiver should receive packet sent by the physics emitter containing \"%s\" not \"%s\" "
                         "after emitting msg2 with aperture 0.0",
                         "The value sent is: [0.24, 0.27]", buffer);
  wb_receiver_next_packet(receiver_infra_red);

  // CASE 8: reduced receiver aperture and disabled receiver
  wb_receiver_enable(receiver_infra_red_reduced, TIME_STEP);
  wb_receiver_disable(receiver_infra_red);
  wb_emitter_set_channel(emitter_infra_red, 0);
  queueLength = wb_receiver_get_queue_length(receiver_infra_red_reduced);
  wb_emitter_send(emitter_infra_red, msg2, 2 * sizeof(double));
  // wait for answer
  i = 0;
  valid = false;
  queueLength = 0;
  while (!valid && i < 5) {
    wb_robot_step(TIME_STEP);
    queueLength = wb_receiver_get_queue_length(receiver_infra_red_reduced);
    if (queueLength > 0)
      valid = true;
    i++;
  }

  ts_assert_boolean_equal(valid,
                          "No answer received from the physics plugin when emitting on channel 2 with receiver aperture 0.0");

  // check queueLength of infra-red reduced receiver
  ts_assert_int_equal(queueLength, 1,
                      "Infra-red receiver with aperture 0.0 should receive %d packets not %d packets after emitting msg2", 1,
                      queueLength);

  // check received data
  buffer = wb_receiver_get_data(receiver_infra_red_reduced);
  ts_assert_string_equal(buffer, "The value sent is: [0.24, 0.27]",
                         "Infra-red receiver with aperture 0.0 should receive packet sent by the physics emitter containing "
                         "\"%s\" not \"%s\" after emitting msg2",
                         "The value sent is: [0.24, 0.27]", buffer);
  wb_receiver_next_packet(receiver_infra_red_reduced);

  // check queueLength of infra-red receiver
  queueLength = wb_receiver_get_queue_length(receiver_infra_red);
  ts_assert_int_equal(
    queueLength, 0, "Disabled infra-red receiver should receive %d packets not %d packets after emitting msg2", 0, queueLength);

  ts_send_success();
  return EXIT_SUCCESS;
}
