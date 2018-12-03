/*
 * Description:  Test robustness Emitter and Receiver function calling them
 *               with wrong arguments.
 */

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define IS_VERBOSE 0

void sendLog(const char *text) {
  if (IS_VERBOSE != 0)
    printf("%s\n", text);
}

int main(int argc, char **argv) {
  ts_setup(argv[1]);
  WbDeviceTag emitter = 0;
  WbDeviceTag receiver = 0;
  WbDeviceTag otherDevice = 0;
  int queueLength, i;
  double d;
  const char *buffer;
  const double *doubleArray;

  // messages
  const char *msgString0 = "Hello";

  if (strcmp(wb_robot_get_name(), "robot_emitter") == 0) {
    emitter = wb_robot_get_device("emitter");  // radio,    channel 2
    otherDevice = wb_robot_get_device("receiver");
  } else if (strcmp(wb_robot_get_name(), "robot_receiver") == 0) {
    receiver = wb_robot_get_device("receiver");  // radio,    channel -1
    otherDevice = wb_robot_get_device("gps");
  }

  i = wb_receiver_get_sampling_period(receiver);
  ts_assert_int_equal(i, 0, "Get sampling period of a disabled receiver should return 0 not %d", i);

  wb_robot_step(TIME_STEP);

  // TEST 1: enable/disable invalid receiver
  if (receiver) {
    wb_receiver_enable(receiver, TIME_STEP + 5);
    i = wb_receiver_get_sampling_period(receiver);
    ts_assert_int_equal(i, TIME_STEP + 5, "Get sampling period of a just enabled receiver should return %d not %d",
                        TIME_STEP + 5, i);

    sendLog("Enable NULL receiver");
    wb_receiver_enable(0, TIME_STEP);
    i = wb_receiver_get_sampling_period(0);
    ts_assert_int_equal(i, 0, "Get sampling period of a NULL receiver should return %d not %d", 0, TIME_STEP);
    sendLog("Disable NULL receiver");
    wb_receiver_disable(0);

    sendLog("Enable an invalid receiver");
    wb_receiver_enable(otherDevice, TIME_STEP);
    i = wb_receiver_get_sampling_period(otherDevice);
    ts_assert_int_equal(i, 0, "Get sampling period of an invalid receiver should return %d not %d", 0, TIME_STEP);
    sendLog("Disable an invalid receiver");
    wb_receiver_disable(otherDevice);

    wb_robot_step(TIME_STEP);

    sendLog("Enable an already enabled receiver with different sampling period");
    wb_receiver_enable(receiver, TIME_STEP);
    i = wb_receiver_get_sampling_period(receiver);
    ts_assert_int_equal(i, TIME_STEP, "Get sampling period of an already enabled receiver should be changed to %d not %d",
                        TIME_STEP, i);

    wb_robot_step(TIME_STEP);

    sendLog("Enable an already enabled receiver with sampling period < 0");
    wb_receiver_enable(receiver, -7);
    i = wb_receiver_get_sampling_period(receiver);
    ts_assert_int_equal(i, TIME_STEP,
                        "Get sampling period of an enabled receiver should be %d not %d: sampling period < 0 is not valid",
                        TIME_STEP, i);

  } else if (emitter) {
    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);
  }

  // TEST 2:  send message on invalid emitter
  wb_robot_step(TIME_STEP);
  if (receiver) {
    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);

    i = wb_receiver_get_queue_length(receiver);
    ts_assert_int_equal(i, 0, "Receiver should receive 0 not %d packets when send from a NULL emitter", i);

    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);

    i = wb_receiver_get_queue_length(receiver);
    ts_assert_int_equal(i, 0, "Receiver should receive 0 not %d packets when send from an invalid emitter", i);

  } else if (emitter) {
    sendLog("Send data from a NULL emitter");
    wb_emitter_send(0, msgString0, strlen(msgString0) + 1);

    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);

    sendLog("Send data from an invalid emitter");
    wb_emitter_send(otherDevice, msgString0, strlen(msgString0) + 1);

    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);
  }

  // TEST 3:  send NULL message on valid emitter
  wb_robot_step(TIME_STEP);
  if (receiver) {
    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);

    i = wb_receiver_get_queue_length(receiver);
    ts_assert_int_equal(i, 0, "Receiver should receive 0 not %d packets when NULL data is sent", i);

    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);
    // TODO
    i = wb_receiver_get_queue_length(receiver);
    ts_assert_int_equal(i, 0, "Receiver should receive 0 not %d packets when data with wrong size is sent", i);
    /*buffer = wb_receiver_get_data(receiver);
    ts_assert_string_equal(buffer, "He",
      "Receiver should receive data \"%s\" not \"%s\" when data with wrong size is sent", "He", buffer);
    wb_receiver_next_packet(receiver);
    */

  } else if (emitter) {
    sendLog("Send NULL data from a valid emitter");
    wb_emitter_send(emitter, NULL, strlen(msgString0) + 1);

    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);

    sendLog("Send data with wrong size from a avalid emitter");
    wb_emitter_send(otherDevice, msgString0, strlen(msgString0) - 3);

    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);
  }

  // TEST 4: set/get channel on invalid device
  if (receiver) {
    sendLog("Set channel with NULL receiver");
    wb_receiver_set_channel(0, 5);
    sendLog("Get channel of a NULL receiver");
    i = wb_receiver_get_channel(0);
    ts_assert_int_equal(i, -1, "Get channel of a NULL receiver should return %d not %d", -1, i);

    sendLog("Set channel with invalid receiver");
    wb_receiver_set_channel(otherDevice, 5);
    sendLog("Get channel of an invalid receiver");
    i = wb_receiver_get_channel(otherDevice);
    ts_assert_int_equal(i, -1, "Get channel of an invalid receiver should return %d not %d", -1, i);

    wb_receiver_disable(receiver);
    wb_robot_step(TIME_STEP);

    sendLog("Set channel of a disabled receiver");
    wb_receiver_set_channel(receiver, 7);
    i = wb_receiver_get_channel(receiver);
    ts_assert_int_equal(i, 7, "Setting the channel of a disabled receiver should possible, channel should be %d not %d", 7, i);

    wb_receiver_enable(receiver, TIME_STEP);
    wb_robot_step(TIME_STEP);

    sendLog("Set channel of a valid receiver to a negative value");
    wb_receiver_set_channel(receiver, -5);
    i = wb_receiver_get_channel(receiver);
    ts_assert_int_equal(
      i, 7, "Setting the channel of a receiver to a negative value should not be possible, channel should be %d not %d", 7, i);

  } else if (emitter) {
    sendLog("Set channel with NULL emitter");
    wb_emitter_set_channel(0, 5);
    sendLog("Get channel of a NULL emitter");
    i = wb_emitter_get_channel(0);
    ts_assert_int_equal(i, -1, "Get channel of a NULL emitter should return %d not %d", -1, i);

    sendLog("Set channel with invalid emitter");
    wb_emitter_set_channel(otherDevice, 5);
    sendLog("Get channel of an invalid emitter");
    i = wb_emitter_get_channel(otherDevice);
    ts_assert_int_equal(i, -1, "Get channel of an invalid emitter should return %d not %d", -1, i);

    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);

    sendLog("Set channel of a valid emitter to a negative value");
    wb_emitter_set_channel(emitter, -5);
    i = wb_emitter_get_channel(emitter);
    ts_assert_int_equal(
      i, 2, "Setting the channel of an emitter to a negative value should not be possible, channel should be should %d not %d",
      2, i);
  }

  wb_robot_step(TIME_STEP);
  if (receiver) {
    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);
  } else if (emitter) {
    // TEST 5: set/get range on invalid emitter

    sendLog("Set range with NULL emitter");
    wb_emitter_set_range(0, 0.33);
    sendLog("Get range of a NULL emitter");
    d = wb_emitter_get_range(0);
    ts_assert_double_equal(d, NAN, "Get range of a NULL emitter should return NAN not %f", d);

    wb_robot_step(TIME_STEP);

    sendLog("Set range with invalid emitter");
    wb_emitter_set_range(otherDevice, 0.33);
    sendLog("Get range of an invalid emitter");
    d = wb_emitter_get_range(otherDevice);
    ts_assert_double_equal(d, NAN, "Get range of an invalid emitter should return NAN not %f", d);

    sendLog("Set range of a valid emitter to a negative value");
    wb_emitter_set_range(emitter, -0.33);
    d = wb_emitter_get_range(emitter);
    ts_assert_double_equal(
      d, 0.8, "Setting the range of an emitter to a negative value should not be possible, range should be should %d not %f",
      0.8, d);

    // TEST 6: get buffer size on invalid emitter
    wb_robot_step(TIME_STEP);
    sendLog("Get buffer size of a NULL emitter");
    i = wb_emitter_get_buffer_size(0);
    ts_assert_int_equal(i, -1, "Get buffer size of a NULL emitter should return -1 not %d", i);

    sendLog("Get buffer size of an invalid emitter");
    i = wb_emitter_get_buffer_size(otherDevice);
    ts_assert_int_equal(i, -1, "Get buffer size of an invalid emitter should return -1 not %d", i);

    ts_send_success();
    return EXIT_SUCCESS;
  }

  // ONLY receiver

  // TEST 6: get queue_legnth on invalid receiver
  wb_robot_step(TIME_STEP);
  sendLog("Get queue length of a NULL receiver");
  queueLength = wb_receiver_get_queue_length(0);
  ts_assert_int_equal(queueLength, -1, "Get queue length of a NULL emitter should return -1 not %d", queueLength);

  sendLog("Get queue length of an invalid receiver");
  queueLength = wb_receiver_get_queue_length(otherDevice);
  ts_assert_int_equal(queueLength, -1, "Get queue length of an invalid emitter should return -1 not %d", queueLength);

  wb_receiver_disable(receiver);
  wb_robot_step(TIME_STEP);

  sendLog("Get queue length of a disabled receiver");
  queueLength = wb_receiver_get_queue_length(receiver);
  ts_assert_int_equal(queueLength, 0, "Get queue length of a disabled emitter should return 0 not %d", queueLength);

  // TEST 7: call next packet on invalid receiver
  wb_robot_step(TIME_STEP);
  wb_receiver_disable(receiver);
  wb_robot_step(TIME_STEP);

  sendLog("Call next packet of a NULL receiver");
  wb_receiver_next_packet(0);

  sendLog("Call next packet of an invalid receiver");
  wb_receiver_next_packet(otherDevice);

  sendLog("Call next packet of a disabled receiver with empty reception list");
  wb_receiver_next_packet(receiver);

  wb_receiver_enable(receiver, TIME_STEP);
  wb_robot_step(TIME_STEP);

  sendLog("Call next packet of an enabled receiver with empty reception list");
  wb_receiver_next_packet(receiver);

  // TEST 8: get data anda data size on invalid receiver
  wb_robot_step(TIME_STEP);
  wb_receiver_disable(receiver);
  wb_robot_step(TIME_STEP);

  sendLog("Get data of a NULL receiver");
  buffer = wb_receiver_get_data(0);
  ts_assert_pointer_null((void *)buffer, "Get data of a NULL receiver should return NULL not \"%s\"", buffer);

  sendLog("Get data size of a NULL receiver");
  i = wb_receiver_get_data_size(0);
  ts_assert_int_equal(i, -1, "Get data size of a NULL receiver should return -1 not %d", i);

  sendLog("Get data of an invalid receiver");
  buffer = wb_receiver_get_data(otherDevice);
  ts_assert_pointer_null((void *)buffer, "Get data of an invalid receiver should return NULL not \"%s\"", buffer);

  sendLog("Get data size of an invalid receiver");
  i = wb_receiver_get_data_size(otherDevice);
  ts_assert_int_equal(i, -1, "Get data size of an invalid receiver should return -1 not %d", i);

  sendLog("Get data of a disabled receiver with empty reception list");
  buffer = wb_receiver_get_data(receiver);
  ts_assert_pointer_null((void *)buffer, "Get data of an invalid receiver should return NULL not \"%s\"", buffer);

  sendLog("Get data size of a disabled receiver with empty reception list");
  i = wb_receiver_get_data_size(receiver);
  ts_assert_int_equal(i, -1, "Get data size of a disabled receiver with empty reception list should return -1 not %d", i);

  wb_receiver_enable(receiver, TIME_STEP);
  wb_robot_step(TIME_STEP);

  sendLog("Get data of a valid receiver with empty reception list");
  buffer = wb_receiver_get_data(receiver);
  ts_assert_pointer_null((void *)buffer, "Get data of a valid receiver with empty reception list should return NULL not \"%s\"",
                         buffer);

  sendLog("Get data size of a valid receiver with empty reception list");
  i = wb_receiver_get_data_size(receiver);
  ts_assert_int_equal(i, -1, "Get data size of a valid receiver with empty reception list should return -1 not %d", i);

  // TEST 9: get signal strength on invalid receiver
  wb_robot_step(TIME_STEP);
  wb_receiver_disable(receiver);
  wb_robot_step(TIME_STEP);

  sendLog("Get signal strength of a NULL receiver");
  d = wb_receiver_get_signal_strength(0);
  ts_assert_double_equal(d, NAN, "Get signal strength of a NULL receiver should return NAN not %f", d);

  sendLog("Get signal strength of an invalid receiver");
  d = wb_receiver_get_signal_strength(otherDevice);
  ts_assert_double_equal(d, NAN, "Get signal strength of an invalid receiver should return NAN not %f", d);

  sendLog("Get signal strength of a disabled receiver with empty reception list");
  d = wb_receiver_get_signal_strength(receiver);
  ts_assert_double_equal(d, NAN, "Get signal strength of an invalid receiver should return NAN not %f", d);

  wb_receiver_enable(receiver, TIME_STEP);
  wb_robot_step(TIME_STEP);

  sendLog("Get signal strength of a valid receiver with empty reception list");
  d = wb_receiver_get_queue_length(receiver);
  ts_assert_int_equal(d, 0, "Get signal strength of a valid receiver with empty reception list should return 0 not %f", d);

  // TEST 10: get emitter direction on invalid receiver
  wb_robot_step(TIME_STEP);
  sendLog("Get emitter direction of a NULL receiver");
  doubleArray = wb_receiver_get_emitter_direction(0);
  ts_assert_pointer_null((void *)doubleArray, "Get emitter direction of a NULL emitter should return NULL");

  sendLog("Get emitter direction of an invalid receiver");
  doubleArray = wb_receiver_get_emitter_direction(otherDevice);
  ts_assert_pointer_null((void *)doubleArray, "Get emitter direction of an invalid emitter should return NULL");

  sendLog("Get emitter direction of a disabled receiver with empty reception list");
  doubleArray = wb_receiver_get_emitter_direction(receiver);
  ts_assert_pointer_null((void *)doubleArray, "Get emitter direction of an invalid receiver should return NULL");

  wb_receiver_enable(receiver, TIME_STEP);
  wb_robot_step(TIME_STEP);

  sendLog("Get emitter direction of a valid receiver with empty reception list");
  doubleArray = wb_receiver_get_emitter_direction(receiver);
  ts_assert_pointer_null((void *)doubleArray,
                         "Get emitter direction of a valid receiver with empty reception list should return NULL");

  ts_send_success();
  return EXIT_SUCCESS;
}
