#include <webots/differential_wheels.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

static void checkQueueLength(WbDeviceTag tag, const char *sensor_name, int expected, int checkNumber) {
  int count = wb_receiver_get_queue_length(tag);
  ts_assert_int_equal(count, expected,
                      "Wrong number of messages received from %d emitter at check %d: "
                      "expected %d, received %d.",
                      sensor_name, checkNumber, expected, count);
}

static void checkLastEmitterDirection(WbDeviceTag tag, const char *sensor_name, double expectedX, double expectedY,
                                      double expectedZ, int checkNumber) {
  while (wb_receiver_get_queue_length(tag) > 1)
    wb_receiver_next_packet(tag);

  // check emitter direction
  if (wb_receiver_get_queue_length(tag) == 1) {
    const double *emitterDir = wb_receiver_get_emitter_direction(tag);
    ts_assert_vec3_in_delta(emitterDir[0], emitterDir[1], emitterDir[2], expectedX, expectedY, expectedZ, 0.01,
                            "Wrong %s emitter direction for the message received at check %d.", sensor_name, checkNumber);
    wb_receiver_next_packet(tag);
  }
}

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  int nCheck = 1;
  int time_step = wb_robot_get_basic_time_step();
  const char *data = wb_robot_get_custom_data();
  bool dynamic = true;
  if (strcmp(data, "static") == 0)
    dynamic = false;

  // radio type (no ODE rays)
  WbDeviceTag rrs = wb_robot_get_device("receiver radio static");
  WbDeviceTag rrd = wb_robot_get_device("receiver radio dynamic");
  // infra-red type (using ODE rays)
  WbDeviceTag ris = wb_robot_get_device("receiver infra-red static");
  WbDeviceTag rid = wb_robot_get_device("receiver infra-red dynamic");
  wb_receiver_enable(rrs, time_step);
  wb_receiver_enable(rrd, time_step);
  wb_receiver_enable(ris, time_step);
  wb_receiver_enable(rid, time_step);

  // stabilize the system
  wb_robot_step(3 * time_step);

  if (dynamic)
    // move sensor devices during ODE physics step
    wb_differential_wheels_set_speed(20, 20);

  // static emitter
  checkQueueLength(rrs, "static radio", 3, nCheck);
  if (dynamic)
    checkLastEmitterDirection(rrs, "static radio", 0.00, 0.06, 0.99, nCheck);
  else
    checkLastEmitterDirection(rrs, "static radio", 0.00, 0.00, 1.00, nCheck);

  // dynamic emitter
  // checkQueueLength(rrd, "dynamic radio", 1, nCheck);
  wb_receiver_next_packet(rrd);
  wb_receiver_next_packet(rrd);

  // static emitter
  checkQueueLength(ris, "static infra-red", 3, nCheck);
  if (dynamic)
    checkLastEmitterDirection(ris, "static infra-red", 0.00, 0.06, 0.99, nCheck);
  else
    checkLastEmitterDirection(ris, "static infra-red", 0.00, 0.00, 1.00, nCheck);

  // dynamic
  checkQueueLength(rid, "dynamic infra-red", 0, nCheck);

  wb_robot_step(time_step);
  nCheck++;

  // radio type (no ODE rays)
  // static emitter
  checkQueueLength(rrs, "static radio", 1, nCheck);
  if (dynamic)
    checkLastEmitterDirection(rrs, "static radio", -0.18, 0.02, 0.98, nCheck);
  else
    checkLastEmitterDirection(rrs, "static radio", 0.00, 0.00, 1.00, nCheck);

  // dynamic emitter
  checkQueueLength(rrd, "dynamic radio", 1, nCheck);
  if (dynamic)
    checkLastEmitterDirection(rrd, "dynamic radio", 0.12, -0.09, 0.98, nCheck);
  else
    checkLastEmitterDirection(rrd, "dynamic radio", 0.00, -0.13, 0.99, nCheck);

  // infra-red type (using ODE rays)
  // static emitter
  checkQueueLength(ris, "static infra-red", 1, nCheck);
  if (dynamic)
    checkLastEmitterDirection(ris, "static infra-red", -0.18, 0.02, 0.98, nCheck);
  else
    checkLastEmitterDirection(ris, "static infra-red", 0.00, 0.00, 1.00, nCheck);

  // dynamic
  checkQueueLength(rid, "dynamic infra-red", 1, nCheck);
  if (dynamic)
    checkLastEmitterDirection(rid, "dynamic infra-red", 0.12, -0.09, 0.98, nCheck);
  else
    checkLastEmitterDirection(rid, "dynamic infra-red", 0.00, -0.13, 0.99, nCheck);

  wb_robot_step(time_step);
  nCheck++;

  // radio type (no ODE rays)
  // static emitter
  checkQueueLength(rrs, "static radio", 1, nCheck);
  if (dynamic)
    checkLastEmitterDirection(rrs, "static radio", -0.32, -0.09, 0.94, nCheck);
  else
    checkLastEmitterDirection(rrs, "static radio", 0.00, 0.00, 1.00, nCheck);

  // dynamic
  if (dynamic)
    checkQueueLength(rrd, "dynamic radio", 1, nCheck);
  else
    checkQueueLength(rrd, "dynamic radio", 0, nCheck);

  // infra-red type (using ODE rays)
  // static emitter
  if (dynamic)
    checkQueueLength(ris, "static infra-red", 0, nCheck);
  else {
    checkQueueLength(ris, "static infra-red", 1, nCheck);
    checkLastEmitterDirection(ris, "static infra-red", 0.00, 0.00, 1.00, nCheck);
  }

  // dynamic
  checkQueueLength(rid, "dynamic infra-red", 0, nCheck);

  ts_send_success();
  return EXIT_SUCCESS;
}
