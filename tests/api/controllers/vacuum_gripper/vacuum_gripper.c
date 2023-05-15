#include <stdio.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/vacuum_gripper.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define JOINT_STEP 0.1

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag motor = wb_robot_get_device("linear motor");

  WbDeviceTag vacuum_gripper = wb_robot_get_device("vacuum gripper");
  wb_vacuum_gripper_enable_presence(vacuum_gripper, TIME_STEP);

  WbNodeRef object = wb_supervisor_node_get_from_def("OBJECT");
  const double *p = wb_supervisor_node_get_position(object);
  const double p0[3] = {p[0], p[1], p[2]};

  wb_robot_step(5 * TIME_STEP);

  // lift end effector
  wb_motor_set_position(motor, JOINT_STEP);

  wb_robot_step(30 * TIME_STEP);

  ts_assert_boolean_equal(!wb_vacuum_gripper_is_on(vacuum_gripper), "Vacuum gripper should not be on at start.");
  ts_assert_boolean_equal(!wb_vacuum_gripper_get_presence(vacuum_gripper),
                          "Vacuum gripper should not be connected to objects when turned off.");
  ts_assert_int_equal(wb_vacuum_gripper_get_presence_sampling_period(vacuum_gripper), TIME_STEP,
                      "Vacuum gripper wrong sampling period.");

  const double *p1 = wb_supervisor_node_get_position(object);
  ts_assert_vec3_in_delta(p1[0], p1[1], p1[2], p0[0], p0[1], p0[2], 0.002, "Object shouldn't move before vacumm cup is on.");

  // move back end effector to initial position
  wb_motor_set_position(motor, 0);

  wb_robot_step(30 * TIME_STEP);

  wb_vacuum_gripper_turn_on(vacuum_gripper);

  wb_robot_step(TIME_STEP);

  ts_assert_boolean_equal(wb_vacuum_gripper_is_on(vacuum_gripper), "Vacuum gripper should be turned on.");
  ts_assert_boolean_equal(wb_vacuum_gripper_get_presence(vacuum_gripper),
                          "Vacuum gripper should be connected to object when is on.");

  // lift end effector
  wb_motor_set_position(motor, JOINT_STEP);
  wb_robot_step(50 * TIME_STEP);

  const double *p2 = wb_supervisor_node_get_position(object);
  ts_assert_vec3_in_delta(p2[0], p2[1], p2[2], p0[0], p0[1], p0[2] + JOINT_STEP, 0.002,
                          "Object should be lifted when vacumm cup is on.");

  wb_vacuum_gripper_turn_off(vacuum_gripper);

  wb_robot_step(TIME_STEP);
  ts_assert_boolean_equal(!wb_vacuum_gripper_is_on(vacuum_gripper), "Vacuum gripper should be off after turning it off.");
  ts_assert_boolean_equal(!wb_vacuum_gripper_get_presence(vacuum_gripper),
                          "Vacuum gripper should release the object when is turned off.");

  wb_robot_step(30 * TIME_STEP);

  const double *p3 = wb_supervisor_node_get_position(object);
  ts_assert_vec3_in_delta(p3[0], p3[1], p3[2], p0[0], p0[1], p0[2], 0.002,
                          "Object should be release when vacumm cup is turned off.");

  ts_send_success();
  return EXIT_SUCCESS;
}
