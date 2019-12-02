/*
 * Description:  Test Supervisor device API
 *               This file contains tests of the reset physics function:
 *               wb_supervisor_node_reset_physics
 */

#include <string.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  double normalPreviousRotation[4], protoPreviousRotation[4], nestedPreviousRotation[4], hinge2PreviousRotation[4];

  WbNodeRef normalTestBot = wb_supervisor_node_get_from_def("TESTBOT");
  WbNodeRef protoTestBot = wb_supervisor_node_get_from_def("PROTOTESTBOT");
  WbNodeRef nestedTestBot = wb_supervisor_node_get_from_def("NESTEDTESTBOT");
  WbNodeRef hinge2TestBot = wb_supervisor_node_get_from_def("HINGE2TESTBOT");

  // traverse scene tree to get real node's endpoint rotation
  WbFieldRef normalChildren = wb_supervisor_node_get_field(normalTestBot, "children");
  WbNodeRef normalHinge = wb_supervisor_field_get_mf_node(normalChildren, 1);
  WbFieldRef hingeChildren = wb_supervisor_node_get_field(normalHinge, "endPoint");
  WbNodeRef hingeEndPoint = wb_supervisor_field_get_sf_node(hingeChildren);

  WbFieldRef hinge2Children = wb_supervisor_node_get_field(hinge2TestBot, "children");
  WbNodeRef hinge2Hinge = wb_supervisor_field_get_mf_node(hinge2Children, 1);
  WbFieldRef hinge2HingeChildren = wb_supervisor_node_get_field(hinge2Hinge, "endPoint");
  WbNodeRef hinge2EndPoint = wb_supervisor_field_get_sf_node(hinge2HingeChildren);

  WbFieldRef normalRotField = wb_supervisor_node_get_field(hingeEndPoint, "rotation");
  WbFieldRef protoRotField = wb_supervisor_node_get_field(protoTestBot, "hingeRotation");
  WbFieldRef nestedRotField = wb_supervisor_node_get_field(nestedTestBot, "hingeRotation");
  WbFieldRef hinge2RotField = wb_supervisor_node_get_field(hinge2EndPoint, "rotation");

  const double *normalRotation = wb_supervisor_field_get_sf_rotation(normalRotField);
  const double *protoRotation = wb_supervisor_field_get_sf_rotation(protoRotField);
  const double *nestedRotation = wb_supervisor_field_get_sf_rotation(nestedRotField);
  const double *hinge2Rotation = wb_supervisor_field_get_sf_rotation(hinge2RotField);

  memcpy(normalPreviousRotation, normalRotation, sizeof(double) * 4);
  memcpy(protoPreviousRotation, protoRotation, sizeof(double) * 4);
  memcpy(nestedPreviousRotation, nestedRotation, sizeof(double) * 4);
  memcpy(hinge2PreviousRotation, hinge2Rotation, sizeof(double) * 4);

  wb_robot_step(TIME_STEP * 2);

  normalRotation = wb_supervisor_field_get_sf_rotation(normalRotField);
  protoRotation = wb_supervisor_field_get_sf_rotation(protoRotField);
  nestedRotation = wb_supervisor_field_get_sf_rotation(nestedRotField);
  hinge2Rotation = wb_supervisor_field_get_sf_rotation(hinge2RotField);

  ts_assert_double_is_bigger(normalRotation[3], normalPreviousRotation[3],
                             "The base node's motor should rotate, but it is not rotating.");
  ts_assert_double_is_bigger(protoRotation[3], protoPreviousRotation[3],
                             "The PROTO's motor should rotate, but it is not rotating.");
  ts_assert_double_is_bigger(nestedRotation[3], nestedPreviousRotation[3],
                             "The nested PROTO's motor should rotate, but it is not rotating.");
  ts_assert_double_is_bigger(hinge2Rotation[3], hinge2PreviousRotation[3],
                             "The hinge2 motors should rotate, but they are not rotating.");

  wb_robot_step(TIME_STEP * 70);

  wb_supervisor_node_reset_physics(normalTestBot);
  wb_supervisor_node_reset_physics(protoTestBot);
  wb_supervisor_node_reset_physics(nestedTestBot);
  wb_supervisor_node_reset_physics(hinge2TestBot);

  normalRotation = wb_supervisor_field_get_sf_rotation(normalRotField);
  protoRotation = wb_supervisor_field_get_sf_rotation(protoRotField);
  nestedRotation = wb_supervisor_field_get_sf_rotation(nestedRotField);
  hinge2Rotation = wb_supervisor_field_get_sf_rotation(hinge2RotField);

  memcpy(normalPreviousRotation, normalRotation, sizeof(double) * 4);
  memcpy(protoPreviousRotation, protoRotation, sizeof(double) * 4);
  memcpy(nestedPreviousRotation, nestedRotation, sizeof(double) * 4);
  memcpy(hinge2PreviousRotation, hinge2Rotation, sizeof(double) * 4);

  wb_robot_step(TIME_STEP);

  normalRotation = wb_supervisor_field_get_sf_rotation(normalRotField);
  protoRotation = wb_supervisor_field_get_sf_rotation(protoRotField);
  nestedRotation = wb_supervisor_field_get_sf_rotation(nestedRotField);
  hinge2Rotation = wb_supervisor_field_get_sf_rotation(hinge2RotField);

  ts_assert_rotation_equals(normalRotation, normalPreviousRotation, 0.0001,
                            "The base node's motor should have stopped, but it is still turning.");
  ts_assert_rotation_equals(protoRotation, protoPreviousRotation, 0.0001,
                            "The PROTO's motor should have stopped, but it is still turning.");
  ts_assert_rotation_equals(nestedRotation, nestedPreviousRotation, 0.0001,
                            "The Nested PROTO's motor should have stopped, but it is still turning.");
  ts_assert_rotation_equals(hinge2Rotation, hinge2PreviousRotation, 0.0001,
                            "The hinge2 motors should have stopped, but they are still turning.");

  ts_send_success();
  return EXIT_SUCCESS;
}
