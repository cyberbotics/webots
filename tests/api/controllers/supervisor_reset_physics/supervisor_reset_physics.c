/*
 * Description:  Test Supervisor device API
 *               This file contains tests of reset physics methods:
 *                 wb_supervisor_node_reset_physics,
 *                 wb_supervisor_simulation_physics_reset
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  const double *currentTranslation;
  double previousPositionBall1 = 0.0;
  double previousPositionOthers = 0.0;
  double previousHeight = 0.0;

  ts_setup(argv[0]);

  WbNodeRef nodeBall1 = wb_supervisor_node_get_from_def("BALL1");
  WbNodeRef nodeBall2 = wb_supervisor_node_get_from_def("BALL2");
  WbNodeRef nodeBall3 = wb_supervisor_node_get_from_def("BALL3");

  ts_assert_pointer_not_null(nodeBall1, "The supervisor cannot get the WbNodeRef of BALL1.");
  ts_assert_pointer_not_null(nodeBall2, "The supervisor cannot get the WbNodeRef of BALL2.");
  ts_assert_pointer_not_null(nodeBall3, "The supervisor cannot get the WbNodeRef of BALL3.");

  WbFieldRef fieldTranslation1 = wb_supervisor_node_get_field(nodeBall1, "translation");
  WbFieldRef fieldTranslation2 = wb_supervisor_node_get_field(nodeBall2, "translation");
  WbFieldRef fieldTranslation3 = wb_supervisor_node_get_field(nodeBall3, "translation");
  ts_assert_pointer_not_null(fieldTranslation1, "The supervisor cannot get the WbFieldRef of translation of BALL1.");
  ts_assert_pointer_not_null(fieldTranslation2, "The supervisor cannot get the WbFieldRef of translation of BALL2.");
  ts_assert_pointer_not_null(fieldTranslation3, "The supervisor cannot get the WbFieldRef of translation of BALL3.");

  int stepCount = 0;
  while (wb_robot_step(TIME_STEP) != 1) {
    ++stepCount;
    currentTranslation = wb_supervisor_field_get_sf_vec3f(fieldTranslation1);

    if (fabs(previousHeight - currentTranslation[1]) < 1e-6) {
      previousPositionBall1 = currentTranslation[2];
      break;
    }

    previousHeight = currentTranslation[1];
  }

  ts_assert_boolean_equal(stepCount < 55, "Balls don't roll as expected.");

  // stop inertia of BALL1
  wb_supervisor_node_reset_physics(nodeBall1);

  wb_robot_step(32 * TIME_STEP);

  // check BALL1 is stopped
  currentTranslation = wb_supervisor_field_get_sf_vec3f(fieldTranslation1);
  ts_assert_double_in_delta(
    currentTranslation[2], previousPositionBall1, 0.002,
    "Physics of BALL1 has not been reset corectly: the ball position '%f' doesn't match the expected one '%f'.",
    currentTranslation[2], previousPositionBall1);

  // check BALL2 and BALL3 still move
  currentTranslation = wb_supervisor_field_get_sf_vec3f(fieldTranslation2);
  ts_assert_boolean_equal(fabs(currentTranslation[2] - previousPositionBall1) > 0.002,
                          "BALL2 stops moving: the ball position '%f' should be smaller than '%f'.", currentTranslation[2],
                          previousPositionBall1);

  previousPositionOthers = currentTranslation[2];

  currentTranslation = wb_supervisor_field_get_sf_vec3f(fieldTranslation3);
  ts_assert_double_in_delta(currentTranslation[2], previousPositionOthers, 1e-6,
                            "BALL3 stops moving: the position of BALL3 '%f' should match the position of BALL2 '%f'.",
                            currentTranslation[2], previousPositionOthers);

  // stop inertia of all solids
  wb_supervisor_simulation_reset_physics();

  wb_robot_step(2 * TIME_STEP);

  // check BALL1 is stopped
  currentTranslation = wb_supervisor_field_get_sf_vec3f(fieldTranslation1);
  ts_assert_double_in_delta(
    currentTranslation[2], previousPositionBall1, 0.002,
    "Physics of BALL1 has not been reset corectly: the ball position '%f' doesn match the expected one '%f'.",
    currentTranslation[2], previousPositionBall1);

  // check BALL1 is stopped
  currentTranslation = wb_supervisor_field_get_sf_vec3f(fieldTranslation2);
  ts_assert_double_in_delta(
    currentTranslation[2], previousPositionOthers, 0.002,
    "Physics of BALL2 has not been reset corectly: the ball position '%f' doesn match the expected one '%f'.",
    currentTranslation[2], previousPositionOthers);

  // check BALL3 is stopped
  currentTranslation = wb_supervisor_field_get_sf_vec3f(fieldTranslation3);
  ts_assert_double_in_delta(
    currentTranslation[2], previousPositionOthers, 0.002,
    "Physics of BALL3 has not been reset corectly: the ball position '%f' doesn match the expected one '%f'.",
    currentTranslation[2], previousPositionOthers);

  ts_send_success();
  return EXIT_SUCCESS;
}
