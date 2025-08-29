#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

static const double velocities[6][6] = {{0, 0, 0, 0.5, 0, 1}, {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 1},
                                        {0, 0, 0, 1, 0, 1},   {1, 0, 1, 0, 0, 1}, {0, 1, 0, 0, 0, 1}};

static const double expected_position[7][3] = {{-0.4, 0.2, 0.8},    {0.64, 0, 0.5},     {0.5, 0.64, 0.5}, {-0.5, -2.11, -0.5},
                                               {0.64, -2.11, 0.14}, {0.5, -1.47, -0.5}, {0.64, 0.2, 0.5}};

static const double expected_rotation[7][4] = {{0.45, 0, 0.895, 0.716}, {0, 0, 1, 0},    {0, 0, 1, 0.64}, {0.7, 0, 0.7, 0.9},
                                               {0, 0, 1, 0.64},         {0, 0, 1, 0.64}, {0, 0, 1, 0}};

static const double expected_velocities[7][6] = {{0, 0, 0, 0.5, 0, 1},   {1, 0, 0, 0, 0, 0},     {0, 1, 0, 0, 0, 1},
                                                 {0, -6.28, 0, 1, 0, 1}, {1, -6.28, 1, 0, 0, 1}, {0, -5.28, 0, 0, 0, 1},
                                                 {1, 0, 0, 0, 0, 0}};

static const char *nodeNames[7] = {
  "KINEMATIC_ROTATE",  "KINEMATIC_TRANSLATE",      "KINEMATIC_TRANSLATE_ROTATE",  "DYNAMIC_ROTATE",
  "DYNAMIC_TRANSLATE", "DYNAMIC_TRANSLATE_ROTATE", "KINEMATIC_TRANSLATE_CHILDREN"};

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  int i, j;

  WbNodeRef node[7];
  WbFieldRef rotationField[7];

  for (i = 0; i < 7; ++i) {
    node[i] = wb_supervisor_node_get_from_def(nodeNames[i]);
    ts_assert_pointer_not_null(node[i], "The supervisor cannot get the WbNodeRef of %s.", nodeNames[i]);
    rotationField[i] = wb_supervisor_node_get_field(node[i], "rotation");
    ts_assert_pointer_not_null(rotationField[i], "The supervisor cannot get the rotationField field of %s.", nodeNames[i]);
    if (i < 6)  // don't apply speed on the children
      wb_supervisor_node_set_velocity(node[i], velocities[i]);
  }

  for (i = 0; i < 20; ++i)
    wb_robot_step(TIME_STEP);

  for (i = 0; i < 7; ++i) {
    // check expected position
    const double *position = wb_supervisor_node_get_position(node[i]);
    ts_assert_vec3_in_delta(
      position[0], position[1], position[2], expected_position[i][0], expected_position[i][1], expected_position[i][2], 0.02,
      "Solid %s is not at the expected position [%lf %lf %lf] instead of [%lf %lf %lf].", nodeNames[i], position[0],
      position[1], position[2], expected_position[i][0], expected_position[i][1], expected_position[i][2]);

    // check expected orientation
    const double *r = wb_supervisor_field_get_sf_rotation(rotationField[i]);
    double rotation[4];
    if (r[3] * expected_rotation[i][3] < 0) {  // if angles don't have the same sign
      for (j = 0; j < 4; j++)                  // we need to reverse the rotation angle and vector before comparison
        rotation[j] = -r[j];
    } else {
      for (j = 0; j < 4; j++)
        rotation[j] = r[j];
    }
    ts_assert_double_in_delta(rotation[3], expected_rotation[i][3], 0.02,
                              "Solid %s has not the expected orientation angle %lf instead of %lf.", nodeNames[i], rotation[3],
                              expected_rotation[i][3]);
    ts_assert_vec3_in_delta(
      rotation[0], rotation[1], rotation[2], expected_rotation[i][0], expected_rotation[i][1], expected_rotation[i][2], 0.01,
      "Solid %s has not the expected orientation axis [%lf %lf %lf] instead of [%lf %lf %lf].", nodeNames[i], rotation[0],
      rotation[1], rotation[2], expected_rotation[i][0], expected_rotation[i][1], expected_rotation[i][2]);

    // check expected velocities
    const double *velocity = wb_supervisor_node_get_velocity(node[i]);
    ts_assert_doubles_in_delta(
      6, velocity, expected_velocities[i], 0.02,
      "Solid %s has not the expected velocity [%lf %lf %lf %lf %lf %lf] instead of [%lf %lf %lf %lf %lf %lf].", nodeNames[i],
      velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5], expected_velocities[i][0],
      expected_velocities[i][1], expected_velocities[i][2], expected_velocities[i][3], expected_velocities[i][4],
      expected_velocities[i][5]);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
