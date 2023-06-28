/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define BALL_COUNT 9

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  char name[8];
  const double lateral_velocity[6] = {0, 0, 0, -25, 0, 0};
  const double spinning_velocity[6] = {0, 0, 0, 0, 0, 25};

  WbNodeRef ball_nodes[BALL_COUNT];
  for (int i = 0; i < BALL_COUNT; ++i) {
    sprintf(name, "BALL_%d", i + 1);
    ball_nodes[i] = wb_supervisor_node_get_from_def(name);
  }

  // apply initial impulse to balls with rho2 and rhoN
  for (int i = 3; i < BALL_COUNT; ++i) {
    if (i < 6)
      wb_supervisor_node_set_velocity(ball_nodes[i], spinning_velocity);
    else
      wb_supervisor_node_set_velocity(ball_nodes[i], lateral_velocity);
  }

  // save initial positions
  double initial_position[BALL_COUNT][3];
  for (int i = 0; i < BALL_COUNT; ++i) {
    const double *position = wb_supervisor_node_get_position(ball_nodes[i]);
    memcpy(&initial_position[i], position, 3 * sizeof(double));
  }

  wb_robot_step(100 * TIME_STEP);

  const double *velocities[BALL_COUNT];
  for (int i = 0; i < BALL_COUNT; ++i)
    velocities[i] = wb_supervisor_node_get_velocity(ball_nodes[i]);

  // check that velocities of balls with just rho rolling friction are consistent
  ts_assert_double_is_bigger(velocities[0][0], velocities[1][0], "BALL_1 should have higher linear speed than BALL_2.");
  ts_assert_double_is_bigger(velocities[1][0], velocities[2][0], "BALL_2 should have higher linear speed than BALL_3.");

  ts_assert_double_is_bigger(velocities[0][4], velocities[1][4], "BALL_1 should have higher angular speed than BALL_2.");
  ts_assert_double_is_bigger(velocities[1][4], velocities[2][4], "BALL_2 should have higher angular speed than BALL_3.");

  for (int i = 0; i < 3; ++i) {
    sprintf(name, "BALL_%d", i + 1);
    ts_assert_double_in_delta(velocities[i][3], 0.0, 1e-6, "%s spinning in the wrong direction.", name);
    ts_assert_double_in_delta(velocities[i][5], 0.0, 1e-6, "%s spinning in the wrong direction.", name);
  }

  // check that velocities of balls with just rhoN rolling friction are consistent
  ts_assert_double_is_bigger(velocities[3][5], velocities[4][5], "BALL_4 should have higher angular speed than BALL_5.");
  ts_assert_double_is_bigger(velocities[4][5], velocities[5][5], "BALL_5 should have higher angular speed than BALL_6.");

  for (int i = 3; i < 6; ++i) {  // ensure they just spin around z axis
    sprintf(name, "BALL_%d", i + 1);
    const double expected_linear[3] = {0.0, 0.0, 0.0};
    ts_assert_doubles_in_delta(3, velocities[i], expected_linear, 1e-6, "%s should not have linear speed.", name);
    const double expected_angular[2] = {0.0, 0.0};  // should not be spinning around x and y axis
    ts_assert_doubles_in_delta(2, velocities[i] + 3, expected_angular, 1e-6, "%s should only spin around z.", name);
  }

  // check that positions of balls with rho2 rolling friction are consistent
  double delta_y[BALL_COUNT];  // lateral displacement, higher rho2 should limit this sideways motion
  for (int i = 0; i < BALL_COUNT; ++i) {
    const double *position = wb_supervisor_node_get_position(ball_nodes[i]);
    delta_y[i] = fabs(initial_position[i][1] - position[1]);
  }

  ts_assert_double_is_bigger(delta_y[6], delta_y[7], "BALL_7 should have bigger lateral displacement than BALL_8.");
  ts_assert_double_is_bigger(delta_y[7], delta_y[8], "BALL_8 should have bigger lateral displacement than BALL_9.");

  wb_robot_step(100 * TIME_STEP);

  // pre-registered velocities/positions after 200 timesteps (to ensure that if changes to the contact properties affect the
  // rolling behavior it will be detected)
  const double expected_positions[BALL_COUNT][3] = {
    {-5.822856, -3.0, 0.498430},       {-14.564553, 0.0, 0.498392},      {-25.145860, 3.0, 1.942235},
    {-16.0, -8.0, 1.498430},           {-14.0, -8.0, 1.498430},          {-12.0, -8.0, 1.498430},
    {-5.822856, -27.511330, 0.498430}, {1.171054, -19.324655, 0.498430}, {1.171054, -13.936790, 0.498430}};
  const double expected_velocities[BALL_COUNT][6] = {{5.957204, 0.0, 0.0, 0.0, 11.951927, 0.0},
                                                     {4.164077, 0.0, 0.000304, 0.0, 8.355176, 0.0},
                                                     {1.817242, 0.0, -0.368373, 0.0, 3.719816, 0.0},
                                                     {0.0, 0.0, 0.0, 0.0, 0.0, 25.0},
                                                     {0.0, 0.0, 0.0, 0.0, 0.0, 6.164800},
                                                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                     {5.957204, 3.686891, 0.0, -7.397003, 11.951927, -3.446233},
                                                     {7.029157, 0.0, 0.0, 0.0, 0.0, -4.455340},
                                                     {7.029157, 0.0, 0.0, 0.0, 0.0, -4.867729}};

  for (int i = 0; i < BALL_COUNT; ++i) {
    sprintf(name, "BALL_%d", i + 1);
    const double *position = wb_supervisor_node_get_position(ball_nodes[i]);
    const double *velocity = wb_supervisor_node_get_velocity(ball_nodes[i]);

    ts_assert_doubles_in_delta(3, position, expected_positions[i], 1e-6, "%s position different from expected one.", name);
    ts_assert_doubles_in_delta(6, velocity, expected_velocities[i], 1e-6, "%s velocity different from expected one.", name);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
