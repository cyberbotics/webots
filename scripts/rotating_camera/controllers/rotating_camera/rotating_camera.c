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
#include <math.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define CW  0
#define CCW 1
#define UP 0
#define DOWN 1

static WbNodeRef root, nodes;
static WbNodeRef viewpoint;
static WbNodeRef robot;

WbFieldRef children;
WbFieldRef viewpoint_position;
WbFieldRef viewpoint_orientation;
WbFieldRef robot_translation;

// The viewpoint revolves around the robot following a circle trajectory
//  - robot_position: position of the robot.
//  - rotation_direction: CW (clockwise) or CCW (counterclockwise).
//  - vertical_direction: UP or DOWN.
//  - n_turn: number of complete revolution.
//  - radius: distance between the robot and the camera.
//  - height_step: height increase achieved during a complete revolution.
//  - angle_end: make an arc of a circle, set it to (2 * M_PI) for a complete revolution.

static void revolving_view(const double *robot_position, int rotation_direction, int vertical_direction, int n_turn, float radius, float height_step, double angle_end) {
  int k = 0;
  double angle = 0.0;
  double angle_step = 0.01;
  double new_position[3] = {0.0, 0.0, 0.0};
  double orientation[4] = {0, 0, 1, 0}; // Front view

  // Get the Viewpoint node and get the "position" and "orientation" fields
  viewpoint = wb_supervisor_field_get_mf_node(children, 1);
  viewpoint_position = wb_supervisor_node_get_field(viewpoint, "position");
  viewpoint_orientation = wb_supervisor_node_get_field(viewpoint, "orientation");

  while (k < n_turn) { // Start at angle 0 and end at angle n_turn * 2 * PI.

    if (fabs(angle_end - (2 * M_PI)) > 1e-6) {
      if (angle >= angle_end)
        break;
    }
    if (angle >= (2 * M_PI)) {  // Made a revolution
      angle = 0.0;
      k += 1;
    }

    // Set new position to ViewPoint node
    new_position[1] = robot_position[1] + radius * sin(angle);
    if (vertical_direction == UP)
      new_position[2] += height_step / (2 * M_PI / angle_step);
    else
      new_position[2] -= height_step / (2 * M_PI / angle_step);
    new_position[0] = robot_position[0] + radius * cos(angle);
    wb_supervisor_field_set_sf_vec3f(viewpoint_position, new_position);

    // Set new orientation to ViewPoint node
    orientation[3] = angle + 3.14159265359;
    wb_supervisor_field_set_sf_rotation(viewpoint_orientation, orientation);

    // Update the angle value
    if (rotation_direction == CW)
      angle += angle_step;
    else // counterclockwise
      angle -= angle_step;

    wb_robot_step(wb_robot_get_basic_time_step());  // Wait for 32 milliseconds
  }
}

int main(int argc, char *argv[]) {

  const double *robot_position;

  wb_robot_init();

  root = wb_supervisor_node_get_root();
  children = wb_supervisor_node_get_field(root, "children");

  int n = wb_supervisor_field_get_count(children);
  printf("This world contains %d nodes:\n", n);
  for (int i = 0; i < n; i++) {
    nodes = wb_supervisor_field_get_mf_node(children, i);
    printf("-> %s\n", wb_supervisor_node_get_type_name(nodes));
  }
  printf("\n");

  robot = wb_supervisor_field_get_mf_node(children, 5);
  robot_translation = wb_supervisor_node_get_field(robot, "translation");
  robot_position = wb_supervisor_field_get_sf_vec3f(robot_translation);

  printf("Start in 1 second (simulation time)...\n");
  wb_robot_step(1024);  // Wait for about 1 second

  const int n_turn = 3;
  const float radius = 0.7;
  const float height = 0.2;
  const float angle_end = 2 * M_PI;

  printf("\n");
  printf("Robot.position = %g %g %g\n", robot_position[0], robot_position[1], robot_position[2]);
  printf("Number of revolutions: %d\n", n_turn);
  printf("Radius: %f\n", radius);
  printf("Height step per revolution: %f\n", height);
  printf("Angle stop at %f rad\n", angle_end);
  printf("\n");

  revolving_view(robot_position, CW, UP, n_turn, radius, height, angle_end);

  wb_robot_cleanup();
  return 0;
}
