/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
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

#define TOL 1e-6
#define ANGLE_REV 6.28
#define CW  0
#define CCW 1
#define UP 0
#define DOWN 1

// Global variables
WbNodeRef root, nodes;
WbNodeRef nodeViewPoint;
WbNodeRef nodeRobot;

WbFieldRef children;
WbFieldRef fieldViewPointPos;
WbFieldRef fieldViewPointOrient;
WbFieldRef fieldRobotTranslation;

// The viewpoint revolves around the robot following a circle trajectory
//  o robotPos : Specifies the robot position.
//  o direction : CW (Turn ClockWise, default) or CCW (turn CounterClockWise).
//  o nTurn : Make the number of complet revolution defined.
//  o radius : Set the distance between the robot and the camera.
//  o heightStep : Height achieved during a complet revolution.
//  o angleEnd : Make an arc of a circle. Set to 6.28 for complet revolution.

void turningView(double *robotPos, int rotationDirection, int verticalDirection, int nTurn, float radius, float heightStep, double angleEnd) {

  int k  = 0;
  double angle = 0.0;
  double angleStep = 0.01;
  double newPosition[3] = {0.0, 0.0, 0.0};
  double orientation[4] = {0, 1, 0, 0}; // Front view

  // Get Viewpoint node and get the fields "position" and "orientation"
  nodeViewPoint  = wb_supervisor_field_get_mf_node(children, 1); // Search ViewPoint node
  fieldViewPointPos = wb_supervisor_node_get_field(nodeViewPoint, "position");
  fieldViewPointOrient = wb_supervisor_node_get_field(nodeViewPoint, "orientation");

  while (k < nTurn){ // Start at angle 0 and end at angle nTurn*2*PI.

    if (fabs(angleEnd - ANGLE_REV)>TOL) {
      if (angle >= angleEnd) {
        break;
      }
    }
    if (angle >= ANGLE_REV) { // had make a revolution
      angle = 0.0;
      k += 1;
    }

    // Set new position to ViewPoint node
    newPosition[0]  = robotPos[0] + radius * sin(angle);
    if (verticalDirection == UP)
      newPosition[1] += heightStep/(6.28/angleStep);
    else
      newPosition[1] -= heightStep/(6.28/angleStep);
    newPosition[2]  = robotPos[2] + radius * cos(angle);
    wb_supervisor_field_set_sf_vec3f(fieldViewPointPos, newPosition);

    // Set new orientation to ViewPoint node
    orientation[3] = angle;
    wb_supervisor_field_set_sf_rotation(fieldViewPointOrient, orientation);

    // Update the angle value
    if (rotationDirection == CW)
      angle += angleStep;
    else // CounterClockWise
      angle -= angleStep;

    wb_robot_step(wb_robot_get_basic_time_step()); // wait for 32 milliseconds
  }
}



int main(int argc, char *argv[]) {

  const double *temp;
  double robotPos[3] = {0.0, 0.0, 0.0};

  // Init and get the scene tree (=root)
  wb_robot_init();

  root     = wb_supervisor_node_get_root();
  children = wb_supervisor_node_get_field(root, "children");

  // Count and display all nodes in the world
  int n = wb_supervisor_field_get_count(children);
  printf("This world contains %d nodes:\n", n);
  for (int i = 0; i < n; i++) {
    nodes = wb_supervisor_field_get_mf_node(children, i);
    printf("-> %s\n", wb_supervisor_node_get_type_name(nodes));
  }
  printf("\n");

  // In Robot node, get the field "translation"
  nodeRobot = wb_supervisor_field_get_mf_node(children, 5); // Search Robot node
  fieldRobotTranslation = wb_supervisor_node_get_field(nodeRobot, "translation");
  temp = wb_supervisor_field_get_sf_vec3f(fieldRobotTranslation);
  robotPos[0] = temp[0];
  robotPos[1] = temp[1];
  robotPos[2] = temp[2];

  printf("Start in 1 second (simulation time)...\n");
  wb_robot_step(1000); /* wait for 2 seconds */

  int N = 3;
  float R = 0.7;
  float height = 0.2;
  float angleEnd = 6.28;

  // Print information
  printf("\n");
  printf("Robot.position = %g %g %g\n", robotPos[0], robotPos[1], robotPos[2]);
  printf("Number of turn: %d\n", N);
  printf("Radius: %f\n", R);
  printf("Height step per revolution: %f\n", height);
  printf("Angle stop at %f rad\n", angleEnd);
  printf("\n");

  turningView(robotPos, CW, UP, N, R, height, angleEnd);

  wb_robot_cleanup();
  return 0;
}
