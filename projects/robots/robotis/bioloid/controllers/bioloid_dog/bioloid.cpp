// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/***************************************************************************

  bioloid -- controller of the bioloid quadruped robot
  Copyright (C) 2007 Biological Inspired Robotic Group (BIRG), EPFL
  Authors: Jean-Christophe Fillion-Robin
  Email: jean-christophe@fillion-robin.org

***************************************************************************/

#include "Robot.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <webots/keyboard.h>
#include <webots/robot.h>

static Robot *robot = NULL;

static const char *commands[] = {"Select the 3D window and use the keyboard:\n",
                                 "<- = Turn Left, -> = Turn Right, F = Walk Forward, B = Walk Backward\n",
                                 "Q = Increase frequency, W = Decrease frequency\n",
                                 "S = Increase stride length factor, A = Decrease stride length factor\n", NULL};

int main() {
  srand(time(NULL));
  wb_robot_init();
  const char *robotName = wb_robot_get_name();
  robot = new Robot(robotName);

  // enable keyboard
  wb_keyboard_enable(SIMULATION_STEP_DURATION);
  robot->initCamera();
  robot->standing();

  for (int c = 0; commands[c] != NULL; c++)
    printf("%s", commands[c]);

  robot->interactive_walk();

  delete robot;

  return 0;
}
