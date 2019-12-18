// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/***************************************************************************

  Description : controller of the one-legged jumping robot
  Authors     : Yu Xianyuan
  Email       : xy_yu_beijing@163.com
  Operation_1 : Use the arrow( ↑ ↓ ← → ) of the keyboard to change the direction of the robot's jump
  Operation_2 : Use the keyboard's add and subtract keys( + - ) to change the robot's jump distance
  
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/keyboard.h>

#include "webotsInterface.h"
#include "controller.h"

int main(int argc, char **argv) {
  
  wb_robot_init();
  webots_device_init();                           //webots device init
  robot_init();                                   //robot init
  printf("Use the arrow( ↑ ↓ ← → ) of the keyboard to change the direction of the robot's jump.(HOLD ON)\n");
  printf("Use the keyboard's add and subtract keys( + - ) to change the robot's jump distance.(CLICK)\n");
  printf("have fun!\n");
  while (wb_robot_step(TIME_STEP) != -1) {

  updateRobotState();                             //sensor
  robot_control();                                //move
  }
  robot_free();                                   //free memory
  wb_robot_cleanup();
  return 0;
}


