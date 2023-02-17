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

/*
 * Author:       Laurent Lessieux
 */

#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include "rcb3_simulator.h"

rcb3_simulator simulator;

#define NUMBER_OF_MOTION_FILES 24
const char *motion_files[NUMBER_OF_MOTION_FILES + 1] = {"2HV001RC_Walk_Forward.RCB",             // 0
                                                        "2HV002RC_Step_Back.RCB",                // 1
                                                        "2HV003RC_Walk_Left_Diag.RCB",           // 2
                                                        "2HV004RC_Walk_Right_Diag.RCB",          // 3
                                                        "2HV005RC_Side_Step_Left.RCB",           // 4
                                                        "2HV006RC_Side_Step_Right.RCB",          // 5
                                                        "2HV007RC_Turn_Left.RCB",                // 6
                                                        "2HV008RC_Turn_Right.RCB",               // 7
                                                        "2HV009AC_Standup_based_on_sensor.RCB",  // 8
                                                        "2HV010RC_StandUp_Face_Down.RCB",        // 9
                                                        "2HV011RC_Bow.RCB",                      // 10
                                                        "2HV012RC_Kick_Left.RCB",                // 11
                                                        "2HV013RC_Kick_Right.RCB",               // 12
                                                        "2HV014RC_Wheel_Front.RCB",              // 13
                                                        "2HV015RC_Wheel_Back.RCB",               // 14
                                                        "2HV016RC_Wheel_Left.RCB",               // 15
                                                        "2HV017RC_Wheel_Right.RCB",              // 16
                                                        "2HV018RC_PushUps.RCB",                  // 17
                                                        "2HV019RC_Punch_Right.RCB",              // 18
                                                        "2HV020RC_Punch_Left.RCB",               // 19
                                                        "2HV021RC_Free_all.RCB",                 // 20
                                                        "2HV022RC_Happy.RCB",                    // 21
                                                        "2HV023RC_Unhappy.RCB",                  // 22
                                                        "2HV024RC_Startup2.RCB",                 // 23
                                                        NULL};

int main(int argc, char *argv[]) {
  // init
  wb_robot_init();
  simulator.init();
  for (int i = 0; i < NUMBER_OF_MOTION_FILES; i++)
    simulator.load_motion(motion_files[i], i);
  simulator.load_scenario("demo_all.RCB", 0);

  if (argc < 2) {  // no argument
    simulator.play_motion(23);
    printf("Hello, my name is KHR-2HV!\n");
    simulator.play_motion(21);
    printf("I can walk\n");
    simulator.play_motion(0);
    printf("I can do some gym\n");
    simulator.play_motion(11);
    simulator.wait(100);
    simulator.play_motion(15);
    simulator.wait(100);
    simulator.play_motion(14);
    simulator.wait(100);
    simulator.play_motion(0);
  } else {  // play motions given as arguments
    for (int i = 1; i < argc; i++) {
      int iMotion = atoi(argv[i]);
      if (iMotion >= 0 && iMotion <= NUMBER_OF_MOTION_FILES) {
        printf("Play motion #%d: %s...\n", iMotion, motion_files[iMotion]);
        simulator.play_motion(iMotion);
        simulator.wait(100);
      } else
        printf("Motion file id #%d not found\n", iMotion);
    }
  }

  // Allow the user to control the robot
  printf("I am yours\n\n");
  simulator.play_motion(10);
  static const char *commands[] = {"Select the main window and use the keyboard:\n",
                                   "7 = Kick Left, 9 = Kick Right, 4 = Punch Left, 6 = Punch Right\n",
                                   "1 = Zannen (deception), 2 = Pushups, 3 = Happy, 8 = Bow\n",
                                   "R = Walk foward, V = Walk backwards\n",
                                   "C = Turn Left, B = Turn Right\n",
                                   "E = Walk Diag Left, T = Walk Diag Right, D = Side Step Left, G = Side Step Right\n",
                                   "Z = Stand Up (from face down or up), N = Stand up (face down)\n",
                                   "I = Forward Wheel, K = Backward Wheel\n",
                                   "U = Left Wheel, O = Right Wheel\n",
                                   NULL};
  for (int c = 0; commands[c] != NULL; c++)
    printf("%s", commands[c]);

  for (;;) {
    simulator.run();
    wb_robot_step(TIME_STEP);
  }

  return 0;
}
