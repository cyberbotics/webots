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

/*
 * Description:  The controller used to move the Surveyor with the keyboard.
 */

#include <stdio.h>
#include <webots/keyboard.h>
#include <webots/robot.h>
#include "surveyor_protocol.h"

#define TIME_STEP 16
#define MAX_BUFFER_SIZE 4
#define MAX_RESP_BUFFER_SIZE 90

static void print_keyboard_help();

int main() {
  unsigned char buffer[MAX_BUFFER_SIZE];
  static unsigned char response[MAX_RESP_BUFFER_SIZE];

  wb_robot_init();
  surveyor_init();
  wb_keyboard_enable(TIME_STEP);
  print_keyboard_help();

  while (wb_robot_step(TIME_STEP) != -1) {
    int changed = 1;
    buffer[0] = '4';

    /*
     * For more information on how to use the keyboard, please have a look at
     * the print_keyboard_help function.
     */
    switch (wb_keyboard_get_key()) {
      case 'Q':  // Q key -> up & left
        buffer[0] = '7';
        break;
      case 'W':  // W key -> up
        buffer[0] = '8';
        break;
      case 'E':  // E key -> up &right
        buffer[0] = '9';
        break;
      case 'A':  // A key -> spin left
        buffer[0] = '4';
        break;
      case 'S':  // S key -> stop
        buffer[0] = '5';
        break;
      case 'D':  // D key -> spin right
        buffer[0] = '6';
        break;
      case 'Y':  // Y key -> down & left
        buffer[0] = '1';
        break;
      case 'X':  // X key -> down
        buffer[0] = '2';
        break;
      case 'C':  // C key -> down & right
        buffer[0] = '3';
        break;
      case 'O':  // O key -> wander on
        buffer[0] = 'w';
        break;
      case 'P':  // P key -> wander off
        buffer[0] = 'W';
        break;
      case WB_KEYBOARD_UP:  // up key -> fast mode
        buffer[0] = '+';
        break;
      case WB_KEYBOARD_DOWN:  // down key -> low mode
        buffer[0] = '-';
        break;
      case WB_KEYBOARD_LEFT:  // left key -> on spot left rotation
        buffer[0] = '0';
        break;
      case WB_KEYBOARD_RIGHT:  // right key -> on spot right rotation
        buffer[0] = '.';
        break;
      case 'H':  // H key -> help
        print_keyboard_help();
        changed = 0;
        break;
      default:
        changed = 0;
        break;
    }

    surveyor_update(TIME_STEP);

    /*
     * We use the functions given by the protocol to actuate on the
     * Surveyor.
     */
    if (changed == 1)
      surveyor_send(buffer, response);
  }

  return 0;
}

static void print_keyboard_help() {
  printf("Select the 3D window and use the keyboard:\n");
  printf("\n");
  printf(" Q (drift left)\tW (forward)\tE (drift right)\n");
  printf(" A (turn left)\tS (stop)\tD (turn right)\n");
  printf(" Y (back left)\tX (backward)\tC (back right)\n");
  printf("\n");
  printf(" O (Wander on)\tP (Wander off)\n");
  printf("\n");
  printf(" Up (High speed)  \tLeft (Spin 20-deg left)\n");
  printf(" Down (Low speed)\tRight (Spin 20-deg right)\n");
  printf("\n");
  printf("In addition, you can also use the H command\n");
  printf("to print this help message\n");
  printf("-------------------------------------------------\n");
}
