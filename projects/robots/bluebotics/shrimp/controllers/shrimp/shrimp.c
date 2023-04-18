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
 * Description:  The controller used to move the shrimp with the keyboard.
 */

#include <stdio.h>
#include <string.h>
#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include "shrimp_protocol.h"

#define TIME_STEP 64
#define MAX_BUFFER_SIZE 4
#define MAX_STRING_SIZE 1024
#define WHEELS_NUMBER 6

#define MAX_VELOCITY 127
#define MAX_DIRECTION 90

static int velocity, velocity_increment;
static int direction, direction_increment;

static void print_keyboard_help() {
  printf("Select the 3D window and use the keyboard:\n");
  printf(" Q W E: |    control the direction and speed\n");
  printf(" A S D: |\n");
  printf(" Y X C: |\n");
  printf(" M:          stop\n");
  printf(" Space:      stop and reset the direction\n");
  printf(" O:          prints the values of the wheel encoders\n");
  printf(" P:          prints the velocity and the steering angle\n");
  printf(" H:          prints this help message\n");
  printf("-------------------------------------------------\n");
}

/*
 * This function is simply used to display clearly the answers of the Shrimp
 * to our commands.
 */
static void print_answer(unsigned char *answer) {
  int i;
  char answer_string[MAX_STRING_SIZE];

  switch (answer[0]) {
    case 0x01:
      sprintf(answer_string, "0x%x %d.%d.%d", answer[0], answer[1], answer[2], answer[3]);
      break;
    case 0x05:
      sprintf(answer_string, "0x%x %d %d", answer[0], answer[1], answer[2]);
      break;
    case 0x07:
      sprintf(answer_string, "0x%x", answer[0]);

      for (i = 0; i < WHEELS_NUMBER; i++) {
        int temp = answer[(i * 4) + 1] << 24;
        temp += answer[(i * 4) + 2] << 16;
        temp += answer[(i * 4) + 3] << 8;
        temp += answer[(i * 4) + 4];
        char temp_string[MAX_STRING_SIZE];
        sprintf(temp_string, " %d", temp);
        strcat(answer_string, temp_string);
      }

      break;
    case 0x08:
      sprintf(answer_string, "0x%x %d", answer[0], answer[1]);
      break;
    default:
      sprintf(answer_string, "0x%x", answer[0]);
  }

  printf("%s\n", answer_string);
}

static void run_step() {
  unsigned char buffer[MAX_BUFFER_SIZE];
  int changed = 1;

  /*
   * For more information on how to use the keyborad, please have a look at
   * the print_keyboard_help function.
   */
  switch (wb_keyboard_get_key()) {
    case 'Q':  // Q key -> up & left
      velocity += velocity_increment;
      direction -= direction_increment;
      break;
    case 'W':  // W key -> up
      velocity += velocity_increment;
      break;
    case 'E':  // E key -> up &right
      velocity += velocity_increment;
      direction += direction_increment;
      break;
    case 'A':  // A key -> left
      direction -= direction_increment;
      break;
    case 'S':  // S key -> straight
      direction = 0;
      break;
    case 'D':  // D key -> right
      direction += direction_increment;
      break;
    case 'Y':  // Y key -> down & left
      velocity -= velocity_increment;
      direction -= direction_increment;
      break;
    case 'X':  // X key -> down
      velocity -= velocity_increment;
      break;
    case 'C':  // C key -> down & right
      velocity -= velocity_increment;
      direction += direction_increment;
      break;
    case 'M':  // M key -> stop
      velocity = 0;
      break;
    case ' ':  // space -> reset
      direction = 0;
      velocity = 0;
      break;
    case 'O':  // O key -> odometry
      changed = 2;
      break;
    case 'P':  // P key -> speed & direction
      changed = 3;
      break;
    case 'H':  // H key -> help
      print_keyboard_help();
      changed = 0;
      break;
    default:
      changed = 0;
      break;
  }

  /*
   * We use the functions given by the protocol to actuate on the
   * Shrimp.
   */
  if (changed == 1) {
    if (velocity > MAX_VELOCITY)
      velocity = MAX_VELOCITY;
    else if (velocity < -MAX_VELOCITY)
      velocity = -MAX_VELOCITY;

    if (direction > MAX_DIRECTION)
      direction = MAX_DIRECTION;
    else if (direction < -MAX_DIRECTION)
      direction = -MAX_DIRECTION;

    buffer[0] = 0x04;
    buffer[1] = (unsigned char)velocity;
    buffer[2] = (unsigned char)direction;

    shrimp_send(buffer);
  } else if (changed == 2) {
    buffer[0] = 0x07;
    print_answer(shrimp_send(buffer));
  } else if (changed == 3) {
    buffer[0] = 0x05;
    print_answer(shrimp_send(buffer));
  }
}

int main() {
  unsigned char buffer[MAX_BUFFER_SIZE];
  unsigned char *answer;
  int position, in_position;

  wb_robot_init();
  shrimp_init();

  velocity = 0;
  direction = 0;

  /* These values are the ones defined in the Shrimp Documentation. */
  velocity_increment = 10;
  direction_increment = 5;

  /*
   * This is a small example of the possible movements of the Shrimp. We
   * drive the Shrimp through the balls, we make it turn and then we let the
   * user move it using the keyboard.
   */
  buffer[0] = 0x04;
  buffer[1] = (unsigned char)95;
  buffer[2] = (unsigned char)0;

  shrimp_send(buffer);

  /* We need to use this function to allow Webots to make a step. */
  wb_robot_step(TIME_STEP);

  buffer[0] = 0x07;

  in_position = 0;
  do {
    answer = shrimp_send(buffer);

    position = answer[1] << 24;
    position += answer[2] << 16;
    position += answer[3] << 8;
    position += answer[4];

    if (position >= 200)
      in_position = 1;

    wb_robot_step(TIME_STEP);
  } while (in_position != 1);

  buffer[0] = 0x04;
  buffer[1] = (unsigned char)127;
  buffer[2] = (unsigned char)-75;

  shrimp_send(buffer);

  wb_robot_step(TIME_STEP);

  buffer[0] = 0x07;

  in_position = 0;
  do {
    answer = shrimp_send(buffer);

    position = answer[1] << 24;
    position += answer[2] << 16;
    position += answer[3] << 8;
    position += answer[4];

    if (position >= 270)
      in_position = 1;

    wb_robot_step(TIME_STEP);
  } while (in_position != 1);

  buffer[0] = 0x04;
  buffer[1] = (unsigned char)0;
  buffer[2] = (unsigned char)0;

  wb_keyboard_enable(TIME_STEP);
  print_keyboard_help();

  shrimp_send(buffer);

  /*
   * After this demo we go to the run() function where the user will be
   * able to move the Shrimp by hand.
   */

  while (wb_robot_step(TIME_STEP) != -1)
    run_step();

  wb_robot_cleanup();

  return 0;
}
