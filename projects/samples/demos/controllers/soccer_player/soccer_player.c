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
 * Description:  A controller for players of the soccer game from soccer.wbt
 */

#include <stdlib.h>
#include <time.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#define ROBOTS 6
#define TIME_STEP 64

/*
 * The following macros are useful for retrieving the x,y coordinates and
 * angles value for each robot specified by a team id: 'y' or 'b' and a
 * player id: '1', '2', or '3'
 */
#define robot_get_id(t, p) (3 * ((p) - '1' + (((t) == 'y') ? ROBOTS / 2 : 0)))
#define robot_get_x(t, p) packet[robot_get_id(t, p)]
#define robot_get_y(t, p) packet[robot_get_id(t, p) + 1]
#define robot_get_orientation(t, p) packet[robot_get_id(t, p) + 2]
#define ball_get_x() packet[ROBOTS * 3]
#define ball_get_y() packet[ROBOTS * 3 + 1]

int main() {
  WbDeviceTag receiver; /* to receive coordinate information */
  WbDeviceTag left_motor, right_motor;
  char team;   /* can be either 'y' for yellow or 'b' for blue */
  char player; /* can be either '1', '2' or '3' */
  const char *name;
  int counter = 0, max = 50;
  double y, d;

  wb_robot_init();
  name = wb_robot_get_name();
  team = name[0];
  player = name[1];
  receiver = wb_robot_get_device("receiver");

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /*
   * Initialise the random number generator differently for the
   * different robots
   */
  srand(team + player);
  wb_receiver_enable(receiver, 64);

  while (wb_robot_step(TIME_STEP) != -1) {
    int left_speed = 0;
    int right_speed = 0;

    while (wb_receiver_get_queue_length(receiver) > 0) {
      const double *packet = wb_receiver_get_data(receiver);
      switch (player) {
        case '1': /* first player */
          /* falls through */
        case '2': /* second player */
          /*
           * This is a very stupid random behavior, up to you to improve it!
           * Advice: compare the ball coordinates to your own coordinates to
           * decide how to move the robot.
           */
          counter++;
          if (counter < max) {
            left_speed = 10;
            right_speed = 10;
          } else {
            left_speed = -10;
            right_speed = 10;
          }
          if (counter == 100) {
            counter = 0; /* reset the counter */

            /* Random number between 30 and 99 */
            max = 30 + rand() / (RAND_MAX / 70);
          }
          break;
        case '3': /* goal keeper */
          /*
           * Here, we make the goal keeper follow the y coordinate of the ball
           * while remaining inside the goal width with y inside [-0.16;+0.16]
           * This is very simple and doesn't guarantee the goal keeper will
           * remain in the goal for ever...
           */
          y = robot_get_y(team, player); /* my own y coordinate */
          d = y - ball_get_y();

          if (d > 0.01 && y > -0.16) {
            right_speed = 10;
            left_speed = 10;
          } else if (d < -0.01 && y < 0.16) {
            right_speed = -10;
            left_speed = -10;
          }
          break;
      }

      wb_receiver_next_packet(receiver);
    }
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
