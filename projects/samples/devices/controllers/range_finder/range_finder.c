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
 * Description:  An example of use of a range-finder device.
 */

#include <stdlib.h>
#include <string.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>
#include <webots/utils/system.h>

#define SPEED 4
#define TIME_STEP 64

typedef enum { SEARCHING, MOVING } states;

int main() {
  WbDeviceTag range_finder, left_motor, right_motor;
  int range_finder_width, range_finder_height;
  states state = SEARCHING;
  int left_speed, right_speed;
  int i, j, centering_weight;
  float distance;
  bool save_image = true;

  wb_robot_init();
  /*
   * First we get a handler to the range-finder device and then we open and place
   * it. We also store its height, width, near and far parameters for
   * further use.
   */
  range_finder = wb_robot_get_device("range-finder");
  wb_range_finder_enable(range_finder, TIME_STEP);
  range_finder_width = wb_range_finder_get_width(range_finder);
  range_finder_height = wb_range_finder_get_height(range_finder);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    /* This is used to refresh the range-finder. */
    const float *image = wb_range_finder_get_range_image(range_finder);

    float average_distance = 0;
    float minimal_distance = 100;

    /*
     * Here we analyse the image we've got from the range-finder. Our robot has two
     * states: SEARCHING and MOVING. When it is in SEARCHING, it looks around
     * for a direction in which it has enough distance to move. So in this
     * case we compute the weighted average distance of the range-finder. In the
     * MOVING state, it is moving straight until it comes too close from an
     * obsacle. In this case we compute the minimal distance of the range-finder.
     */
    for (i = 0; i < range_finder_width; i++) {
      /*
       * The weight is very simple, it is only an interval in the center of
       * the image to be shure to have enough place to move our robot.
       */
      centering_weight = ((i < (range_finder_width / 4)) || (i > (3 * range_finder_width / 4))) ? 1 : 3;
      for (j = 0; j < range_finder_height; j++) {
        distance = wb_range_finder_image_get_depth(image, range_finder_width, i, j);
        if (state == SEARCHING)
          average_distance += distance * centering_weight;
        else if (distance < minimal_distance)
          minimal_distance = distance;
      }
    }

    average_distance /= range_finder_width * range_finder_height;

    if (average_distance >= 1 && state == SEARCHING)
      state = MOVING;
    else if (minimal_distance < 0.1 && state == MOVING)
      state = SEARCHING;

    if (state == MOVING) {
      left_speed = SPEED;
      right_speed = SPEED;
    } else {
      left_speed = -SPEED;
      right_speed = SPEED;
    }

    if (save_image) {
      /* Save range-finder's current view as HDR image in home directory*/
      char *filepath;
#ifdef _WIN32
      const char *user_directory = wbu_system_short_path(wbu_system_getenv("USERPROFILE"));
      filepath = (char *)malloc(strlen(user_directory) + 16);
      strcpy(filepath, user_directory);
      strcat(filepath, "\\");
#else
      const char *user_directory = wbu_system_getenv("HOME");
      filepath = (char *)malloc(strlen(user_directory) + 16);
      strcpy(filepath, user_directory);
      strcat(filepath, "/");
#endif
      strcat(filepath, "test.hdr");
      wb_range_finder_save_image(range_finder, filepath, 100);
      free(filepath);
      save_image = false;
    }

    /* Set the motor speeds. */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
