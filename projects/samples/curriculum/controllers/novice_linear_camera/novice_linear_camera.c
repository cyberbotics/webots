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

// Included libraries
#include <stdlib.h>
#include <webots/camera.h>
#include <webots/motor.h>  //obtain dif. wheels library
#include <webots/robot.h>  //obtain main library of webots

// Global defines
#define TIME_STEP 32  // [ms] // time step of the simulation
#define TIME_STEP_CAM 64
#define LEFT 0   // Left side
#define RIGHT 1  // right side
#define SPEED_UNIT 0.00628

WbDeviceTag cam;
unsigned short width, height;

// motors
WbDeviceTag left_motor, right_motor;

// This function returns the position
// of the peak contained in the array given
// in argument
int find_middle(const int tab[], int sizeTab) {
  int i, j;
  int *copy = (int *)malloc(sizeof(int) * sizeTab);
  int mean = 0;
  int nb_best = sizeTab / 10;
  int *index_bests = (int *)malloc(sizeof(int) * nb_best);

  // copy the tab, calculate the mean and
  // test if all the values are identical
  int identical = 1;
  for (i = 0; i < sizeTab; i++) {
    copy[i] = tab[i];
    mean += tab[i];
    if (tab[i] != tab[0])
      identical = 0;
  }
  if (identical) {
    free(copy);
    free(index_bests);
    return sizeTab / 2;
  }
  mean /= sizeTab;

  // take the best values of the tab
  for (i = 0; i < nb_best; i++) {
    int index = -1;
    int max = 0;
    for (j = 0; j < sizeTab; j++) {
      if (max < copy[j] && copy[j] > mean) {
        max = copy[j];
        index = j;
      }
    }
    index_bests[i] = index;
    if (index != -1)
      copy[index] = 0;
  }
  free(copy);
  // calculate the position mean of th best values
  int firstMean = 0;
  int count = 0;
  for (i = 0; i < nb_best; i++) {
    if (index_bests[i] != -1) {
      firstMean += index_bests[i];
      count++;
    }
  }
  if (count == 0) {
    free(index_bests);
    return sizeTab / 2;
  }
  firstMean /= count;

  // eliminate extrem values
  int secondMean = 0;
  count = 0;
  for (i = 0; i < nb_best; i++) {
    if (index_bests[i] < firstMean + sizeTab / 10 && index_bests[i] > firstMean - sizeTab / 10) {
      count++;
      secondMean += index_bests[i];
    }
  }
  free(index_bests);
  if (count == 0)
    return sizeTab / 2;

  return secondMean / count;
}

int main() {
  wb_robot_init();

  // get a handler to the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP_CAM);
  width = wb_camera_get_width(cam);
  height = wb_camera_get_height(cam);
  int *blue = (int *)malloc(sizeof(int) * width);

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    int i, delta = 0;
    int speed[2];
    const unsigned char *image;

    // 1. Get the sensors values
    // obstacle will contain a boolean information about a collision
    image = wb_camera_get_image(cam);

    for (i = 0; i < width; i++)
      blue[i] = 255 - wb_camera_image_get_blue(image, width, i, 0);

    // 2. Handle the sensor values
    delta = find_middle(blue, width) - width / 2;

    // adapt the motor speed in function of the intensity of
    // the changement of the direction
    speed[LEFT] = 200 - 4 * abs(delta);
    speed[RIGHT] = 200 - 4 * abs(delta);

    // 3. Send result to actuators
    // send speed values to motors
    wb_motor_set_velocity(left_motor, SPEED_UNIT * (speed[LEFT] + 4 * delta));
    wb_motor_set_velocity(right_motor, SPEED_UNIT * (speed[RIGHT] - 4 * delta));
  }
  free(blue);
  wb_robot_cleanup();

  return 0;
}
