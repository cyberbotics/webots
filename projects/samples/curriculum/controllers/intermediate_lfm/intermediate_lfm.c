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
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Global defines
#define THRESHOLD_DIST 50
#define TIME_STEP 32  // [ms] // time step of the simulation
#define TIME_STEP_CAM 32
#define SIMULATION 0  // for robot_get_mode() function
#define REALITY 2     // for robot_get_mode() function
#define LEFT 0        // Left side
#define RIGHT 1       // right side
#define SPEED_UNIT 0.00628

// leds
#define NB_LEDS 8
#define ON 1
#define OFF 0
WbDeviceTag led[NB_LEDS];

// motors
WbDeviceTag left_motor, right_motor;

// camera
WbDeviceTag cam;
unsigned short width, height;

// motors
WbDeviceTag left_motor;
WbDeviceTag right_motor;

/*****************************
 *
 *  Functions
 *
 ******************************/

// This function returns the position
// of the peak contained in the array given
// in argument
int find_middle(const int tab[], int sizeTab) {
  int i, j;
  int *copy = (int *)malloc(sizeof(int) * sizeTab);
  int mid = 0;
  int nb_best = sizeTab / 10;
  int *index_bests = (int *)malloc(sizeof(int) * nb_best);

  // copy the tab, calculate the mean and
  // test if all the values are identical
  int identical = 1;
  for (i = 0; i < sizeTab; i++) {
    copy[i] = tab[i];
    mid += tab[i];
    if (tab[i] != tab[0])
      identical = 0;
  }
  if (identical) {
    free(copy);
    free(index_bests);
    return sizeTab / 2;
  }
  mid /= sizeTab;

  // take the best values of the tab
  for (i = 0; i < nb_best; i++) {
    int index = -1;
    int max = 0;
    for (j = 0; j < sizeTab; j++) {
      if (max < copy[j] && copy[j] > mid) {
        max = copy[j];
        index = j;
      }
    }
    index_bests[i] = index;
    if (index >= 0 && index < sizeTab)
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

// return the mean of the values of an array
int mean(const int array[], int size) {
  if (size == 0)
    return 0;
  int sum = 0, i;
  for (i = 0; i < size; i++)
    sum += array[i];
  return sum / size;
}

/*****************************
 *
 *  Modules
 *
 ******************************/

// Line following module
#define MAX_DELTA 300.0f
int lfm_speed[2] = {0, 0};
int lfm_active = 1;
void lfm(int array[], int size) {
  if (lfm_active) {
    int delta = find_middle(array, size) - width / 2;
    lfm_speed[LEFT] = MAX_DELTA * delta / size;
    lfm_speed[RIGHT] = -lfm_speed[LEFT];
  } else
    lfm_speed[RIGHT] = lfm_speed[LEFT] = 0;
}

// line entering module
#define SENSIBILITY 10
int previous_mean[] = {0, 0, 0};
int current_mean[] = {0, 0, 0};
int is_in[] = {0, 0, 0};
void lem(const int array[], int size) {
  int *left = (int *)malloc(sizeof(int) * size / 10);
  int *right = (int *)malloc(sizeof(int) * size / 10);
  int *middle = (int *)malloc(sizeof(int) * size / 10);
  int i;

  for (i = 0; i < size / 10; i++) {
    left[i] = array[i];
    right[i] = array[size - 1 - i];
    middle[i] = array[size / 2 - size / 20 + i];
  }

  current_mean[0] = mean(left, size / 10);
  current_mean[1] = mean(middle, size / 10);
  current_mean[2] = mean(right, size / 10);

  for (i = 0; i < 3; i++) {
    if (current_mean[i] > previous_mean[i] + SENSIBILITY)
      is_in[i] = 1;
  }

  if (is_in[0] || is_in[1] || is_in[2]) {
    lfm_active = 1;

    // TODO : leds on
    //...
  }
  free(left);
  free(right);
  free(middle);
}
// line leaving module
void llm(int array[], int size) {
  int i;
  for (i = 0; i < 3; i++) {
    if (current_mean[i] < previous_mean[i] - SENSIBILITY)
      is_in[i] = 0;
    previous_mean[i] = current_mean[i];
  }

  if (!is_in[0] && !is_in[1] && !is_in[2]) {
    lfm_active = 0;

    // TODO : leds off
    //...
  }
}

// U Turn module
int utm_speed[] = {0, 0};
void utm(void) {
  // TODO : implement this module
  // - put your results in the speed_utm array
  // - look at the main function: add a function
  //   call to this function and add your results
  //   in the wb_motor_set_velocity(left_motor, ...) and
  //   wb_motor_set_velocity(right_motor, ...) functions

  //...
}

/*****************************
 *
 *  Standard functions
 *
 ******************************/

static void reset(void) {
  int i;
  char text[5] = "led0";
  for (i = 0; i < NB_LEDS; i++) {
    led[i] = wb_robot_get_device(text);
    text[3]++;
    wb_led_set(led[i], 0);
  }

  // get a handler to the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // enable the camera
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP_CAM);
  width = wb_camera_get_width(cam);
  height = wb_camera_get_height(cam);

  // motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

static int run(void) {
  int i;
  int *gray = (int *)malloc(sizeof(int) * width);
  const int speed[2] = {150, 150};
  const unsigned char *image;

  // 1. Get the sensors values
  image = wb_camera_get_image(cam);
  for (i = 0; i < width; i++)
    gray[i] = 255 - wb_camera_image_get_gray(image, width, i, 0);

  // 2. Behavior-based robotic:
  // call the modules in the right order
  lem(gray, width);
  lfm(gray, width);
  llm(gray, width);
  // utm();

  // 3. Send the values to actuators
  // send speed values to motors
  wb_motor_set_velocity(left_motor, SPEED_UNIT * (speed[LEFT] + lfm_speed[LEFT]));     //+utm_speed[LEFT],
  wb_motor_set_velocity(right_motor, SPEED_UNIT * (speed[RIGHT] + lfm_speed[RIGHT]));  //+utm_speed[RIGHT]
  free(gray);
  return TIME_STEP;
}

int main() {
  wb_robot_init();

  reset();

  while (wb_robot_step(TIME_STEP) != -1)
    run();

  wb_robot_cleanup();

  return 0;
}
