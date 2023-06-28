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
#include <math.h>
#include <stdlib.h>
#include <webots/camera.h>
#include <webots/motor.h>  //obtain dif. wheels library
#include <webots/robot.h>  //obtain main library of webots

// Global defines
#define TIME_STEP 64  // [ms] // time step of the simulation
#define TIME_STEP_CAM 64
#define LEFT 0   // Left side
#define RIGHT 1  // right side
#define SPEED_UNIT 0.00628

#define THRESHOLD 200
#define MAX_SPEED 600
#define BACKWARD_SPEED 200
#define K_TURN 4

// Reality
/*
#define THRESHOLD 160
#define MAX_SPEED 200
#define BACKWARD_SPEED 100
#define K_TURN 2
*/

WbDeviceTag cam;
unsigned short width, height;

// motors
WbDeviceTag left_motor, right_motor;

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
  float *intensity = (float *)malloc(sizeof(float) * width);

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    int i, j, delta, max, index_max = 0, speed[2];
    const unsigned char *image;

    // 1. Get the sensors values
    // obstacle will contain a boolean information about a collision
    image = wb_camera_get_image(cam);

    // 2. Handle the sensor values
    for (i = 0; i < width; i++) {
      int count = 0;
      for (j = 0; j < height; j++)
        count += wb_camera_image_get_gray(image, width, i, j);
      intensity[i] = count;
    }

    delta = 0;
    max = 0;
    for (i = 0; i < width; i++) {
      if (max < intensity[i]) {
        max = intensity[i];
        index_max = i;
        delta = i - (width / 2);
      }
    }
    int iter = 0;
    if (index_max >= 0 && index_max < height) {
      for (j = 0; j < height; j++) {
        if (THRESHOLD < wb_camera_image_get_gray(image, width, index_max, j))
          iter++;
      }
    } else
      iter = (MAX_SPEED * height) / (MAX_SPEED + BACKWARD_SPEED);

    // adapt the motor speed in function of the intensity of
    // the changement of the direction
    speed[LEFT] = MAX_SPEED - (MAX_SPEED + BACKWARD_SPEED) * iter / height;
    speed[RIGHT] = MAX_SPEED - (MAX_SPEED + BACKWARD_SPEED) * iter / height;

    // 3. Send result to actuators
    wb_motor_set_velocity(left_motor, SPEED_UNIT * (speed[LEFT] + K_TURN * delta));
    wb_motor_set_velocity(right_motor, SPEED_UNIT * (speed[RIGHT] - K_TURN * delta));
  }
  free(intensity);
  wb_robot_cleanup();

  return 0;
}
