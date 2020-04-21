/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Simple controller to present the Spot robot.
 */

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NUMBER_OF_LEDS 8
#define NUMBER_OF_JOINTS 12
#define NUMBER_OF_CAMERAS 5

// Initialize the robot's information
static WbDeviceTag motors[NUMBER_OF_JOINTS];
static const char *motor_names[NUMBER_OF_JOINTS] = {
  "front left leg shoulder abduction motor",  "front left leg shoulder rotation motor",  "front left leg elbow motor",
  "front right leg shoulder abduction motor", "front right leg shoulder rotation motor", "front right leg elbow motor",
  "rear left leg shoulder abduction motor",   "rear left leg shoulder rotation motor",   "rear left leg elbow motor",
  "rear right leg shoulder abduction motor",  "rear right leg shoulder rotation motor",  "rear right leg elbow motor"};
static WbDeviceTag cameras[NUMBER_OF_CAMERAS];
static const char *camera_names[NUMBER_OF_CAMERAS] = {"left head camera", "right head camera", "left flank camera",
                                                      "right flank camera", "rear camera"};
static WbDeviceTag leds[NUMBER_OF_LEDS];
static const char *led_names[NUMBER_OF_LEDS] = {"left top led",          "left middle up led", "left middle down led",
                                                "left bottom led",       "right top led",      "right middle up led",
                                                "right middle down led", "right bottom led"};

void step() {
  const double time_step = wb_robot_get_basic_time_step();
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(0);
  }
}

// Movement decomposition
static void movement_decomposition(const double *target, double duration) {
  const double time_step = wb_robot_get_basic_time_step();
  const int n_steps_to_achieve_target = duration * 1000 / time_step;
  double step_difference[NUMBER_OF_JOINTS];
  double current_position[NUMBER_OF_JOINTS];

  for (int k = 0; k < NUMBER_OF_JOINTS; k++) {
    current_position[k] = wb_motor_get_target_position(motors[k]);
    step_difference[k] = (target[k] - current_position[k]) / n_steps_to_achieve_target;
  }

  for (int i = 0; i < n_steps_to_achieve_target; i++) {
    for (int k = 0; k < NUMBER_OF_JOINTS; k++) {
      current_position[k] += step_difference[k];
      wb_motor_set_position(motors[k], current_position[k]);
    }
    step();
  }
}

static void lie_down(double duration) {
  const double motors_target_pos[NUMBER_OF_JOINTS] = {-0.40, -0.99, 1.59,   // Front left leg
                                                      0.40,  -0.99, 1.59,   // Front right leg
                                                      -0.40, -0.99, 1.59,   // Rear left leg
                                                      0.40,  -0.99, 1.59};  // Rear right leg
  movement_decomposition(motors_target_pos, duration);
}

static void stand_up(double duration) {
  const double motors_target_pos[NUMBER_OF_JOINTS] = {-0.1, 0.0, 0.0,   // Front left leg
                                                      0.1,  0.0, 0.0,   // Front right leg
                                                      -0.1, 0.0, 0.0,   // Rear left leg
                                                      0.1,  0.0, 0.0};  // Rear right leg

  movement_decomposition(motors_target_pos, duration);
}

static void sit_down(double duration) {
  const double motors_target_pos[NUMBER_OF_JOINTS] = {-0.20, -0.40, -0.19,  // Front left leg
                                                      0.20,  -0.40, -0.19,  // Front right leg
                                                      -0.40, -0.90, 1.18,   // Rear left leg
                                                      0.40,  -0.90, 1.18};  // Rear right leg

  movement_decomposition(motors_target_pos, duration);
}

static void give_paw() {
  // Stabilize posture
  const double motors_target_pos_1[NUMBER_OF_JOINTS] = {-0.20, -0.30, 0.05,   // Front left leg
                                                        0.20,  -0.40, -0.19,  // Front right leg
                                                        -0.40, -0.90, 1.18,   // Rear left leg
                                                        0.49,  -0.90, 0.80};  // Rear right leg

  movement_decomposition(motors_target_pos_1, 4);

  const double initial_time = wb_robot_get_time();
  while (wb_robot_get_time() - initial_time < 8) {
    wb_motor_set_position(motors[4], 0.2 * sin(2 * wb_robot_get_time()) + 0.6);  // Upperarm movement
    wb_motor_set_position(motors[5], 0.4 * sin(2 * wb_robot_get_time()));        // Forearm movement
    step();
  }
  // Get back in sitting posture
  const double motors_target_pos_2[NUMBER_OF_JOINTS] = {-0.20, -0.40, -0.19,  // Front left leg
                                                        0.20,  -0.40, -0.19,  // Front right leg
                                                        -0.40, -0.90, 1.18,   // Rear left leg
                                                        0.40,  -0.90, 1.18};  // Rear right leg

  movement_decomposition(motors_target_pos_2, 4);
}

// static void go_forward() {
//   const double motors_target_pos_1[NUMBER_OF_JOINTS] = {0.00, -0.14, 0.41,   // Front left leg
//                                                         0.00, -0.71, 0.35,   // Front right leg
//                                                         0.00, -0.75, 0.33,   // Rear left leg
//                                                         0.00, -0.07, 0.43};  // Rear right leg
//
//   const double motors_target_pos_2[NUMBER_OF_JOINTS] = {0.00, -0.31, 0.39,   // Front left leg
//                                                         0.00, -0.85, 0.70,   // Front right leg
//                                                         0.00, -0.88, 0.72,   // Rear left leg
//                                                         0.00, -0.24, 0.41};  // Rear right leg
//
//   const double motors_target_pos_3[NUMBER_OF_JOINTS] = {0.00, -0.31, 0.36,   // Front left leg
//                                                         0.00, -0.61, 0.88,   // Front right leg
//                                                         0.00, -0.68, 0.72,   // Rear left leg
//                                                         0.00, -0.34, 0.42};  // Rear right leg
//
//   const double motors_target_pos_4[NUMBER_OF_JOINTS] = {0.00, -0.51, 0.31,   // Front left leg
//                                                         0.00, -0.17, 0.76,   // Front right leg
//                                                         0.00, -0.24, 0.72,   // Rear left leg
//                                                         0.00, -0.44, 0.37};  // Rear right leg
//
//   const double motors_target_pos_5[NUMBER_OF_JOINTS] = {0.00, -0.71, 0.50,   // Front left leg
//                                                         0.00, -0.14, 0.41,   // Front right leg
//                                                         0.00, -0.07, 0.43,   // Rear left leg
//                                                         0.00, -0.75, 0.33};  // Rear right leg
//
//   const double motors_target_pos_6[NUMBER_OF_JOINTS] = {0.00, -0.85, 0.70,   // Front left leg
//                                                         0.00, -0.31, 0.38,   // Front right leg
//                                                         0.00, -0.24, 0.41,   // Rear left leg
//                                                         0.00, -0.88, 0.72};  // Rear right leg
//
//   const double motors_target_pos_7[NUMBER_OF_JOINTS] = {0.00, -0.61, 0.88,   // Front left leg
//                                                         0.00, -0.31, 0.36,   // Front right leg
//                                                         0.00, -0.34, 0.42,   // Rear left leg
//                                                         0.00, -0.68, 0.72};  // Rear right leg
//
//   const double motors_target_pos_8[NUMBER_OF_JOINTS] = {0.00, -0.17, 0.76,   // Front left leg
//                                                         0.00, -0.51, 0.31,   // Front right leg
//                                                         0.00, -0.44, 0.37,   // Rear left leg
//                                                         0.00, -0.24, 0.72};  // Rear right leg
//
//   const double motors_target_pos_9[NUMBER_OF_JOINTS] = {0.00, -0.14, 0.41,   // Front left leg
//                                                         0.00, -0.71, 0.35,   // Front right leg
//                                                         0.00, -0.75, 0.33,   // Rear left leg
//                                                         0.00, -0.07, 0.50};  // Rear right leg
//
//   const double motors_target_pos_10[NUMBER_OF_JOINTS] = {0.00, -0.31, 0.30,   // Front left leg
//                                                          0.00, -0.85, 0.70,   // Front right leg
//                                                          0.00, -0.88, 0.72,   // Rear left leg
//                                                          0.00, -0.24, 0.54};  // Rear right leg
//
//   const double motors_target_pos_11[NUMBER_OF_JOINTS] = {0.00, -0.31, 0.30,   // Front left leg
//                                                          0.00, -0.61, 0.88,   // Front right leg
//                                                          0.00, -0.68, 0.62,   // Rear left leg
//                                                          0.00, -0.38, 0.52};  // Rear right leg
//
//   const double motors_target_pos_12[NUMBER_OF_JOINTS] = {0.00, -0.51, 0.20,   // Front left leg
//                                                          0.00, -0.17, 0.76,   // Front right leg
//                                                          0.00, -0.24, 0.72,   // Rear left leg
//                                                          0.00, -0.44, 0.52};  // Rear right leg
//
//   const double motors_target_pos_13[NUMBER_OF_JOINTS] = {0.00, -0.71, 0.50,   // Front left leg
//                                                          0.00, -0.14, 0.41,   // Front right leg
//                                                          0.00, -0.07, 0.43,   // Rear left leg
//                                                          0.00, -0.75, 0.64};  // Rear right leg
//
//   const double motors_target_pos_14[NUMBER_OF_JOINTS] = {0.00, -0.85, 0.70,   // Front left leg
//                                                          0.00, -0.31, 0.38,   // Front right leg
//                                                          0.00, -0.24, 0.41,   // Rear left leg
//                                                          0.00, -0.88, 0.72};  // Rear right leg
//
//   const double motors_target_pos_15[NUMBER_OF_JOINTS] = {0.00, -0.61, 0.88,   // Front left leg
//                                                          0.00, -0.31, 0.36,   // Front right leg
//                                                          0.00, -0.34, 0.42,   // Rear left leg
//                                                          0.00, -0.68, 0.72};  // Rear right leg
//
//   const double motors_target_pos_16[NUMBER_OF_JOINTS] = {0.00, -0.17, 0.76,   // Front left leg
//                                                          0.00, -0.51, 0.31,   // Front right leg
//                                                          0.00, -0.44, 0.37,   // Rear left leg
//                                                          0.00, -0.24, 0.72};  // Rear right leg
//   movement_decomposition(motors_target_pos_1, 1);
//   printf("Move 1\n");
//   movement_decomposition(motors_target_pos_2, 1);
//   printf("Move 2\n");
//   movement_decomposition(motors_target_pos_3, 1);
//   printf("Move 3\n");
//   movement_decomposition(motors_target_pos_4, 1);
//   printf("Move 4\n");
//   movement_decomposition(motors_target_pos_5, 1);
//   printf("Move 5\n");
//   movement_decomposition(motors_target_pos_6, 1);
//   printf("Move 6\n");
//   movement_decomposition(motors_target_pos_7, 1);
//   printf("Move 7\n");
//   movement_decomposition(motors_target_pos_8, 1);
//   printf("Move 8\n");
//   movement_decomposition(motors_target_pos_9, 1);
//   printf("Move 9\n");
//   movement_decomposition(motors_target_pos_10, 1);
//   printf("Move 10\n");
//   movement_decomposition(motors_target_pos_11, 1);
//   printf("Move 11\n");
//   movement_decomposition(motors_target_pos_12, 1);
//   printf("Move 12\n");
//   movement_decomposition(motors_target_pos_13, 1);
//   printf("Move 13\n");
//   movement_decomposition(motors_target_pos_14, 1);
//   printf("Move 14\n");
//   movement_decomposition(motors_target_pos_15, 1);
//   printf("Move 15\n");
//   movement_decomposition(motors_target_pos_16, 1);
//   printf("Move 16\n");
// }

int main(int argc, char **argv) {
  wb_robot_init();

  // Get cameras
  for (int i = 0; i < NUMBER_OF_CAMERAS; i++)
    cameras[i] = wb_robot_get_device(camera_names[i]);

  // Get the LEDs and turn them on
  for (int i = 0; i < NUMBER_OF_LEDS; i++) {
    leds[i] = wb_robot_get_device(led_names[i]);
    wb_led_set(leds[i], 1);
  }

  // Get the motors (joints) and set initial target position to 0
  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    motors[i] = wb_robot_get_device(motor_names[i]);

  while (true) {
    lie_down(4.0);
    stand_up(4.0);
    sit_down(4.0);
    give_paw();
    stand_up(4.0);
    lie_down(3.0);
    stand_up(3.0);
    lie_down(2.0);
    stand_up(2.0);
    lie_down(1.0);
    stand_up(1.0);
  }

  wb_robot_cleanup();
  return EXIT_FAILURE;
}
