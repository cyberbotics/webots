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
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define LEDS_ON 1
#define LEDS_OFF 0
#define DURATION 4  // Time in second to perform action
#define GIVE_PAW true
#define TIME_STEP (int)wb_robot_get_basic_time_step()  // From world file
#define NUMBER_OF_LEDS 2
#define NUMBER_OF_JOINTS 12
#define NUMBER_OF_CAMERAS 5

static int old_key = -1;
static bool demo = true;
static bool leds_enabled = true;
static bool cameras_enabled = false;
static bool L_pressed = false;
static bool old_L_pressed = false;
static bool C_pressed = false;
static bool old_C_pressed = false;
static bool autopilot = true;
static bool old_autopilot = true;

// Initialize the robot's information
static WbDeviceTag motors[NUMBER_OF_JOINTS];
static const char *motor_names[NUMBER_OF_JOINTS] = {"front left leg shoulder abduction motor",
                                                    "front left leg shoulder rotation motor",
                                                    "front left leg elbow motor",
                                                    "front right leg shoulder abduction motor",
                                                    "front right leg shoulder rotation motor",
                                                    "front right leg elbow motor",
                                                    "rear left leg shoulder abduction motor",
                                                    "rear left leg shoulder rotation motor",
                                                    "rear left leg elbow motor",
                                                    "rear right leg shoulder abduction motor",
                                                    "rear right leg shoulder rotation motor",
                                                    "rear right leg elbow motor"};
static WbDeviceTag cameras[NUMBER_OF_CAMERAS];
static const char *camera_names[NUMBER_OF_CAMERAS] = {"left head camera",
                                                      "right head camera",
                                                      "left flank camera",
                                                      "right flank camera",
                                                      "rear camera"};
static WbDeviceTag leds[NUMBER_OF_LEDS];
static const char *led_names[NUMBER_OF_LEDS] = {"left led", "right led"};

void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  const double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

// Movement decomposition
static void movement_decomposition(double *target) {
  const int n_steps_to_achieve_target = DURATION * 1000 / TIME_STEP;
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

static void lie_down() {
  const double motors_target_pos[NUMBER_OF_JOINTS] = {-0.40, -0.99, 1.59,  // Front left leg
                                                 0.40, -0.99, 1.59,  // Front right leg
                                                -0.40, -0.99, 1.59,  // Rear left leg
                                                 0.40, -0.99, 1.59}; // Rear right leg
  movement_decomposition(motors_target_pos);
}

static void stand_up() {
  const double motors_target_pos[NUMBER_OF_JOINTS] = {-0.1, 0.0, 0.0,  // Front left leg
                                                 0.1, 0.0, 0.0,  // Front right leg
                                                -0.1, 0.0, 0.0,  // Rear left leg
                                                 0.1, 0.0, 0.0}; // Rear right leg

  movement_decomposition(motors_target_pos);
}

static void sit_down(bool give_paw) {
  // Sit down first
  double motors_target_pos[NUMBER_OF_JOINTS] = {-0.20, -0.40, -0.19,  // Front left leg
                                                 0.20, -0.40, -0.19,  // Front right leg
                                                -0.40, -0.90,  1.18,  // Rear left leg
                                                 0.40, -0.90,  1.18}; // Rear right leg

  movement_decomposition(motors_target_pos);

  if (give_paw) { // Front right leg
    // Stabilize posture
    double motors_target_pos_1[NUMBER_OF_JOINTS] = {-0.20, -0.30,  0.05,  // Front left leg
                                                   0.20, -0.40, -0.19,  // Front right leg
                                                  -0.40, -0.90,  1.18,  // Rear left leg
                                                   0.49, -0.90,  0.80}; // Rear right leg

    movement_decomposition(motors_target_pos_1);

    double initialTime = wb_robot_get_time();
    while (true) {
      double time = wb_robot_get_time() - initialTime;
      wb_motor_set_position(motors[4], 0.2 * sin(2 * time) + 0.6);  // Upperarm movement
      wb_motor_set_position(motors[5], 0.4 * sin(2 * time));  // Forearm movement
      if (time >= 2*DURATION)
        break;
      step();
    }
    // Get back in sitting posture
    double motors_target_pos_2[NUMBER_OF_JOINTS] = {-0.20, -0.40, -0.19,  // Front left leg
                                                     0.20, -0.40, -0.19,  // Front right leg
                                                    -0.40, -0.90,  1.18,  // Rear left leg
                                                     0.40, -0.90,  1.18}; // Rear right leg

    movement_decomposition(motors_target_pos_2);
  }

}

static void recover() {

  // Bring all 4 legs close to the body
  double motors_target_pos_1[NUMBER_OF_JOINTS] = {0.0, -0.99, 1.59,  // Front left leg
                                                  0.0, -0.99, 1.59,  // Front right leg
                                                  0.0, -0.99, 1.59,  // Rear left leg
                                                  0.0, -0.99, 1.59}; // Rear right leg
// Bend both left legs back
  double motors_target_pos_2[NUMBER_OF_JOINTS] = {0.0, -1.69, 1.59,  // Front left leg
                                                  0.0, -0.99, 1.59,  // Front right leg
                                                  0.0, -1.69, 1.59,  // Rear left leg
                                                  0.0, -0.99, 1.59}; // Rear right leg
// Tighten the front right leg against the body and spread the right rear leg.
  double motors_target_pos_3[NUMBER_OF_JOINTS] = { 0.00, -1.69, 1.59,  // Front left leg
                                                  -0.49, -0.77, 1.59,  // Front right leg
                                                   0.00, -1.69, 1.59,  // Rear left leg
                                                   0.49, -0.99, 1.59}; // Rear right leg
// Bend the rear right leg backwards
  double motors_target_pos_4[NUMBER_OF_JOINTS] = {-0.49, -1.69, 1.59,  // Front left leg
                                                  -0.49, -0.77, 1.59,  // Front right leg
                                                   0.00, -1.69, 1.59,  // Rear left leg
                                                   0.49, -1.50, 1.59}; // Rear right leg
// Extend the front right legm
  double motors_target_pos_5[NUMBER_OF_JOINTS] = {-0.49, -1.69, 1.59,  // Front left leg
                                                   0.00,  1.50, -0.35, // Front right leg
                                                   0.00, -1.69, 1.59,  // Rear left leg
                                                   0.49, -1.50, 1.59}; // Rear right leg
// Spread the shoulder of the front right leg
  double motors_target_pos_6[NUMBER_OF_JOINTS] = {-0.49, -1.69, 1.59,  // Front left leg
                                                   0.49,  1.50, -0.35,  // Front right leg
                                                   0.00, -1.69, 1.59,  // Rear left leg
                                                   0.49, -1.50, 1.59}; // Rear right leg
// Folds in the forearm of the front right leg
  double motors_target_pos_7[NUMBER_OF_JOINTS] = { 0.00, -1.69, 1.59,  // Front left leg
                                                   0.49,  1.50, 0.50,  // Front right leg
                                                   0.00, -1.69, 1.59,  // Rear left leg
                                                   0.49, -1.50, 1.59}; // Rear right leg
// Straighten the shoulders
  double motors_target_pos_8[NUMBER_OF_JOINTS] = { 0.00, -1.69, 1.59,  // Front left leg
                                                  -0.15,  1.50, 0.50,  // Front right leg
                                                   0.00, -1.69, 1.59,  // Rear left leg
                                                  -0.15, -1.50, 1.59}; // Rear right leg
// Tilt to the left side, bringing to the body the left-side' shoulders and arms.
  double motors_target_pos_9[NUMBER_OF_JOINTS] = { 0.40, -0.40, 0.80,  // Front left leg
                                                  -0.30,  1.50, 0.50,  // Front right leg
                                                   0.40, -0.40, 0.80,  // Rear left leg
                                                  -0.30, -1.50, 1.59}; // Rear right leg
// Bend the front right leg under the body and further back.
  double motors_target_pos_10[NUMBER_OF_JOINTS] = { 0.40, -0.40, 0.80,  // Front left leg
                                                   -0.30, -1.69, 1.59,  // Front right leg
                                                    0.40, -0.40, 0.80,  // Rear left leg
                                                   -0.30, -1.50, 1.59}; // Rear right leg
// Straighten the front right shoulder
  double motors_target_pos_11[NUMBER_OF_JOINTS] = { 0.40, -0.40, 0.80,  // Front left leg
                                                    0.00, -1.69, 1.59,  // Front right leg
                                                    0.40, -0.40, 0.80,  // Rear left leg
                                                   -0.30, -1.50, 1.59}; // Rear right leg
// Lean on the rear right arm and straighten the shoulder as well.
  double motors_target_pos_12[NUMBER_OF_JOINTS] = {0.40, -0.40, 0.80,  // Front left leg
                                                   0.40, -1.69, 1.59,  // Front right leg
                                                   0.40, -0.40, 0.80,  // Rear left leg
                                                   0.40, -1.69, 1.59}; // Rear right leg
// Stabilize reception
  double motors_target_pos_13[NUMBER_OF_JOINTS] = {-0.40, -0.75, 1.20,  // Front left leg
                                                    0.40, -0.99, 1.59,  // Front right leg
                                                   -0.40, -0.75, 1.20,  // Rear left leg
                                                    0.40, -0.99, 1.59}; // Rear right leg
  movement_decomposition(motors_target_pos_1);
  movement_decomposition(motors_target_pos_2);
  movement_decomposition(motors_target_pos_3);
  movement_decomposition(motors_target_pos_4);
  movement_decomposition(motors_target_pos_5);
  movement_decomposition(motors_target_pos_6);
  movement_decomposition(motors_target_pos_7);
  movement_decomposition(motors_target_pos_8);
  movement_decomposition(motors_target_pos_9);
  movement_decomposition(motors_target_pos_10);
  movement_decomposition(motors_target_pos_11);
  movement_decomposition(motors_target_pos_12);
  movement_decomposition(motors_target_pos_13);
  lie_down();
  passive_wait(1);
  stand_up();
}

static void check_keyboard() {
  int key = wb_keyboard_get_key();
  if ((key >= 0) && key != old_key) {
    switch (key) {
      case WB_KEYBOARD_UP:
        printf("Robot stands up\n");
        stand_up();
        autopilot = false;
        break;
      case WB_KEYBOARD_DOWN:
        printf("Robot lies down\n");
        lie_down();
        autopilot = false;
        break;
      case WB_KEYBOARD_END:
      case 'L':
        if (key != old_key)  // perform this action just once
          L_pressed = !L_pressed;
        break;
      case 'D':
        if (key != old_key)
          demo = !demo;
        break;
      case 'C':
          if (key != old_key)
            C_pressed = !C_pressed;
          break;
      case 'A':
        if (key != old_key)
          autopilot = !autopilot;
        break;
      default:
        fprintf(stderr, "Wrong keyboard input\n");
        break;
    }
  }
  if (autopilot != old_autopilot) {
    old_autopilot = autopilot;
    if (autopilot)
      printf("Auto control\n");
    else
      printf("Manual control\n");
  }
  if (C_pressed != old_C_pressed) {
    old_C_pressed = C_pressed;
    if (cameras_enabled) {  // Switch off
      for (int i = 0; i < NUMBER_OF_CAMERAS; i++)
        wb_camera_disable(cameras[i]);
      cameras_enabled = false;
      printf("Cameras OFF\n");
    }
    else {
      for (int i = 0; i < NUMBER_OF_CAMERAS; i++)
        wb_camera_enable(cameras[i], TIME_STEP);
      cameras_enabled = true;
      printf("Cameras ON\n");
    }
  }
  if (L_pressed != old_L_pressed) {
    old_L_pressed = L_pressed;
    if (leds_enabled) {  // Switch off
      for (int i = 0; i < NUMBER_OF_LEDS; i++)
        wb_led_set(leds[i], LEDS_OFF);
      leds_enabled = false;
      printf("LEDs OFF\n");
    }
    else {
      for (int i = 0; i < NUMBER_OF_LEDS; i++)
        wb_led_set(leds[i], LEDS_ON);
      leds_enabled = true;
      printf("LEDs ON\n");
    }
  }
  old_key = key;
}

// Demonstration
static void run_demo() {
  printf("The demonstration will start in...\n");
  printf("3 \n"); passive_wait(1);
  printf("2 \n"); passive_wait(1);
  printf("1 \n"); passive_wait(1);
  printf("\n");

  printf("Demonstration started !\n");
  recover();
  lie_down();
  stand_up();
  sit_down(GIVE_PAW);
  stand_up();
  printf("Demonstration finished !\n");
  printf("\n");
  demo = false;
}

// Autopilot mode: Move forward
static void run_autopilot() {

  printf("Autopilot started !\n");
  /*lie_down();
  printf("AUTO Lied down !\n");
  passive_wait(1);

  stand_up();
  printf("AUTO Standed up !\n");
  passive_wait(1);
  printf("Autopilot ended !\n");
  autopilot = false;
*/
}

static void display_instructions() {
  printf("Control commands:\n");
  printf(" - Robot stands up: press 'Arrow up'\n");
  printf(" - Robot lies down: press 'Arrow down'\n");
  printf(" - Enable/disable cameras: press 'C'\n");
  printf(" - Enable/disable LEDs: press 'L'\n");
  printf(" - Demonstration mode: press 'D'\n");
  printf(" - Autopilot mode : press 'A'\n");
  printf("\n");
}

int main(int argc, char **argv) {
  wb_robot_init();

  // List devices
  int n_devices = wb_robot_get_number_of_devices();
  printf("Available devices:\n");
  for (int i = 0; i < n_devices; i++) {
    WbDeviceTag tag = wb_robot_get_device_by_index(i);
    const char *name = wb_device_get_name(tag);
    printf(" Device #%d name = %s\n", i, name);
  }
  printf("\n");

  // Get cameras
  for (int i = 0; i < NUMBER_OF_CAMERAS; i++)
    cameras[i] = wb_robot_get_device(camera_names[i]);

  // Get the LEDs and enable them
  for (int i = 0; i < NUMBER_OF_LEDS; i++) {
    leds[i] = wb_robot_get_device(led_names[i]);
    wb_led_set(leds[i], LEDS_ON);
  }

  // Get the motors (joints) and set initial target position to 0
  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    motors[i] = wb_robot_get_device(motor_names[i]);
    wb_motor_set_position(motors[i], 0.0);
  }

  // Display instructions to control the robot
  display_instructions();
  passive_wait(2.0);

  // Enable the keyboard inputs
  old_key = 0;
  wb_keyboard_enable(TIME_STEP);

  // Main loop
  //  o Demonstration mode if nothing else done
  //  o Autopilot mode (automatically after the demo)
  //  o Enable/Disable cameras with keyboard "C"
  while (true) {
    step();
    check_keyboard();
    if (demo)
      run_demo();
    if (autopilot)
      run_autopilot();
  }

  wb_robot_cleanup();
  return EXIT_FAILURE;
}
