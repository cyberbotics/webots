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
 * Description:   Simple hello controller
 */

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
//#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define DURATION 4 // Time in second to perform action
#define TIME_STEP (int)wb_robot_get_basic_time_step()  // from world file
#define NUMBER_OF_JOINTS 12
#define NUMBER_OF_CAMERAS 5

static int old_key = -1;
static bool demo = false;
static bool autopilot = true;
static bool cameras_activated = false;
static bool old_autopilot = true;

// initialize the robot's information
static WbDeviceTag motors[NUMBER_OF_JOINTS];
static const char *motor_names[NUMBER_OF_JOINTS] = {"front left leg shoulder elevation motor",
                                                    "front left leg shoulder rotation motor",
                                                    "front left leg elbow motor",
                                                    "front right leg shoulder elevation motor",
                                                    "front right leg shoulder rotation motor",
                                                    "front right leg elbow motor",
                                                    "rear left leg shoulder elevation motor",
                                                    "rear left leg shoulder rotation motor",
                                                    "rear left leg elbow motor",
                                                    "rear right leg shoulder elevation motor",
                                                    "rear right leg shoulder rotation motor",
                                                    "rear right leg elbow motor"};
/*
static WbDeviceTag position_sensors[NUMBER_OF_JOINTS];
static const char *position_sensor_names[NUMBER_OF_JOINTS] = {"front left leg shoulder elevation sensor",
                                                               "front left leg shoulder rotation sensor",
                                                               "front left leg elbow sensor",
                                                               "front right leg shoulder elevation sensor",
                                                               "front right leg shoulder rotation sensor",
                                                               "front right leg elbow sensor",
                                                               "rear left leg shoulder elevation sensor",
                                                               "rear left leg shoulder rotation sensor",
                                                               "rear left leg elbow sensor",
                                                               "rear right leg shoulder elevation sensor",
                                                               "rear right leg shoulder rotation sensor",
                                                               "rear right leg elbow sensor"};
*/
static WbDeviceTag cameras[NUMBER_OF_CAMERAS];
static const char *camera_names[NUMBER_OF_CAMERAS] = {"left head camera",
                                                      "right head camera",
                                                      "left flank camera",
                                                      "right flank camera",
                                                      "rear camera"};

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

static void lie_down() {
  double motors_target_pos[NUMBER_OF_JOINTS] = {-0.40, -1.00,  1.60,  // front left leg
                                                -0.40, -1.00,  1.60,  // front right leg
                                                -0.40, -1.00,  1.60,  // rear left leg
                                                -0.40, -1.00,  1.60}; // rear right leg

  int n_steps_to_achieve_target = DURATION * 1000 / TIME_STEP;
  for (int i = 0; i < n_steps_to_achieve_target; i++) {
    double ratio = (double)i / n_steps_to_achieve_target;
    wb_motor_set_position(motors[i], motors_target_pos[i] * ratio);
    step();
  }
}

static void stand_up() {
  double motors_target_pos[NUMBER_OF_JOINTS] = {0.0, 0.0, 0.0,  // front left leg
                                                0.0, 0.0, 0.0,  // front right leg
                                                0.0, 0.0, 0.0,  // rear left leg
                                                0.0, 0.0, 0.0}; // rear right leg

  int n_steps_to_achieve_target = DURATION * 1000 / TIME_STEP;
  for (int i = 0; i < n_steps_to_achieve_target; i++) {
    double ratio = (double)i / n_steps_to_achieve_target;
    wb_motor_set_position(motors[i], motors_target_pos[i] * ratio);
    step();
  }
}

static void sit_down() {
  double motors_target_pos[NUMBER_OF_JOINTS] = {-0.2, -0.90,  1.18,  // front left leg
                                                 0.2, -0.90,  1.18,  // front right leg
                                                -0.2, -0.40, -0.19,  // rear left leg
                                                 0.2, -0.40, -0.19}; // rear right leg

  int n_steps_to_achieve_target = DURATION * 1000 / TIME_STEP;
  for (int i = 0; i < n_steps_to_achieve_target; i++) {
    double ratio = (double)i / n_steps_to_achieve_target;
    wb_motor_set_position(motors[i], motors_target_pos[i] * ratio);
    step();
  }
}

static void give_paw(int number) {
  double motors_target_pos[NUMBER_OF_JOINTS] = {-0.2, -0.90,  1.18,  // front left leg
                                                 0.0,  0.60,  0.00,  // front right leg
                                                -0.2, -0.40, -0.19,  // rear left leg
                                                 0.2, -0.40, -0.19}; // rear right leg

  int n_steps_to_achieve_target = (DURATION/2) * 1000 / TIME_STEP;
  for (int i = 0; i < n_steps_to_achieve_target; i++) {
    double ratio = (double)i / n_steps_to_achieve_target;
    wb_motor_set_position(motors[i], motors_target_pos[i] * ratio);
    step();
  }
  // TODO: Finished this part
/*
  // Move up and down the front right leg the number of times specified
  double initialTime = wb_robot_get_time();
  double time = wb_robot_get_time() - initialTime;
  wb_motor_set_position(motors[4], 0.2 * sin(5 * time) + 0.6);  // Upperarm movement
  wb_motor_set_position(motors[5], 0.8 * sin(5 * time));  // Forearm movement
*/
}

// Demonstration
// Displace in a square shape to show the mecanum wheels capabilities
static void run_demo() {
  printf("The demonstration will start in...\n");
  printf("3... \n"); passive_wait(1);
  printf("2... \n"); passive_wait(1);
  printf("1... \n"); passive_wait(1);
  printf("\n");

  printf("Demonstration started !\n");
  lie_down();   passive_wait(1);
  stand_up();   passive_wait(1);
  sit_down();   passive_wait(1);
  give_paw(3);  passive_wait(1);
  stand_up();   passive_wait(1);
  printf("Demonstration finished !\n");

  printf("\n");
  demo = false;
}

/*
// Autopilot mode: Move forward
static void run_autopilot() {
  //go_forward();
}
*/

static void display_instructions() {
  printf("Control commands:\n");
  printf(" - Arrow up: The robot stands up\n");
  printf(" - Arrow down: The robot lies down\n");
  printf(" - C: Activates cameras\n");
  printf("\n");
  printf("Demonstration mode: press 'D'\n");
  printf("Autopilot mode : press 'A'\n");
}

static void check_keyboard() {
  int key = wb_keyboard_get_key();
  if ((key >= 0) && key != old_key) {
    switch (key) {
      case WB_KEYBOARD_UP:
        printf("Robot stands up\n");
        stand_up();
        break;
      case WB_KEYBOARD_DOWN:
        printf("Robot lies down\n");
        lie_down();
        break;
      case 'C':
        if (key != old_key)  // perform this action just once
          cameras_activated = !cameras_activated;
        break;
      case 'D':
        if (key != old_key)  // perform this action just once
          demo = !demo;
        break;
      case 'A':
        if (key != old_key)  // perform this action just once
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
  old_key = key;
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

  // Get and enable cameras
  for (int i = 0; i < NUMBER_OF_CAMERAS; i++) {
    cameras[i] = wb_robot_get_device(camera_names[i]);
    wb_camera_enable(cameras[i], TIME_STEP);
  }

  // Get the motors (joints) and set initial target position to 0
  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    motors[i] = wb_robot_get_device(motor_names[i]);
    wb_motor_set_position(motors[i], 0.0);
  }
/*
  // Get and enable position sensors (joints)
  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    position_sensors[i] = wb_robot_get_device(position_sensor_names[i]);
    wb_position_sensor_enable(position_sensors[i], TIME_STEP);
  }
*/
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
  //double initialTime = wb_robot_get_time();
  while (true) {
    check_keyboard();

    if (demo)
      run_demo();

    if (cameras_activated)
      for (int i = 0; i < NUMBER_OF_CAMERAS; i++)
        wb_camera_enable(cameras[i], TIME_STEP);
    else
      for (int i = 0; i < NUMBER_OF_CAMERAS; i++)
        wb_camera_disable(cameras[i]);
/*
    if (autopilot)
      run_autopilot();
*/
    step();
  }

  wb_robot_cleanup();
  return EXIT_FAILURE;
}
