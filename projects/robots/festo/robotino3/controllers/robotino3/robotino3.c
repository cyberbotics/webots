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
 * Description: Simple avoidance controller and possibility to use the keyboard
 *              to control manually the robot. A demo mode is also present.
 *              The velocity of each wheel is set according to a
 *              Braitenberg-like algorithm which takes the values returned
 *              by the 9 infrared sensors as input.
 */

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "base.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NUMBER_OF_INFRARED_SENSORS 9

static double time_step;
static int old_key = -1;
static bool demo = false;
static bool autopilot = true;
static bool old_autopilot = true;
static bool display_message = false;

extern WbDeviceTag wheels[3];
static WbDeviceTag camera;
static WbDeviceTag infrared_sensors[NUMBER_OF_INFRARED_SENSORS];
static const char *infrared_sensors_names[NUMBER_OF_INFRARED_SENSORS] = {"ir1", "ir2", "ir3", "ir4", "ir5",
                                                                         "ir6", "ir7", "ir8", "ir9"};
static const char *camera_name = {"Webcam for Robotino 3"};

double convert_volt_to_meter(WbDeviceTag tag, double V) {
  const char *model = wb_device_get_model(tag);

  if (strcmp(model, "GP2D120"))
    return 0.1594 * pow(V, -0.8533) - 0.02916;
  else if (strcmp(model, "GP2Y0A02YK0F"))
    return 0.7611 * pow(V, -0.9313) - 0.1252;
  else if (strcmp(model, "GP2Y0A41SK0F"))
    return 0.1594 * pow(V, -0.8533) - 0.02916;
  else if (strcmp(model, "GP2Y0A710K0F"))
    return 20.24 * pow(V, -4.76) + 0.6632;
  else {
    printf("This infrared sensor model is not compatible\n");
    return -1;
  }
}

static void step() {
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  const double start_time = wb_robot_get_time();
  do {
    step();
    base_accelerate();
  } while (start_time + sec > wb_robot_get_time());
}

// Demonstration
// Displace in a square shape to show the mecanum wheels capabilities
static void run_demo() {
  printf("Demonstration started\n");

  // Make a square movement
  base_forwards();
  passive_wait(2.0);
  base_reset();
  passive_wait(1.0);

  base_strafe_left();
  passive_wait(2.0);
  base_reset();
  passive_wait(1.0);

  base_backwards();
  passive_wait(2.0);
  base_reset();
  passive_wait(1.0);

  base_strafe_right();
  passive_wait(2.0);
  base_reset();
  passive_wait(1.0);

  base_forwards();
  passive_wait(2.0);
  base_reset();
  passive_wait(1.0);

  base_strafe_left();
  passive_wait(2.0);
  base_reset();
  passive_wait(1.0);

  base_backwards();
  passive_wait(2.0);
  base_reset();
  passive_wait(1.0);

  base_turn_left();
  passive_wait(3.0);
  base_reset();
  base_turn_right();
  passive_wait(3.0);
  base_reset();
  printf("Demonstration finished\n");
  demo = false;
}

// Autopilot mode
// Move using a simple Braitenberg algorithm to avoid obstacle
static void run_autopilot(bool display_message, int *last_displayed_second) {
  // Get sensors values and convert them
  double sensors_values[NUMBER_OF_INFRARED_SENSORS];
  for (int i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i)
    sensors_values[i] = convert_volt_to_meter(infrared_sensors[0], wb_distance_sensor_get_value(infrared_sensors[i]));

  if (display_message) {
    // Display some IR sensor data every second
    int display_second = (int)wb_robot_get_time();
    if (display_second != *last_displayed_second) {
      *last_displayed_second = display_second;
      printf("time = %d [s]\n", display_second);
      for (int i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i)
        printf("infrared sensor('%s') = %f [m]\n", infrared_sensors_names[i], sensors_values[i]);
    }
  }
  base_braitenberg_avoidance(sensors_values);
}

static void display_instructions() {
  printf("Control commands:\n");
  printf(" - Arrows:       Move the robot\n");
  printf(" - Page Up/Down: Rotate the robot\n");
  printf(" - Space: Stop/Reset\n");
  printf("\n");
  printf("Demonstration mode: press 'D'\n");
  printf("Autopilot mode : press 'A'\n");
}

static void check_keyboard() {
  int key = wb_keyboard_get_key();
  if ((key >= 0) && key != old_key) {
    switch (key) {
      case WB_KEYBOARD_UP:
        printf("Go forwards\n");
        base_forwards();
        autopilot = false;
        break;
      case WB_KEYBOARD_DOWN:
        printf("Go backwards\n");
        base_backwards();
        autopilot = false;
        break;
      case WB_KEYBOARD_LEFT:
        printf("Strafe left\n");
        base_strafe_left();
        autopilot = false;
        break;
      case WB_KEYBOARD_RIGHT:
        printf("Strafe right\n");
        base_strafe_right();
        autopilot = false;
        break;
      case WB_KEYBOARD_PAGEUP:
        printf("Turn left\n");
        base_turn_left();
        autopilot = false;
        break;
      case WB_KEYBOARD_PAGEDOWN:
        printf("Turn right\n");
        base_turn_right();
        autopilot = false;
        break;
      case WB_KEYBOARD_END:
      case ' ':
        printf("Reset\n");
        base_reset();
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
  // Initialization
  wb_robot_init();
  time_step = wb_robot_get_basic_time_step();

  // Get and enable the camera
  camera = wb_robot_get_device(camera_name);
  if (camera > 0)
    wb_camera_enable(camera, time_step);

  // Get and enable the infrared sensors
  for (int i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i) {
    infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
    wb_distance_sensor_enable(infrared_sensors[i], time_step);
  }

  // Get the motors and set target position to infinity (speed control)
  char wheel_name[16];
  for (int i = 0; i < 3; i++) {
    sprintf(wheel_name, "wheel%d_joint", i);
    wheels[i] = wb_robot_get_device(wheel_name);
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0.0);
  }

  // Display instructions to control the robot
  display_instructions();
  passive_wait(2.0);

  // Enable the keyboard inputs
  old_key = 0;
  wb_keyboard_enable(time_step);

  // Store the last time a message was displayed
  int last_display_second = 0;

  // Main loop
  //  o Autopilot mode if nothing else done
  //  o Demonstration mode
  //  o Manual control with keyboard arrows
  while (true) {
    step();
    check_keyboard();
    if (demo)
      run_demo();
    if (autopilot) {
      run_autopilot(display_message, &last_display_second);
    }
    base_accelerate();
  }

  wb_robot_cleanup();
  return 0;
}
