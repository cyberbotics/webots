/*
 * Copyright 1996-2019 Cyberbotics Ltd.
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
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 * Description: Simple avoidance controller
 *              The velocity of each wheel is set according to a
 *              Braitenberg-like algorithm which takes the values returned
 *              by the Hokuyo URG-04LX-UG01 as input.
 */

#include <webots/keyboard.h>
#include <webots/robot.h>

#include <base.h>
#include <tiny_math.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32

#define NUMBER_OF_INFRARED_SENSORS 9
static const char *infrared_sensors_names[NUMBER_OF_INFRARED_SENSORS] = {"ir1", "ir2", "ir3", "ir4", "ir5",
                                                                         "ir6", "ir7", "ir8", "ir9"};

static int old_key = -1;
static bool demo = true;
static bool autopilot = true;
static bool old_autopilot = true;

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

// Demonstration
// Displace in a square shape to show the mecanum wheels capabilities
static void run_demo() {
  // Make a square movement
  passive_wait(1.0);

  base_forwards();
  passive_wait(1.0);
  base_reset();
  passive_wait(1.0);

  base_strafe_left();
  passive_wait(1.0);
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
  passive_wait(1.0);
  base_reset();
  passive_wait(1.0);

  base_backwards();
  passive_wait(1.0);
  base_reset();
  passive_wait(1.0);

  base_turn_left();
  passive_wait(1.0);
  base_reset();
  base_turn_right();
  passive_wait(1.0);
  base_reset();
}

// Autopilot
// Move using a Braitenberg algorithm
static void run_autopilot() {
  base_braitenberg_avoidance();
}

static void display_instructions() {
  printf("Control commands:\n");
  printf(" Arrows:       Move the robot\n");
  printf(" Page Up/Down: Rotate the robot\n");
  printf(" Space: Reset\n");
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
        if (key != old_key) {  // perform this action just once
          demo = !demo;
        }
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
  // base_init();
  // base_init_sensors();

  int time_step = (int)wb_robot_get_basic_time_step();
  int i;

  // get and enable the infrared sensors
  WbDeviceTag infrared_sensors[NUMBER_OF_INFRARED_SENSORS];
  for (i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i) {
    infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
    wb_distance_sensor_enable(infrared_sensors[i], time_step);
  }

  // get the motors and set target position to infinity (speed control)
  WbDeviceTag motor_0, motor_1, motor_2;
  motor_0 = wb_robot_get_device("wheel0_joint");
  motor_1 = wb_robot_get_device("wheel1_joint");
  motor_2 = wb_robot_get_device("wheel2_joint");
  wb_motor_set_position(motor_0, INFINITY);
  wb_motor_set_position(motor_1, INFINITY);
  wb_motor_set_position(motor_2, INFINITY);
  wb_motor_set_velocity(motor_0, 0.0);
  wb_motor_set_velocity(motor_1, 0.0);
  wb_motor_set_velocity(motor_2, 0.0);

  // Display instructions to control the robot
  display_instructions();
  passive_wait(2.0);

  int old_key = 0;
  wb_keyboard_enable(TIME_STEP);

  // store the last time a message was displayed
  int last_display_second = 0;

  // User moves the robot with keyboard arrows
  while (true) {
    step();
    // display some sensor data every second
    // and change randomly the led colors
    int display_second = (int)wb_robot_get_time();
    if (display_second != last_display_second) {
      last_display_second = display_second;
      for (i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i)
        printf("infrared sensor('%s') = %f [m]\n", infrared_sensors_names[i],
               wb_distance_sensor_get_value(infrared_sensors[i]));
    }

    // simple obstacle avoidance algorithm
    // based on the front infrared sensors
    double speed_offset = 0.2 * (MAX_SPEED - 0.03 * wb_distance_sensor_get_value(infrared_sensors[3]));
    double speed_delta =
      0.03 * wb_distance_sensor_get_value(infrared_sensors[2]) - 0.03 * wb_distance_sensor_get_value(infrared_sensors[4]);
    wb_motor_set_velocity(left_motor, speed_offset + speed_delta);
    wb_motor_set_velocity(right_motor, speed_offset - speed_delta);

    /*
    check_keyboard();
    if (autopilot)
      run_autopilot();
    if (demo)
      run_demo();
    */
  }

  wb_robot_cleanup();

  return 0;
}
