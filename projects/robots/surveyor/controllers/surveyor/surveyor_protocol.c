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
 * Description:  Main file of the interface with the Surveyor.
 */

/*
 * This program has been developed as an interface between Webots and any
 * other program made to control the Surveyor robot.
 *
 * It emulates both the Surveyor's protocol and behavior.
 *
 * For more informations about this robot or the protocol, please have a
 * look at the on-line documentation.
 *
 * For more general informations about the interface of this program, please
 * have a look at the surveyor.h file.
 */

#include "surveyor_protocol.h"
#include "surveyor_scan.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define SENSORS_NUMBER 4

#define WHEELS_NUMBER 10
#define WHEEL_RADIUS 0.014

#define HIGH_VELOCITY 0.5
#define INITIAL_VELOCITY 0.3
#define LOW_VELOCITY 0.2

#define STRAIGHT_WHEEL_POWER 0.5
#define DRIFT_WHEEL_POWER 0.6
#define TURN_WHEEL_POWER 0.7

#define TURNING_20_DEG_DURATION 200

#define LOW_BYTE_MASK 0xFF
#define HIGH_BYTE_SHIFT 8

/* This is the time step with which Webots will update the values. */
#define TIME_STEP 16

/* The size of buffer used to answer to the commands. */
#define MAX_BUFFER_SIZE 90

/*
 * In C unsigned char is really a byte of information
 * As our protocol is binary this is really useful.
 */
typedef unsigned char byte;
typedef enum { OFF, ON, SIMULATION } status;
/* All the commands of the protocol implemented in our model. */

/*
 * In order to know more about these commands please have a look at
 * the function itself.
 */
static void command_00();
static void command_01();
static void command_02();
static void command_03();
static void command_04();
static void command_05();
static void command_06();
static void command_07();
static void command_08();
static void command_09();
static void command_dot();
static void command_plus();
static void command_minus();
static void command_M(int, int, int);
static void command_R(byte *);
static void command_B(byte *);

/* The motors of our robot. */
static WbDeviceTag wheels[WHEELS_NUMBER];
static WbDeviceTag position_sensor[WHEELS_NUMBER];
static WbDeviceTag sensors[SENSORS_NUMBER];
static WbDeviceTag camera;

static double robot_speed, left_power;
static int time, objective_time;
static int robot_id;

static status wandering;

/*
 * This function must be called before any other, it initializes all the
 * static variables used in this program.
 */
void surveyor_init() {
  int i;

  /* This variable is used to loop over all our motors. */
  char temp_motors_name[] = "wheel_motor00";

  /* This variable is used to loop over all our position sensors. */
  char temp_sensor_name[] = "wheel_sensor00";

  /* This variable is used to loop over all our sensors. */
  char temp_sensors_name[] = "ds0";

  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  for (i = 0; i < WHEELS_NUMBER; i++) {
    wheels[i] = wb_robot_get_device(temp_motors_name);
    position_sensor[i] = wb_robot_get_device(temp_sensor_name);

    temp_motors_name[12]++;
    temp_sensor_name[13]++;

    if ((i % 10) == 9) {
      temp_motors_name[12] = '0';
      temp_motors_name[11]++;
      temp_sensor_name[13] = '0';
      temp_sensor_name[12]++;
    }

    if (position_sensor[i] != 0)
      wb_position_sensor_enable(position_sensor[i], TIME_STEP);
    else
      printf("Error: position sensor of wheel %d was not found\n", i);

    if (wheels[i] == 0)
      printf("Error: Motor of wheel %d was not found\n", i);
    wb_motor_set_velocity(wheels[i], 0);
    wb_motor_set_position(wheels[i], INFINITY);  // speed control mode
  }

  for (i = 0; i < SENSORS_NUMBER; i++) {
    sensors[i] = wb_robot_get_device(temp_sensors_name);

    temp_sensors_name[2]++;

    if (wheels[i] != 0)
      wb_distance_sensor_enable(sensors[i], TIME_STEP);
    else
      printf("Error: Sensor %d was not found\n", i);
  }

  robot_speed = INITIAL_VELOCITY;
  left_power = STRAIGHT_WHEEL_POWER;
  time = 0;
  objective_time = 0;
  robot_id = 0;

  wandering = ON;
  wb_robot_step(TIME_STEP);
  surveyor_init_wander(wb_camera_get_image(camera));

  return;
}

/*
 * This is the main interface with our robot, it emulates the Surveyor's
 * protocol. This is a passive robot, you need to send a command to it in
 * order recieve some data. This is done by using this function which returns
 * its answer which structure is defined by the protocol.
 */
void surveyor_send(unsigned char *command, unsigned char *result) {
  int i;

  for (i = 0; i < MAX_BUFFER_SIZE; i++)
    result[i] = 0;

  if (command[0] == (byte)'@') {
    if ((command[1] == (byte)robot_id || command[1] == (byte)255) && robot_id != (byte)0)
      command = &command[2];
    else
      return;
  }

  /* All the answers normally begin with the same tag as the commands */
  result[0] = (byte)'#';
  result[1] = command[0];

  if (objective_time == 0) {
    switch (command[0]) {
      case '0':
        command_00();
        break;
      case '1':
        command_01();
        break;
      case '2':
        command_02();
        break;
      case '3':
        command_03();
        break;
      case '4':
        command_04();
        break;
      case '5':
        command_05();
        break;
      case '6':
        command_06();
        break;
      case '7':
        command_07();
        break;
      case '8':
        command_08();
        break;
      case '9':
        command_09();
        break;
      case 'M':
        command_M(command[1], command[2], command[3]);
        break;
      case 'm':
        wandering = SIMULATION;
        surveyor_init_wander(wb_camera_get_image(camera));
        break;
      case 'w':
        wandering = ON;
        surveyor_init_wander(wb_camera_get_image(camera));
        break;
      case 'W':
        wandering = OFF;
        command_05();
        break;
      case '.':
        command_dot();
        break;
      case '+':
        command_plus();
        break;
      case '-':
        command_minus();
        break;
      case 'r':
      case 'R':
        command_R(result);
        break;
      case 'B':
        command_B(result);
        break;
      case 'S':
        surveyor_get_scan_values(result);
        break;
      default:
#ifdef VERBOSE
        printf("Error: Function %c not implemented yet\n", (byte)command[0]);
#endif
        break;
    }
  }

  return;
}

/* This function updates the image of the camera and measures the time. */
void surveyor_update(int time_step) {
  const unsigned char *image;

  image = wb_camera_get_image(camera);

  if (objective_time != 0) {
    time += time_step;

    if (time >= objective_time)
      objective_time = 0;
  }

  /* Here we should analyse the values of the distance scan to move around. */
  if (wandering != OFF) {
    const byte wandering_command = surveyor_wandering(image);

    if (wandering == ON) {
      switch (wandering_command) {
        case '0':
          command_00();
          break;
        case '2':
          command_02();
          break;
        case '4':
          command_04();
          break;
        case '6':
          command_06();
          break;
        case '7':
          command_07();
          break;
        case '8':
          command_08();
          break;
        case '9':
          command_09();
          break;
        case '.':
          command_dot();
          break;
      }
    }
  }

  return;
}

/* The robot turns back left. */
static void command_01() {
  int i;
  double power;

  left_power = (1 - TURN_WHEEL_POWER);

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2)
      power = left_power;
    else
      power = 1 - left_power;
    power /= STRAIGHT_WHEEL_POWER;

    wb_motor_set_velocity(wheels[i], power * robot_speed / WHEEL_RADIUS);
  }
}

/* The robot drives backward. */
static void command_02() {
  int i;

  left_power = STRAIGHT_WHEEL_POWER;

  for (i = 0; i < WHEELS_NUMBER; i++)
    wb_motor_set_velocity(wheels[i], robot_speed / WHEEL_RADIUS);
}

/* The robot turns back right. */
static void command_03() {
  int i;
  double power;

  left_power = TURN_WHEEL_POWER;

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2)
      power = left_power;
    else
      power = 1 - left_power;
    power /= STRAIGHT_WHEEL_POWER;

    wb_motor_set_velocity(wheels[i], power * robot_speed / WHEEL_RADIUS);
  }
}

/* The robot turns on the left. */
static void command_04() {
  int i;
  double power;

  left_power = (1 - TURN_WHEEL_POWER);

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2)
      power = left_power;
    else
      power = 1 - left_power;
    power /= STRAIGHT_WHEEL_POWER;

    wb_motor_set_velocity(wheels[i], -power * robot_speed / WHEEL_RADIUS);
  }
}

/* The robot stops. */
static void command_05() {
  int i;

  for (i = 0; i < WHEELS_NUMBER; i++)
    wb_motor_set_velocity(wheels[i], 0);
}

/* The robot turns on the right. */
static void command_06() {
  int i;
  double power;

  left_power = TURN_WHEEL_POWER;

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2)
      power = left_power;
    else
      power = 1 - left_power;
    power /= STRAIGHT_WHEEL_POWER;

    wb_motor_set_velocity(wheels[i], -power * robot_speed / WHEEL_RADIUS);
  }

  return;
}

/* The robot drifts on the left. */
static void command_07() {
  int i;
  double power;

  left_power = (1 - DRIFT_WHEEL_POWER);

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2)
      power = left_power;
    else
      power = 1 - left_power;
    power /= STRAIGHT_WHEEL_POWER;

    wb_motor_set_velocity(wheels[i], -power * robot_speed / WHEEL_RADIUS);
  }

  return;
}

/* The robot drives straight forward. */
static void command_08() {
  int i;

  left_power = STRAIGHT_WHEEL_POWER;

  for (i = 0; i < WHEELS_NUMBER; i++)
    wb_motor_set_velocity(wheels[i], -robot_speed / WHEEL_RADIUS);

  return;
}

/* The robot drifts on the right. */
static void command_09() {
  int i;
  double power;

  left_power = DRIFT_WHEEL_POWER;

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2)
      power = left_power;
    else
      power = 1 - left_power;
    power /= STRAIGHT_WHEEL_POWER;

    wb_motor_set_velocity(wheels[i], -power * robot_speed / WHEEL_RADIUS);
  }

  return;
}

/* The robot rotates 20-deg left. */
static void command_00() {
  int i;

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2)
      wb_motor_set_velocity(wheels[i], INITIAL_VELOCITY / WHEEL_RADIUS);
    else
      wb_motor_set_velocity(wheels[i], -INITIAL_VELOCITY / WHEEL_RADIUS);
  }
  time = 0;
  objective_time = TURNING_20_DEG_DURATION;
}

/* The robot rotates 20-deg right. */
static void command_dot() {
  int i;

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2)
      wb_motor_set_velocity(wheels[i], -INITIAL_VELOCITY / WHEEL_RADIUS);
    else
      wb_motor_set_velocity(wheels[i], INITIAL_VELOCITY / WHEEL_RADIUS);
  }
  time = 0;
  objective_time = TURNING_20_DEG_DURATION;
}

/* The robot moves with the high velocity. */
static void command_plus() {
  int i;
  double power;

  robot_speed = HIGH_VELOCITY;

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2)
      power = left_power;
    else
      power = 1 - left_power;
    power /= STRAIGHT_WHEEL_POWER;

    wb_motor_set_velocity(wheels[i], power * robot_speed / WHEEL_RADIUS);
  }

  return;
}

/* The robot moves using the low velocity. */
static void command_minus() {
  int i;
  double power;

  robot_speed = LOW_VELOCITY;

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2)
      power = left_power;
    else
      power = 1 - left_power;
    power /= STRAIGHT_WHEEL_POWER;

    wb_motor_set_velocity(wheels[i], power * robot_speed / WHEEL_RADIUS);
  }

  return;
}

/* The robot moves according to the values received. */
static void command_M(int left_speed, int right_speed, int duration) {
  int i;
  double power = 0.0;

  if (left_speed > 127)
    left_speed -= 256;
  if (right_speed > 127)
    right_speed -= 256;

  for (i = 0; i < WHEELS_NUMBER; i++) {
    if (i < WHEELS_NUMBER / 2) {
      if (left_speed < 0)
        wb_motor_set_velocity(wheels[i], power * robot_speed / WHEEL_RADIUS);
      else
        wb_motor_set_velocity(wheels[i], -power * robot_speed / WHEEL_RADIUS);
      power = fabs((double)left_speed / 100);
    } else {
      if (right_speed < 0)
        wb_motor_set_velocity(wheels[i], power * robot_speed / WHEEL_RADIUS);
      else
        wb_motor_set_velocity(wheels[i], -power * robot_speed / WHEEL_RADIUS);
      power = fabs((double)right_speed / 100);
    }
    power /= STRAIGHT_WHEEL_POWER;

    wb_motor_set_velocity(wheels[i], power * robot_speed / WHEEL_RADIUS);
  }
  time = 0;
  objective_time = duration * 10;
}

/* The robot picks a new ID. */
static void command_R(byte *result) {
  result[1] = '#';
  result[2] = 'r';

  robot_id = (253 * (double)rand() / RAND_MAX) + 1;

  result[3] = robot_id;
}

/* This function returns the value of the DistanceSensors. */
static void command_B(byte *result) {
  int i;

  memcpy(result, "##BounceIR - ", 13);

  for (i = 0; i < SENSORS_NUMBER; i++) {
    result[(2 * i) + 13] = (byte)wb_distance_sensor_get_value(sensors[i]);
    result[(2 * i) + 14] = (byte)' ';
  }

  result[(2 * SENSORS_NUMBER) + 12] = '\n';
}
