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
 * Description:  Main file of the interface with the Shrimp.
 */

/*
 * This program has been developed as an interface between Webots and any
 * other program made to control the Bluebotics' Shrimp robot.
 *
 * It emulates both the Shrimp's protocol and behavior.
 *
 * For more informations about this robot or the protocol, please have a
 * look at their documentation.
 *
 * For more general informations about the interface of this program, please
 * have a look at the shrimp.h file.
 */

#include "shrimp_protocol.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define WHEELS_NUMBER 6

#define SHRIMP_MAJOR_VERSION 1
#define SHRIMP_MINOR_VERSION 4
#define SHRIMP_PATCH_NUMBER 7

/* The maximal values defined by the protocol. */
#define MAX_VELOCITY 127
#define MAX_STEERING_ANGLE 90

/* This is the time step with which Webots will update the values. */
#define TIME_STEP 32

/* The technical values of both the motor, the gearhead and the encoder. */
#define MOTOR_MAX_RPM 5000
#define REDUCTION_RATION 86
#define ENCODER_PER_ROTATION 16

/* The size of buffer used to answer to the commands. */
#define MAX_BUFFER_SIZE 32

/*
 * In C unsigned char is really a byte of information
 * As our protocol is binary this is really useful.
 */
typedef unsigned char byte;

/* All the commands of the protocol implemented in our model. */

/*
 * In order to know more about these commands please have a look at
 * 'Controlling the Shrimp'.
 */
static void command_01(byte *);
static byte command_04(byte *);
static void command_05(byte *);
static void command_06(void);
static void command_07(byte *);
static void command_08(byte *);

/*
 * As not all the wheels are orientated in the same direction, we need to
 * know it in order to be able to produce an homogeneous movement.
 */
static int wheel_rotational_direction[WHEELS_NUMBER];

static byte robot_status;

static short int robot_velocity;
static short int robot_steer_angle;

/* The motors of our robot. */
static WbDeviceTag wheels[WHEELS_NUMBER];
static WbDeviceTag wheels_sensor[WHEELS_NUMBER];
static WbDeviceTag front_direction;
static WbDeviceTag back_direction;

/*
 * This function must be called before any other, it initializes all the
 * static variables used in this program.
 */
void shrimp_init(void) {
  int i;

  /* This variable is used to loop over all our motors and position sensors. */
  char temp_motors_name[] = "wheel motor 00";
  char temp_sensors_name[] = "wheel sensor 00";

  /*
   * Here we define in which direction we need to turn the wheels
   * in order to produce a forward movement.
   */
  wheel_rotational_direction[0] = -1;
  wheel_rotational_direction[1] = 1;
  wheel_rotational_direction[2] = -1;
  wheel_rotational_direction[3] = 1;
  wheel_rotational_direction[4] = -1;
  wheel_rotational_direction[5] = -1;

  for (i = 0; i < WHEELS_NUMBER; i++) {
    wheels[i] = wb_robot_get_device(temp_motors_name);
    wheels_sensor[i] = wb_robot_get_device(temp_sensors_name);

    temp_motors_name[13]++;
    temp_sensors_name[14]++;

    if ((i % 10) == 9) {
      temp_motors_name[13] = 0;
      temp_motors_name[12]++;
      temp_sensors_name[14] = 0;
      temp_sensors_name[13]++;
    }

    if (wheels[i] != 0) {
      /* This enables the position reading of the motor. */
      wb_position_sensor_enable(wheels_sensor[i], TIME_STEP);

      wb_motor_set_velocity(wheels[i], 0);
      wb_motor_set_position(wheels[i], 0);
    } else
      printf("Error: Motor of wheel %d was not found\n", i);
  }

  /* These are the directional motors. */
  front_direction = wb_robot_get_device("front wheel rotation");
  back_direction = wb_robot_get_device("back wheel rotation");

  /* The robot is always turned on. */
  robot_status = 0x01;
  robot_velocity = 0;
  robot_steer_angle = 0;
}

/*
 * This is the main interface with our robot, it emulates the Shrimp's
 * protocol. This is a passive robot, you need to send a command to it in
 * order recieve some data. This is done by using this function which returns
 * its answer which structure is defined by the protocol.
 *
 * For more informations about this protocol please have a look at
 * 'Controlling the Shrimp'.
 */
unsigned char *shrimp_send(unsigned char *command) {
  static byte result[MAX_BUFFER_SIZE];
  int i;

  for (i = 0; i < MAX_BUFFER_SIZE; i++)
    result[i] = 0;

  /* All the answers normally begin with the same tag as the commands */
  result[0] = command[0];

  switch (command[0]) {
    case 0x00:
      break;
    case 0x01:
      command_01(result);
      break;
    case 0x02:
      /* falls through */
    case 0x03:
#ifdef VERBOSE
      printf("The simulated robot is always turned on\n");
#endif
      break;
    case 0x04:
      result[0] = command_04(command);
      break;
    case 0x05:
      command_05(result);
      break;
    case 0x06:
      command_06();
      break;
    case 0x07:
      command_07(result);
      break;
    case 0x08:
      command_08(result);
      break;
    case 0x09:
#ifdef VERBOSE
      printf("The simulated robot does not use power\n");
#endif
      /*
       * This is the value for a full 12V battery using the units defined
       * in the protocol.
       *
       * For more informations about this protocol please have a look at
       * 'Controlling the Shrimp'.
       */
      result[1] = 192;
      break;
    case 0x0a:
#ifdef VERBOSE
      printf("The simulated robot does not use power\n");
#endif
      /* This is the status for ALL_OK. */
      result[1] = 0x01;
      break;
    case 0x0d:
      /* falls through */
    case 0x0e:
#ifdef VERBOSE
      printf("The simulated robot does not have a buzzer\n");
#endif
      break;
    case 0x0b:
      /* falls through */
    case 0x0c:
#ifdef VERBOSE
      printf("Infrared remote control is not implemented\n");
#endif
      break;
    default:
#ifdef VERBOSE
      printf("Error: Function 0x%2x not implemented yet\n", (unsigned int)command[3]);
#endif
      /*
       * According to the protocol, this is the error code for unknown
       * commands.
       *
       * For more informations about this protocol please have a look at
       * 'Controlling the Shrimp'.
       */
      result[0] = 0x80;
      break;
  }

  return result;
}

/*
 * This function allows to read the firmware version of the robot.
 *
 * In order to know more about this function please have a look at
 * 'Controlling the Shrimp'.
 */
static void command_01(byte *answer) {
  answer[1] = SHRIMP_MAJOR_VERSION;
  answer[2] = SHRIMP_MINOR_VERSION;
  answer[3] = SHRIMP_PATCH_NUMBER;
}

/*
 * This function allows to to set the speed and the steering angle of the
 * robot.
 * It returns the tag of the answer as it can produce some errors which need
 * to be noticed in the answer.
 *
 * In order to know more about this function please have a look at
 * 'Controlling the Shrimp'.
 */
static byte command_04(byte *command) {
  int i;
  byte result;
  double radius_back, radius_front, radius_right, radius_left;
  double temp_speed[WHEELS_NUMBER];
  double steer_angle_back, steer_angle_front;
  double angular_speed, radius_center;

  robot_velocity = (short int)command[1];
  robot_steer_angle = (short int)command[2];

  /* We store the original answer tag in case we do not have any error. */
  result = command[0];

  /* We clear the ROBOT_STOPPED. */
  robot_status &= 0xFD;

  /* We need to convert the recieved numbers as they are in fact signed. */
  if (robot_velocity > 128)
    robot_velocity -= 256;
  if (robot_steer_angle > 128)
    robot_steer_angle -= 256;

  /* We check for errors. */
  if (robot_steer_angle > 0) {
    if (robot_steer_angle > MAX_STEERING_ANGLE) {
      robot_steer_angle = MAX_STEERING_ANGLE;
      result = 0x81;
    } else if (robot_steer_angle == MAX_STEERING_ANGLE)
      result = 0x83;
  } else if (robot_steer_angle < 0) {
    if (robot_steer_angle < -MAX_STEERING_ANGLE) {
      robot_steer_angle = -MAX_STEERING_ANGLE;
      result = 0x81;
    } else if (robot_steer_angle == -MAX_STEERING_ANGLE)
      result = 0x83;
  }

  if (robot_velocity > 0) {
    if (robot_velocity > MAX_VELOCITY) {
      robot_velocity = MAX_VELOCITY;
      result = 0x81;
    } else if (robot_velocity == MAX_VELOCITY)
      result = 0x83;
  } else if (robot_velocity < 0) {
    if (robot_velocity < -MAX_VELOCITY) {
      robot_velocity = -MAX_VELOCITY;
      result = 0x81;
    } else if (robot_velocity == -MAX_VELOCITY)
      result = 0x83;
  }

  /*
   * In order for the robot to be able to turn, we need to compute the
   * value of each wheel depending on the radius of curvature of the
   * movement of the robot.
   *
   * The values we have recieved are the ones for the back wheel. We
   * need to convert them.
   */
  steer_angle_back = (robot_steer_angle * 2 * M_PI) / 360.0;
  temp_speed[5] = ((robot_velocity * MOTOR_MAX_RPM * 2 * M_PI) / (60 * REDUCTION_RATION * MAX_VELOCITY));

  /*
   * The steering angle allows us to compute the radius of curvature of the
   * back wheel.
   */
  radius_back = 0.242 / cos(M_PI / 2 - fabs(steer_angle_back));

  /* Using it we can deduce the angular speed of the robot. */
  angular_speed = temp_speed[5] / radius_back;

  /*
   * And with this values we can compute the radius of curvature of the
   * center of the robot. We need to take in accout the special case of
   * sqrt(0) which returns an error.
   */
  if (fabs(radius_back) != 0.242)
    radius_center = sqrt((radius_back * radius_back) - (0.242 * 0.242));
  else
    radius_center = 0;

  /* Now we can compute all the values for the front wheel. */
  radius_front = sqrt((radius_center * radius_center) + (0.271 * 0.271));
  temp_speed[0] = angular_speed * radius_front;
  steer_angle_front = (M_PI / 2) - asin(radius_center / radius_front);

  /*
   * And depending on the steering angle of the robot, we can also compute
   * all the values for the left and right wheels.
   */
  if (robot_steer_angle < 0)
    steer_angle_front *= -1;

  if (robot_steer_angle > 0) {
    radius_left = sqrt(((radius_center + 0.182) * (radius_center + 0.182)) + (0.089 * 0.089));
    radius_right = sqrt(((radius_center - 0.182) * (radius_center - 0.182)) + (0.089 * 0.089));
  } else {
    radius_left = sqrt(((radius_center - 0.182) * (radius_center - 0.182)) + (0.089 * 0.089));
    radius_right = sqrt(((radius_center + 0.182) * (radius_center + 0.182)) + (0.089 * 0.089));
  }

  temp_speed[1] = angular_speed * radius_left;
  temp_speed[2] = angular_speed * radius_right;

  /*
   * In the case the radius of curvature is smaller than the distance from
   * the center to the left/right wheels, the wheels should turn in the
   * opposite direction.
   */
  if (radius_center < 0.182) {
    if (robot_steer_angle < 0)
      temp_speed[1] *= -1;
    else
      temp_speed[2] *= -1;
  }

  /* The wheels of the same side turn at the same speed. */
  temp_speed[3] = temp_speed[1];
  temp_speed[4] = temp_speed[2];

  /* Finally we update the robots values. */
  for (i = 0; i < WHEELS_NUMBER; ++i) {
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], wheel_rotational_direction[i] * temp_speed[i]);
  }

  wb_motor_set_position(front_direction, -steer_angle_front);
  wb_motor_set_position(back_direction, steer_angle_back);

  return result;
}

/*
 * This function allows to read the speed and the steering angle of the
 * robot.
 *
 * In order to know more about this function please have a look at
 * 'Controlling the Shrimp'.
 */
static void command_05(byte *answer) {
  answer[1] = (byte)robot_velocity;
  answer[2] = (byte)robot_steer_angle;
}

/*
 * This function is the emergency stop of the robot.
 *
 * In order to know more about this function please have a look at
 * 'Controlling the Shrimp'.
 */
static void command_06(void) {
  int i;

  robot_velocity = 0;

  for (i = 0; i < WHEELS_NUMBER; i++)
    wb_motor_set_velocity(wheels[i], 0);

  robot_status |= 0x02;
}

/*
 * This function allows to read the wheels encoder values of the robot.
 *
 * In order to know more about this function please have a look at
 * 'Controlling the Shrimp'.
 */
static void command_07(byte *answer) {
  int i;

  for (i = 0; i < WHEELS_NUMBER; i++) {
    const int temp_encoder = ceil(wheel_rotational_direction[i] *
                                  (wb_position_sensor_get_value(wheels_sensor[i]) / (2 * M_PI)) * ENCODER_PER_ROTATION);

    answer[(i * 4) + 1] = temp_encoder >> 24;
    answer[(i * 4) + 2] = (temp_encoder >> 16) & 0xFF;
    answer[(i * 4) + 3] = (temp_encoder >> 8) & 0xFF;
    answer[(i * 4) + 4] = temp_encoder & 0xFF;
  }
}

/*
 * This function allows to status of the robot.
 *
 * In order to know more about this function please have a look at
 * 'Controlling the Shrimp'.
 */
static void command_08(byte *answer) {
  answer[1] = robot_status;
}
