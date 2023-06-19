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
 * Description:  A controller for the Khepera robot intended to be
 *               cross-compiled in order to run on the real robot.
 */

#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#define FORWARD_SPEED 8
#define TURN_SPEED 5
#define SENSOR_THRESHOLD 150
#define NB_SENSOR 8
#define TIME_STEP 64

static WbDeviceTag ds[NB_SENSOR], left_motor, right_motor, left_position_sensor, right_position_sensor;

/*
 * The RS232 communication system is useful to send commands to a real
 * Khepera robot using the Khepera protocol, this way, it is possible for
 * example to retrive information from custom extension turret using the
 * "T" protocol command. The example here shows how the Khepera robot
 * answers to the "B" command, asking for software versions.
 * This system will be ignored in simulation and with the cross-compiled
 * version of this controller.
 */

#ifndef KROS_COMPILATION
static WbDeviceTag rs232_out, rs232_in;
#endif

int main() {
  short left_speed, right_speed;
  unsigned short ds_value[NB_SENSOR];
  char text[4];
  int i;
#ifndef KROS_COMPILATION
  const char command[] = "B\n";
#endif

  left_speed = 0;
  right_speed = 0;
  double left_encoder_offset = 0.0;
  double right_encoder_offset = 0.0;

  wb_robot_init();

  text[1] = 's';
  text[3] = '\0';
  for (i = 0; i < NB_SENSOR; i++) {
    text[0] = 'd';
    text[2] = '0' + i;
    ds[i] = wb_robot_get_device(text); /* distance sensors */
  }

#ifndef KROS_COMPILATION
  rs232_out = wb_robot_get_device("rs232_out");
  rs232_in = wb_robot_get_device("rs232_in");
#endif
  for (i = 0; i < NB_SENSOR; i++)
    wb_distance_sensor_enable(ds[i], TIME_STEP);
#ifndef KROS_COMPILATION
  wb_receiver_enable(rs232_in, TIME_STEP);
#endif

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);

  for (;;) { /* The robot never dies! */
    ds_value[1] = wb_distance_sensor_get_value(ds[1]);
    ds_value[2] = wb_distance_sensor_get_value(ds[2]);
    ds_value[3] = wb_distance_sensor_get_value(ds[3]);
    ds_value[4] = wb_distance_sensor_get_value(ds[4]);

    if (ds_value[2] > SENSOR_THRESHOLD && ds_value[3] > SENSOR_THRESHOLD) {
      left_speed = -TURN_SPEED; /* go backwards */
      right_speed = -TURN_SPEED;
    } else if (ds_value[1] < SENSOR_THRESHOLD && ds_value[2] < SENSOR_THRESHOLD && ds_value[3] < SENSOR_THRESHOLD &&
               ds_value[4] < SENSOR_THRESHOLD) {
      left_speed = FORWARD_SPEED; /* go forward */
      right_speed = FORWARD_SPEED;
    } else if (ds_value[3] > SENSOR_THRESHOLD || ds_value[4] > SENSOR_THRESHOLD) {
      left_speed = -TURN_SPEED; /* turn left */
      right_speed = TURN_SPEED;
    }

    if (ds_value[1] > SENSOR_THRESHOLD || ds_value[2] > SENSOR_THRESHOLD) {
      right_speed = -TURN_SPEED; /* turn right */
      left_speed = TURN_SPEED;
    }

    const int left_encoder = wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset;
    const int right_encoder = wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset;

    if (left_encoder > 90)
      left_encoder_offset = left_encoder;

    if (right_encoder > 10) {
      right_encoder_offset = right_encoder;
#ifndef KROS_COMPILATION
      wb_emitter_send(rs232_out, command, strlen(command) + 1);
#endif
    }
    /* Set the motor speeds */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);

    wb_robot_step(TIME_STEP); /* run one step */

#ifndef KROS_COMPILATION
    if (wb_receiver_get_queue_length(rs232_in) > 0) {
      printf("received %s\n", (const char *)wb_receiver_get_data(rs232_in));
      wb_receiver_next_packet(rs232_in);
    }
#endif
  }

  return 0;
}
