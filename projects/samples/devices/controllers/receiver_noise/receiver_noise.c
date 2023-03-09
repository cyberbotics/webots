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
 * Description:   Example usage of signalStrengthNoise and directionNoise in a Receiver,
 *                used to compute the position of an Emitter and compared to the actual
 *                position of the Emitter measured with a noise-free GPS.
 */

#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/utils/ansi_codes.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#define SPEED 6
#define TIME_STEP 64
#define COMMUNICATION_CHANNEL 1
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

typedef enum { EMITTER, RECEIVER } robot_types;

int main() {
  WbDeviceTag ds0, ds1, communication, gps, left_motor, right_motor;
  robot_types robot_type;
  double left_speed, right_speed;
  double ds0_value, ds1_value;

  wb_robot_init();

  /*
   * as we are using the same controller for the emitter and the reciever,
   * we need to differentiate them
   */
  if (strncmp(wb_robot_get_name(), "MyBot emitter", 13) == 0) {
    robot_type = EMITTER;
    communication = wb_robot_get_device("emitter");

    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);

    /* if the cannel is not the right one, change it */
    const int channel = wb_emitter_get_channel(communication);
    if (channel != COMMUNICATION_CHANNEL)
      wb_emitter_set_channel(communication, COMMUNICATION_CHANNEL);
  } else if (strncmp(wb_robot_get_name(), "MyBot receiver", 14) == 0) {
    robot_type = RECEIVER;
    communication = wb_robot_get_device("receiver");
    wb_receiver_enable(communication, TIME_STEP);
  } else {
    printf("Unrecognized robot name '%s'. Exiting...\n", wb_robot_get_name());
    wb_robot_cleanup();
    return 0;
  }

  /* get a handler to the distance sensors and enable them */
  ds0 = wb_robot_get_device("ds0");
  ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * the emitter simply sends a message, and the receiver
     * exctract the position of the receiver from it.
     */
    if (robot_type == EMITTER) {
      /* send null-terminated message */
      const char *message = "Hello!";
      wb_emitter_send(communication, message, strlen(message) + 1);
      const double *gpsPosition = wb_gps_get_values(gps);
      /* print real position measured from the GPS */
      ANSI_CLEAR_CONSOLE();
      printf("GPS position: time = %.3lf   X = %.3lf Y = %.3lf\n", wb_robot_get_time(), gpsPosition[0], gpsPosition[1]);

    } else {
      /* is there at least one packet in the receiver's queue ? */
      if (wb_receiver_get_queue_length(communication) > 0) {
        /* compute and print position of the Emitter from signal strength and direction */
        double signalStrength = wb_receiver_get_signal_strength(communication);
        const double *direction = wb_receiver_get_emitter_direction(communication);
        double dist = 1 / sqrt(signalStrength);
        ANSI_CLEAR_CONSOLE();
        printf("Emitter position: time = %.3lf   X = %.3lf Y = %.3lf\n", wb_robot_get_time(), direction[0] * dist,
               direction[1] * dist);

        wb_receiver_next_packet(communication);
      }
    }

    if (robot_type == EMITTER) {
      ds0_value = wb_distance_sensor_get_value(ds0);
      ds1_value = wb_distance_sensor_get_value(ds1);

      if (ds1_value > 500) {
        /*
         * If both distance sensors are detecting something, this means that
         * we are facing a wall. In this case we need to move backwards.
         */
        if (ds0_value > 200) {
          left_speed = -SPEED / 2;
          right_speed = -SPEED;
        } else {
          /*
           * we turn proportionally to the sensors value because the
           * closer we are from the wall, the more we need to turn.
           */
          left_speed = -ds1_value / 100;
          right_speed = (ds0_value / 100) + 0.5;
        }
      } else if (ds0_value > 500) {
        left_speed = (ds1_value / 100) + 0.5;
        right_speed = -ds0_value / 100;
      } else {
        /*
         * if nothing was detected we can move forward at maximal speed.
         */
        left_speed = SPEED;
        right_speed = SPEED;
      }

      /* set the motor speeds. */
      wb_motor_set_velocity(left_motor, CLAMP(left_speed, -10.0, 10.0));
      wb_motor_set_velocity(right_motor, CLAMP(right_speed, -10.0, 10.0));
    }
  }

  wb_robot_cleanup();

  return 0;
}
