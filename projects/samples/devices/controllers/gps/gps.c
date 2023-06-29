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
 * Description:   An example of a controller using gps devices of two types,
 *                either using the device from Webots or using a supervisor
 *                sending the position with an emitter. You can choose which
 *                one you are using with the keyboard.
 *                For more information on how the GPS is simulated using a
 *                supervisor, please have a look at the controller
 *                gps_supervisor.
 */

#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#define SPEED 6
#define TIME_STEP 64

typedef enum { GPS, SUPERVISED } gps_mode_types;

int main() {
  double left_speed, right_speed;

  wb_robot_init();

  /* get a handler to the distance sensors */
  WbDeviceTag ds0 = wb_robot_get_device("ds0");
  WbDeviceTag ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  /* get and enable GPS device */
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  /* get and enable receiver */
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* get key presses from keyboard */
  wb_keyboard_enable(TIME_STEP);

  printf("Press 'G' to read the GPS device's position\n");
  printf("Press 'S' to read the Supervisor's position\n");
  printf("Press 'V' to read the GPS device's speed vector\n");

  while (wb_robot_step(TIME_STEP) != -1) {
    switch (wb_keyboard_get_key()) {
      case 'G': {
        const double *gps_values = wb_gps_get_values(gps);
        printf("GPS position: %.3f %.3f %.3f\n", gps_values[0], gps_values[1], gps_values[2]);
        break;
      }
      case 'S': {
        /* received nothing so far */
        if (wb_receiver_get_queue_length(receiver) == 0)
          break;

        /* destroy all packets but last one */
        while (wb_receiver_get_queue_length(receiver) > 1)
          wb_receiver_next_packet(receiver);

        /* read the last packet */
        const double *buffer = wb_receiver_get_data(receiver);
        printf("Supervisor position: %.3f %.3f %.3f\n", buffer[0], buffer[1], buffer[2]);
        break;
      }
      case 'V': {
        const double *speed_vector_values = wb_gps_get_speed_vector(gps);
        printf("GPS speed vector: %.3f %.3f %.3f\n", speed_vector_values[0], speed_vector_values[1], speed_vector_values[2]);
        break;
      }
      default:
        break;
    }

    double ds0_value = wb_distance_sensor_get_value(ds0);
    double ds1_value = wb_distance_sensor_get_value(ds1);

    if (ds1_value > 500) {
      /*
       * If both distance sensors are detecting something, this means that
       * we are facing a wall. In this case we need to move backwards.
       */
      if (ds0_value > 200) {
        left_speed = -SPEED;
        right_speed = -SPEED / 2;
      } else {
        /*
         * We turn proportionnaly to the sensors value because the
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
       * If nothing was detected we can move forward at maximal speed.
       */
      left_speed = SPEED;
      right_speed = SPEED;
    }

    /* Set the motor speeds. */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
