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
 * Description:  An example of use of the encoders of a differential wheel
 */

#include <stdio.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 64
#define SPEED_UNIT 0.1
#define ENCODER_UNIT 0.25

int main(void) {
  int goal[2] = {0, 0};
  WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;
  double left_encoder_offset = 0.0;
  double right_encoder_offset = 0.0;

  wb_robot_init();

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

  while (wb_robot_step(TIME_STEP) != -1) {
    int i;
    int encoder_value[2];
    float speed[2];

    encoder_value[0] = ENCODER_UNIT * (wb_position_sensor_get_value(left_position_sensor) - left_encoder_offset);
    encoder_value[1] = ENCODER_UNIT * (wb_position_sensor_get_value(right_position_sensor) - right_encoder_offset);

    /*
     * simply turn the wheels in the direction corresponding to the
     * objectif position, when it is reached, stop the wheel.
     */
    if (encoder_value[0] != goal[0] || encoder_value[1] != goal[1]) {
      for (i = 0; i < 2; i++) {
        speed[i] = goal[i] - encoder_value[i];
        if (speed[i] != 0)
          speed[i] = speed[i] > 0 ? 40 : -40;
      }

      /* set the motor speeds */
      wb_motor_set_velocity(left_motor, SPEED_UNIT * speed[0]);
      wb_motor_set_velocity(right_motor, SPEED_UNIT * speed[1]);
    } else {
      /*
       * When both wheels are in place, we choose randomly new objectif
       * positions and we reset the encoders.
       */
      for (i = 0; i < 2; i++)
        goal[i] = (int)(20 * ((float)rand() / RAND_MAX)) - 10;

      left_encoder_offset = wb_position_sensor_get_value(left_position_sensor);
      right_encoder_offset = wb_position_sensor_get_value(right_position_sensor);

      printf("Goal position for the encoders: %d %d\n", goal[0], goal[1]);
    }
  }

  wb_robot_cleanup();

  return 0;
}
