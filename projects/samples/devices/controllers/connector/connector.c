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
 * Description:  An example of controller using a Connector device.
 */

#include <math.h>
#include <stdlib.h>
#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define SPEED 2
#define TIME_STEP 64

typedef enum { CONNECTOR_IN_PLACE, CONNECTING, PASSING_OVER, NEXT_HOP } states;

int main() {
  WbDeviceTag connector, upper_motor, upper_position_sensor, left_motor, right_motor;
  /*
   * we use these different variables for:
   *   - robot_number : To differentiate the robots
   *   - hop_counts : To know when to invert the direction
   *   - direction : To know in which direction we will perform the next hop
   */
  int robot_number, hop_counts = 0, direction = 1;
  states state;

  wb_robot_init();

  /* get a handler to the connector, the motor and the position_sensor. */
  connector = wb_robot_get_device("connector");
  upper_motor = wb_robot_get_device("upper motor");
  upper_position_sensor = wb_robot_get_device("upper sensor");

  /* activate them. */
  wb_connector_enable_presence(connector, TIME_STEP);
  wb_position_sensor_enable(upper_position_sensor, TIME_STEP);

  wb_motor_set_position(upper_motor, -1.57);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /*
   * we need to substract the ASCII code of '0' (i.e., 48) to convert
   * the char of the number into an integer.
   */
  robot_number = (wb_robot_get_name())[6] - '0';
  state = CONNECTOR_IN_PLACE;

  while (wb_robot_step(TIME_STEP) != -1) {
    int left_speed = 0;
    int right_speed = 0;

    switch (state) {
      /* wait for the motor to be in place. */
      case CONNECTOR_IN_PLACE:
        if (fabs(wb_position_sensor_get_value(upper_position_sensor) + (direction * 1.56)) < 0.01)
          state = CONNECTING;
        break;

      /* wait for another Connector device to lock with. */
      case CONNECTING:
        if (wb_connector_get_presence(connector) == 1) {
          wb_connector_lock(connector);
          wb_motor_set_position(upper_motor, direction * 1.57);
          state = PASSING_OVER;
        }

        /*
         * if it is the robot which is moving, we need to take it closer to
         * the other one so that they can attach to each other
         */
        if (robot_number == 1) {
          left_speed = direction * SPEED;
          right_speed = direction * SPEED;
        }
        break;

      /* wait for the motor to be in the new place (hopping). */
      case PASSING_OVER:
        if (fabs(wb_position_sensor_get_value(upper_position_sensor) - (direction * 1.56)) < 0.01) {
          wb_connector_unlock(connector);
          state = NEXT_HOP;
          hop_counts++;
        }
        break;

      /* wait for the other Connector device to be far enough. */
      case NEXT_HOP:
        if (wb_connector_get_presence(connector) == 0) {
          /*
           * if it is the robot which is moving, we need to replace the
           * motor in order to be able to atach to the next robot
           */
          if (robot_number == 1) {
            if (hop_counts % 2 == 0)
              direction *= -1;
            wb_motor_set_position(upper_motor, -1.57 * direction);
            state = CONNECTOR_IN_PLACE;

            /*
             * If not, the motor is already in the right place as the moving
             * robot will come back at the same place.
             */
          } else {
            state = CONNECTING;
            direction *= -1;
          }
        }

        /*
         * the moving robot needs to get away from the connector in order to
         * avoid attaching always to the same robot.
         */
        if (robot_number == 1) {
          left_speed = direction * SPEED;
          right_speed = direction * SPEED;
        }
        break;
    }
    /* set the motor speeds. */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
