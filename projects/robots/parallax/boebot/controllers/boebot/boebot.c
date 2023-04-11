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
 * Description:  The controller of the boebot robot.
 */

#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64
#define FORWARD_SPEED 8
#define TURN_SPEED 10

/*
 * We use this variable to allow the robot to make a longer
 * backward movement.
 */
static int backward_counter = 0;

int main() {
  /* initialize Webots */
  wb_robot_init();

  /* Here we get handles to the devices and enable them. */
  WbDeviceTag left_distance_sensor = wb_robot_get_device("lds");
  WbDeviceTag right_distance_sensor = wb_robot_get_device("rds");

  WbDeviceTag left_light_sensor = wb_robot_get_device("lls");
  WbDeviceTag right_light_sensor = wb_robot_get_device("rls");

  WbDeviceTag left_led = wb_robot_get_device("left_led");
  WbDeviceTag right_led = wb_robot_get_device("right_led");

  wb_distance_sensor_enable(left_distance_sensor, TIME_STEP);
  wb_distance_sensor_enable(right_distance_sensor, TIME_STEP);

  wb_light_sensor_enable(left_light_sensor, TIME_STEP);
  wb_light_sensor_enable(right_light_sensor, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    double left_speed, right_speed;
    int ldsv = wb_distance_sensor_get_value(left_distance_sensor);
    int rdsv = wb_distance_sensor_get_value(right_distance_sensor);

    /*
     * Compute the current state using the distance sensors if we are not
     * making a backward movement.
     */
    int state;
    if (backward_counter == 0)
      state = ldsv << 1 | rdsv;
    else
      state = 0;

    /*
     * Depending on which sensor has detected an obstacle, we adapt the speed
     * of the wheels and we notify it by turning the corresponding led on.
     */
    switch (state) {
      case 0:
        /* Obstacle front of the Boe Bot. */
        left_speed = -TURN_SPEED;
        right_speed = -2 * TURN_SPEED;
        wb_led_set(left_led, 1);
        wb_led_set(right_led, 1);

        /* If we have already begun the movement we simply continue. */
        if (backward_counter == 0)
          backward_counter = 10;
        else
          backward_counter--;
        break;
      case 1:
        /* Obstacle on the left of the Boe Bot. */
        left_speed = TURN_SPEED;
        right_speed = -TURN_SPEED;
        wb_led_set(left_led, 1);
        wb_led_set(right_led, 0);
        break;
      case 2:
        /* Obstacle on the right of the Boe Bot. */
        left_speed = -TURN_SPEED;
        right_speed = TURN_SPEED;
        wb_led_set(left_led, 0);
        wb_led_set(right_led, 1);
        break;
      case 3:
        /* No obstacle front of the Boe Bot. */
        left_speed = FORWARD_SPEED;
        right_speed = FORWARD_SPEED;
        wb_led_set(left_led, 0);
        wb_led_set(right_led, 0);
        break;
      default:
        left_speed = 0;
        right_speed = 0;
        wb_led_set(left_led, 0);
        wb_led_set(right_led, 0);
        break;
    }

    /* Set the motor speeds */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
