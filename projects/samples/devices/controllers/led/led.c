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
 * Description:  An example of controller using a led device.
 */

#include <stdio.h>
#include <stdlib.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SPEED 6
#define TIME_STEP 64

int main() {
  WbDeviceTag ds0, ds1, led[3], left_motor, right_motor;
  int led_counter[3] = {0, 0, 0};
  int left_speed, right_speed;
  int random_color;
  int led1_increase = 10;

  wb_robot_init();

  /* get a handler to the distance sensors. */
  ds0 = wb_robot_get_device("ds0");
  ds1 = wb_robot_get_device("ds1");

  /* and also to the leds. */
  led[0] = wb_robot_get_device("led0");
  led[1] = wb_robot_get_device("led1");
  led[2] = wb_robot_get_device("led2");

  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control).
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * For led 0 (right hand side), which is not a gradual LED, we simply
     * select randomly a color and we change its color using it. According
     * to the documentation, when using the color number 0 this turns the
     * led off.
     */
    if (led_counter[0] == 0) {
      random_color = (int)(10 * ((float)rand() / RAND_MAX));
      wb_led_set(led[0], random_color);

      /* We will rechange the color in a random amount of time. */
      led_counter[0] = (int)(4 * ((float)rand() / RAND_MAX)) + 4;
      if (random_color == 0)
        printf("LED 0 is turned off\n");
      else
        printf("LED 0 uses color %d\n", random_color);
    } else
      led_counter[0]--;

    /*
     * For led 1 (left hand side), which is a monochromatic gradual LED,
     * we increase and decrease its value, making it glow
     */
    wb_led_set(led[1], led_counter[1]);
    led_counter[1] += led1_increase;
    if (led_counter[1] > 255) {
      led_counter[1] = 255;
      led1_increase = -10;
    } else if (led_counter[1] < 0) {
      led_counter[1] = 0;
      led1_increase = 10;
    }
    /*
     * For led 2 (back side), which is a RGB gradual LED, we set a
     * random value each 1024 ms.
     */
    if (led_counter[2] == 0)
      wb_led_set(led[2], (int)(0xffffff * ((float)rand() / RAND_MAX)));
    led_counter[2]++;
    if (led_counter[2] == 16)
      led_counter[2] = 0;

    const double ds0_value = wb_distance_sensor_get_value(ds0);
    const double ds1_value = wb_distance_sensor_get_value(ds1);

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
