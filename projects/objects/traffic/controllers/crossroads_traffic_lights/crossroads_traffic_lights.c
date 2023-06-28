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
 * Description:   Example of a traffic light system in crossroads.
 */

#include <stdio.h>
#include <webots/led.h>
#include <webots/robot.h>

#define TIME_STEP 64
#define N_LIGHT 12

int main(int argc, char **argv) {
  wb_robot_init();
  WbDeviceTag red_light[N_LIGHT], orange_light[N_LIGHT], green_light[N_LIGHT];

  char red_light_string[32];
  char orange_light_string[32];
  char green_light_string[32];
  int i;
  for (i = 0; i < N_LIGHT; i++) {
    snprintf(red_light_string, 32, "red light %d", i);
    snprintf(orange_light_string, 32, "orange light %d", i);
    snprintf(green_light_string, 32, "green light %d", i);
    red_light[i] = wb_robot_get_device(red_light_string);
    orange_light[i] = wb_robot_get_device(orange_light_string);
    green_light[i] = wb_robot_get_device(green_light_string);
  }

  int t = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    t += TIME_STEP;
    // Turn on the green lights to go ahead or turn right in the first and third traffic light.
    // Turn on red lights in all others.
    if (t == TIME_STEP) {
      wb_led_set(green_light[0], 1);
      wb_led_set(green_light[1], 1);
      wb_led_set(green_light[6], 1);
      wb_led_set(green_light[7], 1);
      wb_led_set(red_light[3], 1);
      wb_led_set(red_light[4], 1);
      wb_led_set(red_light[9], 1);
      wb_led_set(red_light[10], 1);
      wb_led_set(red_light[2], 1);
      wb_led_set(red_light[8], 1);
      wb_led_set(red_light[5], 1);
      wb_led_set(red_light[11], 1);
    }
    // Turn off green lights and turn on orange lights.
    if (t == 125 * TIME_STEP) {
      wb_led_set(green_light[0], 0);
      wb_led_set(green_light[1], 0);
      wb_led_set(green_light[6], 0);
      wb_led_set(green_light[7], 0);
      wb_led_set(orange_light[0], 1);
      wb_led_set(orange_light[1], 1);
      wb_led_set(orange_light[6], 1);
      wb_led_set(orange_light[7], 1);
    }
    // Turn on green lights to go ahead or turn right in second and fourth traffic light.
    // Turn on red lights in all others.
    if (t == 156 * TIME_STEP) {
      wb_led_set(orange_light[0], 0);
      wb_led_set(orange_light[1], 0);
      wb_led_set(orange_light[6], 0);
      wb_led_set(orange_light[7], 0);
      wb_led_set(red_light[0], 1);
      wb_led_set(red_light[1], 1);
      wb_led_set(red_light[6], 1);
      wb_led_set(red_light[7], 1);
      wb_led_set(red_light[3], 0);
      wb_led_set(red_light[4], 0);
      wb_led_set(red_light[9], 0);
      wb_led_set(red_light[10], 0);
      wb_led_set(green_light[3], 1);
      wb_led_set(green_light[4], 1);
      wb_led_set(green_light[9], 1);
      wb_led_set(green_light[10], 1);
    }
    // Turn off green lights and turn on orange lights.
    if (t == 281 * TIME_STEP) {
      wb_led_set(green_light[3], 0);
      wb_led_set(green_light[4], 0);
      wb_led_set(green_light[9], 0);
      wb_led_set(green_light[10], 0);
      wb_led_set(orange_light[3], 1);
      wb_led_set(orange_light[4], 1);
      wb_led_set(orange_light[9], 1);
      wb_led_set(orange_light[10], 1);
    }
    // Turn on green lights to turn left in first and third traffic light.
    // Turn on red lights in all others.
    if (t == 312 * TIME_STEP) {
      wb_led_set(orange_light[3], 0);
      wb_led_set(orange_light[4], 0);
      wb_led_set(orange_light[9], 0);
      wb_led_set(orange_light[10], 0);
      wb_led_set(red_light[3], 1);
      wb_led_set(red_light[4], 1);
      wb_led_set(red_light[9], 1);
      wb_led_set(red_light[10], 1);
      wb_led_set(red_light[2], 0);
      wb_led_set(red_light[8], 0);
      wb_led_set(green_light[2], 1);
      wb_led_set(green_light[8], 1);
    }
    // Turn off green lights and turn on orange lights.
    if (t == 467 * TIME_STEP) {
      wb_led_set(green_light[2], 0);
      wb_led_set(green_light[8], 0);
      wb_led_set(orange_light[2], 1);
      wb_led_set(orange_light[8], 1);
    }
    // Turn on green lights to turn left in second and fourth traffic light.
    // Turn on red lights in all others.
    if (t == 498 * TIME_STEP) {
      wb_led_set(orange_light[2], 0);
      wb_led_set(orange_light[8], 0);
      wb_led_set(red_light[2], 1);
      wb_led_set(red_light[8], 1);
      wb_led_set(red_light[5], 0);
      wb_led_set(red_light[11], 0);
      wb_led_set(green_light[5], 1);
      wb_led_set(green_light[11], 1);
    }
    // Turn off green lights and turn on orange lights.
    if (t == 623 * TIME_STEP) {
      wb_led_set(green_light[5], 0);
      wb_led_set(green_light[11], 0);
      wb_led_set(orange_light[5], 1);
      wb_led_set(orange_light[11], 1);
    }
    // Turn off orange lights and red light to go ahead and turn right in first and third traffic light and restart.
    if (t == 654 * TIME_STEP) {
      wb_led_set(orange_light[5], 0);
      wb_led_set(orange_light[11], 0);
      wb_led_set(red_light[0], 0);
      wb_led_set(red_light[1], 0);
      wb_led_set(red_light[6], 0);
      wb_led_set(red_light[7], 0);
      t = 0;  // restart
    }
  };

  wb_robot_cleanup();

  return 0;
}
