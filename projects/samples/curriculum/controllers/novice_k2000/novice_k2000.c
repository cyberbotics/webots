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

// Included libraries
#include <webots/led.h>    //obtain leds library
#include <webots/robot.h>  //obtain main library of webots

// Global defines
#define ON 1          // boolean True - Led On
#define OFF 0         // boolean False - Led Off
#define TIME_STEP 32  // [ms] // time step of the simulation
#define NB_LEDS 8     // Number of leds

// Global variables
WbDeviceTag led[NB_LEDS];  // array containing references to leds
int i = 0;

static void switch_off_all_leds() {
  int it;  // index
  for (it = 0; it < NB_LEDS; it++)
    wb_led_set(led[it], OFF);  // switch off led[it]
}

int main() {
  wb_robot_init();

  int it;                 // index
  char text[5] = "led0";  // led node to be search in world

  // loop on led array
  for (it = 0; it < NB_LEDS; it++) {
    led[it] = wb_robot_get_device(text);  // put led[it] into array
    text[3]++;                            // change text tag : led0, led1, ..., led7
  }

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    switch_off_all_leds();

    // switch on current led
    wb_led_set(led[i], ON);

    // update index i. i has to loop from 0 to 7
    i++;
    i = i % NB_LEDS;
  }

  wb_robot_cleanup();

  return 0;
}
