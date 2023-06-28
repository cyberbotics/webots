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

#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Generate a random number between min and max.
static double rand_float(double min, double max) {
  const double range = (max - min);
  const double div = RAND_MAX / range;
  return min + (rand() / div);
}

int main(int argc, char **argv) {
  wb_robot_init();

  const int timestep = (int)wb_robot_get_basic_time_step();
  const int seed = time(NULL);
  srand(seed);

  printf("This C controller actuates the motor with a constant speed, actuates the pivot with a random sinusoidal function, "
         "and sets the LED colors.\n");

  WbDeviceTag pivot = wb_robot_get_device("pivot");
  WbDeviceTag motor = wb_robot_get_device("motor");
  wb_motor_set_position(motor, INFINITY);

  wb_led_set(wb_robot_get_device("motor led"), 0xFF0000);
  wb_led_set(wb_robot_get_device("pivot led"), 0x00FF00);

  // Choose 3 global random numbers.
  const double a = rand_float(-1.0, 1.0);
  const double b = rand_float(-1.0, 1.0);
  const double c = rand_float(0.5, 5.0);

  // Loop until the simulator stops the controller.
  while (wb_robot_step(timestep) != -1) {
    const double current_time = wb_robot_get_time();

    wb_motor_set_position(pivot, 0.5 * (sin(current_time * c + b) * a));
    wb_motor_set_velocity(motor, -c);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
