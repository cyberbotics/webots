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

#include <webots/device.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Maximum limit of devices.
#define MAX_DEVICES 1024
// Set of predefined colors for LEDs.
#define N_LED_COLORS 7
static const int led_colors[N_LED_COLORS] = {
  0xFF0000,  // RED
  0x00FF00,  // GREEN
  0x0000FF,  // BLUE
  0x00FFFF,  // MINT
  0xFF00FF,  // PURPLE
  0xFFFF00,  // YELLOW
  0xFFFFFF   // WHITE
};

// Generate a random number between min and max.
static double rand_float(double min, double max) {
  const double range = (max - min);
  const double div = RAND_MAX / range;
  return min + (rand() / div);
}
static int rand_int(int min, int max) {
  return (rand() % (max + 1 - min)) + min;
}

int main(int argc, char **argv) {
  wb_robot_init();

  const int timestep = (int)wb_robot_get_basic_time_step();
  const int seed = time(NULL);
  srand(seed);

  printf("This demo C controller actuates the Tinkerbots motors with random sinusoidal functions and sets the LED colors.\n\n");
  printf("Devices:\n\n");

  // Parse all the devices; get the motors and set a random color for each LED.
  WbDeviceTag motors[MAX_DEVICES];
  int n_motors = 0;
  int d;
  for (d = 0; d < wb_robot_get_number_of_devices(); ++d) {
    WbDeviceTag device = wb_robot_get_device_by_index(d);
    WbNodeType device_type = wb_device_get_node_type(device);

    if (device_type == WB_NODE_ROTATIONAL_MOTOR) {
      printf("- Motor: %s\n", wb_device_get_name(device));
      // Store motor.
      motors[n_motors++] = device;
      // Switch to velocity control mode for unbounded motors.
      if (wb_motor_get_min_position(device) == wb_motor_get_max_position(device))
        wb_motor_set_position(device, INFINITY);
    } else if (device_type == WB_NODE_LED) {
      printf("- LED: %s\n", wb_device_get_name(device));
      // Choose a random color.
      wb_led_set(device, led_colors[rand_int(0, N_LED_COLORS)]);
    }
  }

  // Choose 3 global random numbers.
  const double a = rand_float(-1.0, 1.0);
  const double b = rand_float(-1.0, 1.0);
  const double c = rand_float(0.5, 2.0);

  // Loop until the simulator stops the controller.
  while (wb_robot_step(timestep) != -1) {
    const double current_time = wb_robot_get_time();

    srand(seed);  // Restart the seed at each step in order to have the same random numbers for each motor.

    // Actuate each motor.
    for (d = 0; d < n_motors; ++d) {
      WbDeviceTag motor = motors[d];

      // Choose 3 random numbers for this motor.
      const double m_a = rand_float(-1.0, 1.0);
      const double m_b = rand_float(-1.0, 1.0);
      const double m_c = rand_float(0.5, 2.0);
      const double m_d = rand_float(0.25, 0.75);

      if (wb_motor_get_min_position(motor) == wb_motor_get_max_position(motor)) {
        // Unbounded motors: velocity control.
        const double v = m_d * wb_motor_get_max_velocity(motor);
        wb_motor_set_velocity(motor, v);
      } else if (strstr(wb_device_get_name(motor), "finger") != NULL) {
        // Grabber special case: move the motors together.
        double p = 0.5 * (sin(current_time * c + b) * a) + 0.5;
        p = p * (wb_motor_get_max_position(motor) - wb_motor_get_min_position(motor)) + wb_motor_get_min_position(motor);
        wb_motor_set_position(motor, p);
      } else {
        // else: position control.
        double p = 0.5 * (sin(current_time * m_c + m_b) * m_a) + 0.5;
        p = p * (wb_motor_get_max_position(motor) - wb_motor_get_min_position(motor)) + wb_motor_get_min_position(motor);
        wb_motor_set_position(motor, p);
      }
    }
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
