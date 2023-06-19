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

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/plugins/robot_window/robot_wwi.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>

static bool stop_motors = false;
static WbDeviceTag left_motor = 0;
static WbDeviceTag right_motor = 0;
static WbDeviceTag ps0 = 0;

// Window initialization: get some robot devices.
void wb_robot_window_init() {
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  ps0 = wb_robot_get_device("ps0");
}

// A simulation step occurred.
void wb_robot_window_step(int time_step) {
  // Window initialization: get some robot devices.
  const char *message;
  while ((message = wb_robot_wwi_receive_text())) {
    if (strcmp(message, "stop motors") == 0) {
      // Stop the motors.
      printf("Received 'stop motors' message from JavaScript\n");
      stop_motors = true;
    } else if (strcmp(message, "release motors") == 0) {
      // Release the command which stops the motors.
      printf("Received 'release motors' message from JavaScript\n");
      stop_motors = false;
    } else
      // This should not occur.
      fprintf(stderr, "Unkown message: '%s'\n", message);
  }

  // Actually stop the motors.
  if (stop_motors) {
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
  }

  // At each step, send the "ps0" distance sensor value to JavaScript.
  double ps0_value = wb_distance_sensor_get_value(ps0);
  if (ps0_value > 0 && !isnan(ps0_value)) {
    char answer[0x100];
    sprintf(answer, "ps0: %f", ps0_value);
    wb_robot_wwi_send_text(answer);
  }
}

void wb_robot_window_cleanup() {
  // This is called when the robot window is destroyed.
  // There is nothing to do here in this example.
  // This callback can be used to store information.
}
