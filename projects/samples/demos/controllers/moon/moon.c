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
 * Description:   A controller for the Koala robot which makes it turn on place
 */

#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64

int main() {
  WbDeviceTag ir[16];
  WbDeviceTag left_motor, right_motor;
  int manual_control = 0;
  int i, j;
  const char *robot_name;
  double speed[2];
  double sensor_value[16];
  double kUnit = 8.24;
  double matrix[2][16] = {{3, -18, -15, -15, -5, -5, 4, 4, 6, 6, 4, 4, 4, 4, 3, -18},
                          {-18, 3, 4, 4, 4, 4, 6, 6, 4, 4, -5, -5, -15, -15, -18, 3}};

  wb_robot_init();
  robot_name = wb_robot_get_name();

  if (strncmp(robot_name, "koala", 5) == 0) {
    for (i = 0; i < 16; i++) {
      char sensor_name[16];
      sprintf(sensor_name, "ds%d", i);
      ir[i] = wb_robot_get_device(sensor_name);
    }
  }

  for (i = 0; i < 16; i++)
    wb_distance_sensor_enable(ir[i], TIME_STEP);

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  wb_keyboard_enable(TIME_STEP);
  printf("You can drive this robot by using the arrow keys of your keyboard\n");

  while (wb_robot_step(TIME_STEP) != -1) {
    for (i = 0; i < 16; i++)
      sensor_value[i] = wb_distance_sensor_get_value(ir[i]);

    for (i = 0; i < 2; i++) {
      speed[i] = 80;
      for (j = 0; j < 16; j++)
        speed[i] += kUnit * matrix[i][j] * sensor_value[j] / 200.0;
      speed[i] /= 50;
    }

    const int key = wb_keyboard_get_key();

    if ((key >= 0) && !manual_control)
      manual_control = 1;

    if (manual_control) {
      switch (key) {
        case WB_KEYBOARD_DOWN:
          speed[0] = -1;
          speed[1] = -1;
          break;
        case WB_KEYBOARD_UP:
          speed[0] = 1;
          speed[1] = 1;
          break;
        case WB_KEYBOARD_LEFT:
          speed[0] = -1;
          speed[1] = 1;
          break;
        case WB_KEYBOARD_RIGHT:
          speed[0] = 1;
          speed[1] = -1;
          break;
        case 0:
          speed[0] = 0;
          speed[1] = 0;
      }
    }

    /* Set the motor speeds */
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }

  wb_robot_cleanup();

  return 0;
}
