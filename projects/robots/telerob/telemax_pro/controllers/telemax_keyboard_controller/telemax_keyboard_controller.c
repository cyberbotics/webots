/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdio.h>
#include <string.h>

#define TIME_STEP 32
#define WHEELS_TO_TRACK_RATION 0.155  // radius of the wheels

void set_motor_position_in_limit(WbDeviceTag motor, double *postion) {
  double max = wb_motor_get_max_position(motor);
  double min = wb_motor_get_min_position(motor);
  if (max != 0.0 || min != 0.0) {
    if (*postion > max)
      *postion = max;
    else if (*postion < min)
      *postion = min;
  }
  wb_motor_set_position(motor, *postion);
}

int main(int argc, char **argv) {
  wb_robot_init();

  wb_keyboard_enable(TIME_STEP);

  WbDeviceTag front_motor = wb_robot_get_device("front motor");
  WbDeviceTag rear_motor = wb_robot_get_device("rear motor");
  WbDeviceTag front_left_track = wb_robot_get_device("left front track");
  WbDeviceTag front_right_track = wb_robot_get_device("right front track");
  WbDeviceTag rear_left_track = wb_robot_get_device("left rear track");
  WbDeviceTag rear_right_track = wb_robot_get_device("right rear track");

  wb_motor_set_position(front_left_track, INFINITY);
  wb_motor_set_position(front_right_track, INFINITY);
  wb_motor_set_position(rear_left_track, INFINITY);
  wb_motor_set_position(rear_right_track, INFINITY);
  wb_motor_set_velocity(front_left_track, 0.0);
  wb_motor_set_velocity(front_right_track, 0.0);
  wb_motor_set_velocity(rear_left_track, 0.0);
  wb_motor_set_velocity(rear_right_track, 0.0);

  // check if the wheel motor are present
  WbDeviceTag front_left_motor = (WbDeviceTag)0;
  WbDeviceTag front_right_motor = (WbDeviceTag)0;
  WbDeviceTag rear_left_motor = (WbDeviceTag)0;
  WbDeviceTag rear_right_motor = (WbDeviceTag)0;
  WbDeviceTag led = (WbDeviceTag)0;
  int i = 0;
  int nuber_of_devices = wb_robot_get_number_of_devices();
  for (i = 0; i < nuber_of_devices; i++) {
    WbDeviceTag tag = wb_robot_get_device_by_index(i);
    const char *name = wb_device_get_name(tag);
    WbNodeType type = wb_device_get_node_type(tag);
    if (type == WB_NODE_ROTATIONAL_MOTOR) {
      if (strcmp(name, "left front wheel") == 0) {
        front_left_motor = tag;
        wb_motor_set_position(tag, INFINITY);
        wb_motor_set_velocity(tag, 0.0);
      } else if (strcmp(name, "right front wheel") == 0) {
        front_right_motor = tag;
        wb_motor_set_position(tag, INFINITY);
        wb_motor_set_velocity(tag, 0.0);
      } else if (strcmp(name, "left rear wheel") == 0) {
        rear_left_motor = tag;
        wb_motor_set_position(tag, INFINITY);
        wb_motor_set_velocity(tag, 0.0);
      } else if (strcmp(name, "right rear wheel") == 0) {
        rear_right_motor = tag;
        wb_motor_set_position(tag, INFINITY);
        wb_motor_set_velocity(tag, 0.0);
      }
    } else if (type == WB_NODE_LED) {
      if (strcmp(name, "camera 0 led") == 0)
        led = tag;
    }
  }

  WbDeviceTag arm_motors[9];
  double arm_position[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  char buffer[32];
  for (i = 0; i < 7; ++i) {
    sprintf(buffer, "arm %d motor", i);
    arm_motors[i] = wb_robot_get_device(buffer);
  }
  arm_motors[7] = wb_robot_get_device("left gripper motor");
  arm_motors[8] = wb_robot_get_device("right gripper motor");

  WbDeviceTag central_led = wb_robot_get_device("central led");
  WbDeviceTag left_led = wb_robot_get_device("left led");
  WbDeviceTag right_led = wb_robot_get_device("right led");

  WbDeviceTag camera = wb_robot_get_device("gripper camera");

  double position[2] = {0.0, 0.0};
  double speed = 0.0;
  double angle = 0.0;
  bool led_key_pressed[3] = {false, false, false};
  bool camera_key_pressed = false;
  int led_value = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    int key = wb_keyboard_get_key();
    bool current_led_key_pressed[3] = {false, false, false};
    bool current_camera_key_pressed = false;
    while (key != -1) {
      switch (key) {
        case WB_KEYBOARD_HOME:
          position[0] += 0.01;
          break;
        case WB_KEYBOARD_END:
          position[0] -= 0.01;
          break;
        case WB_KEYBOARD_PAGEUP:
          position[1] += 0.01;
          break;
        case WB_KEYBOARD_PAGEDOWN:
          position[1] -= 0.01;
          break;
        case WB_KEYBOARD_UP:
          speed -= 0.005;
          break;
        case WB_KEYBOARD_DOWN:
          speed += 0.005;
          break;
        case WB_KEYBOARD_RIGHT:
          angle -= 0.002;
          break;
        case WB_KEYBOARD_LEFT:
          angle += 0.002;
          break;
        case '1':
        case 'Q':
          arm_position[0] += key == '1' ? 0.01 : -0.01;
          set_motor_position_in_limit(arm_motors[0], &arm_position[0]);
          break;
        case '2':
        case 'W':
          arm_position[1] += key == '2' ? 0.01 : -0.01;
          set_motor_position_in_limit(arm_motors[1], &arm_position[1]);
          break;
        case '3':
        case 'E':
          arm_position[2] += key == '3' ? 0.002 : -0.002;
          set_motor_position_in_limit(arm_motors[2], &arm_position[2]);
          break;
        case '4':
        case 'R':
          arm_position[3] += key == '4' ? 0.01 : -0.01;
          set_motor_position_in_limit(arm_motors[3], &arm_position[3]);
          break;
        case '5':
        case 'T':
          arm_position[4] += key == '5' ? 0.01 : -0.01;
          set_motor_position_in_limit(arm_motors[4], &arm_position[4]);
          break;
        case '6':
        case 'Z':
          arm_position[5] += key == '6' ? 0.01 : -0.01;
          set_motor_position_in_limit(arm_motors[5], &arm_position[5]);
          break;
        case '7':
        case 'U':
          arm_position[6] += key == '7' ? 0.01 : -0.01;
          set_motor_position_in_limit(arm_motors[6], &arm_position[6]);
          break;
        case '8':
        case 'I':
          arm_position[7] += key == '8' ? 0.0002 : -0.0002;
          arm_position[8] += key == '8' ? 0.0002 : -0.0002;
          set_motor_position_in_limit(arm_motors[7], &arm_position[7]);
          set_motor_position_in_limit(arm_motors[8], &arm_position[8]);
          break;
        case 'B':
          current_led_key_pressed[0] = true;
          if (!led_key_pressed[0])
            wb_led_set(left_led, !wb_led_get(left_led));
          break;
        case 'N':
          current_led_key_pressed[1] = true;
          if (!led_key_pressed[1])
            wb_led_set(central_led, !wb_led_get(central_led));
          break;
        case 'M':
          current_led_key_pressed[2] = true;
          if (!led_key_pressed[2])
            wb_led_set(right_led, !wb_led_get(right_led));
          break;
        case 'C':
          current_camera_key_pressed = true;
          if (!camera_key_pressed) {
            if (wb_camera_get_sampling_period(camera) > 0)
              wb_camera_disable(camera);
            else
              wb_camera_enable(camera, 32);
          }
          break;
        case 'X':
          if (led && led_value < 100) {
            led_value++;
            wb_led_set(led, led_value);
          }
          printf("%d\n", led_value);
          break;
        case 'Y':
          if (led && led_value > 0) {
            led_value--;
            wb_led_set(led, led_value);
          }
          printf("%d\n", led_value);
          break;
        default:
          printf("Unrecognized key: %d %d %d\n", key, '+', '+' + WB_KEYBOARD_SHIFT);
      }
      key = wb_keyboard_get_key();
    }

    led_key_pressed[0] = current_led_key_pressed[0];
    led_key_pressed[1] = current_led_key_pressed[1];
    led_key_pressed[2] = current_led_key_pressed[2];
    camera_key_pressed = current_camera_key_pressed;

    wb_motor_set_position(front_motor, position[0]);
    wb_motor_set_position(rear_motor, position[1]);
    wb_motor_set_velocity(front_left_track, speed + angle);
    if (front_left_motor)
      wb_motor_set_velocity(front_left_motor, (speed + angle) / WHEELS_TO_TRACK_RATION);
    wb_motor_set_velocity(front_right_track, speed - angle);
    if (front_right_motor)
      wb_motor_set_velocity(front_right_motor, (speed - angle) / WHEELS_TO_TRACK_RATION);
    wb_motor_set_velocity(rear_left_track, -speed - angle);
    if (rear_left_motor)
      wb_motor_set_velocity(rear_left_motor, (-speed - angle) / WHEELS_TO_TRACK_RATION);
    wb_motor_set_velocity(rear_right_track, -speed + angle);
    if (rear_right_motor)
      wb_motor_set_velocity(rear_right_motor, (-speed + angle) / WHEELS_TO_TRACK_RATION);
  };

  wb_robot_cleanup();

  return 0;
}
