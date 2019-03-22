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
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32

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

  WbDeviceTag central_led = wb_robot_get_device("central led");
  WbDeviceTag left_led = wb_robot_get_device("left led");
  WbDeviceTag right_led = wb_robot_get_device("right led");

  WbDeviceTag camera = wb_robot_get_device("gripper camera");

  double position[2] = {0.0, 0.0};
  double speed = 0.0;
  double angle = 0.0;
  bool led_key_pressed[3] = {false, false, false};
  bool camera_key_pressed = false;

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
        case 'I':
          current_led_key_pressed[0] = true;
          if (!led_key_pressed[0])
            wb_led_set(left_led, !wb_led_get(left_led));
          break;
        case 'O':
          current_led_key_pressed[1] = true;
          if (!led_key_pressed[1])
            wb_led_set(central_led, !wb_led_get(central_led));
          break;
        case 'P':
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
    wb_motor_set_velocity(front_right_track, speed - angle);
    wb_motor_set_velocity(rear_left_track, -speed - angle);
    wb_motor_set_velocity(rear_right_track, -speed + angle);
  };

  wb_robot_cleanup();

  return 0;
}
