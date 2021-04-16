/*
 * Copyright 1996-2021 Cyberbotics Ltd.
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

/*
 * Description:  This controller is used to move the mecanum-wheeled robot Summit-XL Steel in an industrial environment
 *               using the keyboard. The keys are the following:
 *
 *               vx: ↑/↓
 *               vy: ←/→
 *               ω: +/-
 *               Belt*: Page Up/Page Down
 *               STOP: S
 *               *It is assumed that its Conveyor Platform runs the 'keyboard-belt-control' controller (these keys are
 *               handled only by the latter).
 */

#include <math.h>
#include <stdio.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define WHEEL_RADIUS 0.127
#define LX 0.215
#define LY 0.240
#define SPEED_INCREMENT 0.2
#define MAX_SPEED 2.0

int main() {
  wb_robot_init();

  WbDeviceTag motor_fl = wb_robot_get_device("summit_xl_front_left_wheel_joint");
  WbDeviceTag motor_fr = wb_robot_get_device("summit_xl_front_right_wheel_joint");
  WbDeviceTag motor_bl = wb_robot_get_device("summit_xl_back_left_wheel_joint");
  WbDeviceTag motor_br = wb_robot_get_device("summit_xl_back_right_wheel_joint");

  // Velocity control, so position must be set to infinity.
  wb_motor_set_position(motor_fl, INFINITY);
  wb_motor_set_position(motor_fr, INFINITY);
  wb_motor_set_position(motor_bl, INFINITY);
  wb_motor_set_position(motor_br, INFINITY);

  wb_motor_set_velocity(motor_fl, 0.0);
  wb_motor_set_velocity(motor_fr, 0.0);
  wb_motor_set_velocity(motor_bl, 0.0);
  wb_motor_set_velocity(motor_br, 0.0);

  double target_speed[3] = {0.0, 0.0, 0.0};      // vx [m/s], vy [m/s], ω [rad/s].
  int speed_id = 0;                              // index to select either vx, vy, ω.
  int sign;                                      // sign of the increment (decrement if -1).
  double motor_speed[4] = {0.0, 0.0, 0.0, 0.0};  // wheels speed in [m/s], computed from vx, vy and ω.
  bool is_key_valid = 0;

  wb_keyboard_enable(TIME_STEP);
  int waiting_counter = 0;  // waiting counter (to avoid registering too much clicks when user long-clicks.

  printf("To move the Summit-XL Steel with your keyboard, click first inside the simulation window and press:\n \
  vx: ↑/↓                 \n \
  vy: ←/→                 \n \
  ω: +/-                  \n \
  Belt: Page Up/Page Down \n \
  STOP: S                 \n");

  while (wb_robot_step(TIME_STEP) != -1) {
    if (waiting_counter == 0) {
      int key = wb_keyboard_get_key();

      switch (key) {
        case WB_KEYBOARD_UP:
          is_key_valid = 1;
          speed_id = 0;
          sign = 1;
          break;

        case WB_KEYBOARD_DOWN:
          is_key_valid = 1;
          speed_id = 0;
          sign = -1;
          break;

        case WB_KEYBOARD_LEFT:
          is_key_valid = 1;
          speed_id = 1;
          sign = 1;
          break;

        case WB_KEYBOARD_RIGHT:
          is_key_valid = 1;
          speed_id = 1;
          sign = -1;
          break;

        case '+':
          is_key_valid = 1;
          speed_id = 2;
          sign = 1;
          break;

        case '-':
          is_key_valid = 1;
          speed_id = 2;
          sign = -1;
          break;

        case 'S':
          is_key_valid = 1;
          sign = 0;
          break;

        default:
          is_key_valid = 0;
          sign = 0;
      }

      if (is_key_valid) {
        // Increase or decrease target speed, depending on the sign.
        if (sign > 0) {
          target_speed[speed_id] += SPEED_INCREMENT;
          if (target_speed[speed_id] > MAX_SPEED)
            target_speed[speed_id] = MAX_SPEED;
        }
        else if (sign < 0) {
          target_speed[speed_id] -= SPEED_INCREMENT;
          if (target_speed[speed_id] < -MAX_SPEED)
            target_speed[speed_id] = -MAX_SPEED;
        }
        else {
          for (int i = 0; i < 3; ++i)
            target_speed[i] = 0;
        }
        printf("vx:%.1f vy:%.1f ω:%.1f\n", target_speed[0], target_speed[1], target_speed[2]);
        waiting_counter = 10;

        // Computes the wheel motors speeds from vx, vy and ω.
        motor_speed[0] = 1 / WHEEL_RADIUS * (target_speed[0] - target_speed[1] - (LX + LY) * target_speed[2]);
        motor_speed[1] = 1 / WHEEL_RADIUS * (target_speed[0] + target_speed[1] + (LX + LY) * target_speed[2]);
        motor_speed[2] = 1 / WHEEL_RADIUS * (target_speed[0] + target_speed[1] - (LX + LY) * target_speed[2]);
        motor_speed[3] = 1 / WHEEL_RADIUS * (target_speed[0] - target_speed[1] + (LX + LY) * target_speed[2]);

        wb_motor_set_velocity(motor_fl, motor_speed[0]);
        wb_motor_set_velocity(motor_fr, motor_speed[1]);
        wb_motor_set_velocity(motor_bl, motor_speed[2]);
        wb_motor_set_velocity(motor_br, motor_speed[3]);
      }
    }
    else {
      waiting_counter -= 1;
    }
  }

  wb_robot_cleanup();
  return 0;
}
