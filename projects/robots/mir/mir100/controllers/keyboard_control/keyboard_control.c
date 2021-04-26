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
 * Description:  This controller is used to move the mecanum-wheeled robot Fabtino in an industrial environment
 *               using the keyboard. The keys are the following:
 *
 *               vx: ↑/↓
 *               vy: ←/→
 *               ω: Page Up/Page Down
 *               Reset: Space bar
 */

#include <math.h>
#include <stdio.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define WHEEL_RADIUS 0.1
#define SPEED_INCREMENT 0.2
#define MAX_SPEED 2.0

int main() {
  wb_robot_init();

  WbDeviceTag motor_wheel_fl = wb_robot_get_device("mir100_front_left_wheel_joint");
  WbDeviceTag motor_wheel_fr = wb_robot_get_device("mir100_front_right_wheel_joint");
  WbDeviceTag motor_wheel_bl = wb_robot_get_device("mir100_back_left_wheel_joint");
  WbDeviceTag motor_wheel_br = wb_robot_get_device("mir100_back_right_wheel_joint");

  WbDeviceTag motor_caster_fl = wb_robot_get_device("mir100_front_left_caster_joint");
  WbDeviceTag motor_caster_fr = wb_robot_get_device("mir100_front_right_caster_joint");
  WbDeviceTag motor_caster_bl = wb_robot_get_device("mir100_back_left_caster_joint");
  WbDeviceTag motor_caster_br = wb_robot_get_device("mir100_back_right_caster_joint");

  // Velocity control, so position must be set to infinity.
  wb_motor_set_position(motor_wheel_fl, INFINITY);
  wb_motor_set_position(motor_wheel_fr, INFINITY);
  wb_motor_set_position(motor_wheel_bl, INFINITY);
  wb_motor_set_position(motor_wheel_br, INFINITY);

  wb_motor_set_velocity(motor_wheel_fl, 0.0);
  wb_motor_set_velocity(motor_wheel_fr, 0.0);
  wb_motor_set_velocity(motor_wheel_bl, 0.0);
  wb_motor_set_velocity(motor_wheel_br, 0.0);

  // Caster in position control
  wb_motor_set_position(motor_caster_fl, 0.0);
  wb_motor_set_position(motor_caster_fr, 0.0);
  wb_motor_set_position(motor_caster_bl, 0.0);
  wb_motor_set_position(motor_caster_br, 0.0);


  double target_commands[2] = {0.0, 0.0};  // vx [m/s], ω [rad/s].
  int command_id = 0;                              // index to select either vx or ω.
  int sign;                                      // sign of the increment (decrement if -1).
  //double motor_speed[4] = {0.0, 0.0, 0.0, 0.0};  // wheels speed in [m/s], computed from vx and ω.
  const double increments[2] = {0.2, M_PI/12};
  const double limits[2] = {2.0, M_PI/2};
  bool is_key_valid = 0;

  wb_keyboard_enable(TIME_STEP);
  int waiting_counter = 0;  // waiting counter (to avoid registering too much clicks when user long-clicks.

  printf("To move the Fabtino-XL Steel with your keyboard, click first inside the simulation window and press:\n \
    vx   : ↑/↓               \n \
    ω    : Page Up/Page Down \n \
    Reset: Space bar         \n");

  while (wb_robot_step(TIME_STEP) != -1) {
    if (waiting_counter == 0) {
      int key = wb_keyboard_get_key();

      switch (key) {
        case WB_KEYBOARD_UP:
          is_key_valid = 1;
          command_id = 0;
          sign = 1;
          break;

        case WB_KEYBOARD_DOWN:
          is_key_valid = 1;
          command_id = 0;
          sign = -1;
          break;

        case WB_KEYBOARD_PAGEUP:
          is_key_valid = 1;
          command_id = 1;
          sign = 1;
          break;

        case WB_KEYBOARD_PAGEDOWN:
          is_key_valid = 1;
          command_id = 1;
          sign = -1;
          break;

        case ' ':
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
          target_commands[command_id] += increments[command_id];
          if (target_commands[command_id] > limits[command_id])
            target_commands[command_id] = limits[command_id];
        } else if (sign < 0) {
          target_commands[command_id] -= increments[command_id];
          if (target_commands[command_id] < -limits[command_id])
            target_commands[command_id] = -limits[command_id];
        } else {
          for (int i = 0; i < 2; ++i)
            target_commands[i] = 0;
        }
        printf("vx:%.1f ω:%.1f\n", target_commands[0], target_commands[1]);
        waiting_counter = 10;

        // Computes the wheel motors speeds from vx and ω.

        wb_motor_set_velocity(motor_wheel_fl, target_commands[0] / WHEEL_RADIUS);
        wb_motor_set_velocity(motor_wheel_fr, target_commands[0] / WHEEL_RADIUS);
        wb_motor_set_velocity(motor_wheel_bl, target_commands[0] / WHEEL_RADIUS);
        wb_motor_set_velocity(motor_wheel_br, target_commands[0] / WHEEL_RADIUS);

        wb_motor_set_position(motor_caster_fl, -target_commands[1]);
        wb_motor_set_position(motor_caster_fr, -target_commands[1]);
        wb_motor_set_position(motor_caster_bl, target_commands[1]);
        wb_motor_set_position(motor_caster_br, target_commands[1]);
      }
    } else {
      waiting_counter -= 1;
    }
  }

  wb_robot_cleanup();
  return 0;
}
