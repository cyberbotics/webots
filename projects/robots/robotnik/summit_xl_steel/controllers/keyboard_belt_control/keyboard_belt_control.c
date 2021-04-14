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
* Description:  This controller is used to move the ConveyorPlatform of a Summit-XL Steel robot using the keyboard.
                The keys are the following:

                Belt: Page Up/Page Down
                STOP: S

                This controller is meant to be used alongside a Summit-XL Steel robot that runs the
                'keyboard-control' controller. The keys to move the robot itself are defined in the latter.
*/

#include <math.h>
#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>

#define TIME_STEP 32
#define SPEED_INCREMENT 0.1
#define MAX_SPEED 0.6

int main() {
  wb_robot_init();

  WbDeviceTag motor_belt = wb_robot_get_device("belt_motor");

  // Velocity control, so position must be set to infinity.
  wb_motor_set_position(motor_belt, INFINITY);

  double target_belt_speed = 0.0; // in [m/s].
  int speed_id = -1;
  int sign = 1;

  wb_keyboard_enable(TIME_STEP);
  int waiting_counter = 0; // Waiting counter (to avoid registering too much clicks when user long-clicks.

    void increase_target_speed() {
      if ( (speed_id != -1) || (sign == 0) ) {
        target_belt_speed += sign * SPEED_INCREMENT;
        if(sign > 0) {
          if (target_belt_speed > MAX_SPEED)
          target_belt_speed = MAX_SPEED;
        }
        else if(sign < 0){
          if (target_belt_speed < -MAX_SPEED)
          target_belt_speed = -MAX_SPEED;
        }
        else
        {
          target_belt_speed = 0.0;
        }
        printf("vbelt:%.1f\n", target_belt_speed);
        waiting_counter = 10;
      }
      speed_id = -1;
      sign = 1;
    }


    while (wb_robot_step(TIME_STEP) != -1) {
      if(waiting_counter == 0) {

        int key = wb_keyboard_get_key();

        switch (key) {

          case WB_KEYBOARD_PAGEUP:
          speed_id = 1;
          sign = 1;
          break;

          case WB_KEYBOARD_PAGEDOWN:
          speed_id = 1;
          sign = -1;
          break;

          case 'S':
          speed_id = -1;
          sign = 0;
          break;
        }
      }
      else {
        waiting_counter -= 1;
      }
      increase_target_speed();
      wb_motor_set_velocity(motor_belt, target_belt_speed);
    }

    wb_robot_cleanup();
    return 0;
  }
