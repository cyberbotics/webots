/*
 * Copyright 1996-2020 Cyberbotics Ltd.
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

#include <webots/robot.h>
#include <webots/motor.h>

#include <stdio.h>

#define MOTOR_NUMBER 8

static WbDeviceTag motors[MOTOR_NUMBER];

static const char *motor_names[MOTOR_NUMBER] = {
  "1", "2", "3", "4", "5", "6", "7", "7 left"
};

void open_gripper() {
  wb_motor_set_position(motors[6], 0.5);
  wb_motor_set_position(motors[7], 0.5);
}

void close_gripper() {
  wb_motor_set_torque(motors[6], -0.2);
  wb_motor_set_torque(motors[7], -0.2);
}


int main(int argc, char **argv) {
  wb_robot_init();
  const int timestep = wb_robot_get_basic_time_step();

  for (int i = 0; i < MOTOR_NUMBER; ++i)
    motors[i] = wb_robot_get_device(motor_names[i]);

  while(true) {
    wb_motor_set_position(motors[4], -1.95);
    wb_robot_step(500);
  
  
    wb_motor_set_position(motors[1], 1.55);
    wb_motor_set_position(motors[2], 1.12);
    open_gripper();
    wb_robot_step(2000);
    printf("A\n");
    
    
    wb_motor_set_position(motors[4], -1.09);
   
  
    wb_robot_step(2000);
    printf("B\n");
    close_gripper();
  
    wb_robot_step(500);
    printf("C\n");
    wb_motor_set_position(motors[1], -0.92);
    wb_robot_step(300);
    wb_motor_set_position(motors[2], 1.88);
    wb_motor_set_position(motors[4], 1.5);
    wb_robot_step(2000);
    printf("D\n");
    wb_motor_set_position(motors[0], -1.5708);
    wb_robot_step(1000);
  
    wb_motor_set_position(motors[4], -1.04);
    wb_robot_step(1000);
    printf("E\n");
    wb_motor_set_position(motors[2], 1.12);
    wb_motor_set_position(motors[1], 1.53);
    wb_robot_step(2000);
    printf("F\n");
    open_gripper();
    wb_robot_step(500);
    printf("G\n");
    wb_motor_set_position(motors[4], -1.95);
    wb_robot_step(1000);
    printf("H\n");
    wb_motor_set_position(motors[1], 0.0);
    wb_robot_step(2000);
    printf("I\n");
    wb_motor_set_position(motors[0], 0.0);
    wb_robot_step(2000);
  };

  wb_robot_cleanup();

  return 0;
}
