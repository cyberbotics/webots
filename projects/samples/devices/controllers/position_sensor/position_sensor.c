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
 * Description:   An example of a controller using the position sensor.
 *                The system represents and inverted pendulum consisting of a pole
 *                conected to a robot by an hinge joint. The speed of the robot is
 *                adjusted based on the position of the pole in order to balance it.
 */

#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/utils/ansi_codes.h>

#include <stdio.h>

#define KP 35
#define KI 5
#define KD 28.9

int main(int argc, char **argv) {
  WbDeviceTag position_sensor, left_motor, right_motor;
  double time_step;
  double previous_position;
  double integral;

  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, time_step);

  // get a handler to the motors and set target position to infinity (speed control).
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  previous_position = 0;
  integral = 0.0;

  while (wb_robot_step(time_step) != -1) {
    const double position = wb_position_sensor_get_value(position_sensor);

    if (fabs(position) > 0.7)  // pole has fallen
      break;

    // PID control
    integral += (position + previous_position) * 0.5;
    const double derivative = (position - previous_position);
    double speed = KP * position + KI * integral + KD * derivative;

    // check maximum speed
    if (speed > 100)
      speed = 100;
    else if (speed < -100)
      speed = -100;

    wb_motor_set_velocity(left_motor, -speed);
    wb_motor_set_velocity(right_motor, -speed);
    ANSI_CLEAR_CONSOLE();
    printf("Position: %+f -> control force: %+f\n", position, speed);

    previous_position = position;
  };

  printf("Pole has fallen\n");
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_robot_step(time_step);

  wb_robot_cleanup();

  return 0;
}
