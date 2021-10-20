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
 * Description:
 *   Turn the robot using its motors.
 *   Switch on the bottommost LED using the gravity force given by the Accelerometer feedback.
 */

#include <webots/accelerometer.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();
  int time_step = (int)wb_robot_get_basic_time_step();

  // Get the accelerometer and enable it.
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  // Get the LEDs.
  WbDeviceTag front_led = wb_robot_get_device("front led");
  WbDeviceTag back_led = wb_robot_get_device("back led");
  WbDeviceTag left_led = wb_robot_get_device("left led");
  WbDeviceTag right_led = wb_robot_get_device("right led");

  // Get the motors, and actuate them in velocity mode to make the robot turn.
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.5);
  wb_motor_set_velocity(right_motor, -0.5);

  while (wb_robot_step(time_step) != -1) {
    // Get the acceleration vector, which is close the gravity vector.
    const double *acceleration = wb_accelerometer_get_values(accelerometer);

    // Actuate the LEDs according to the acceleration vector.
    if (fabs(acceleration[1]) > fabs(acceleration[0])) {
      wb_led_set(front_led, false);
      wb_led_set(back_led, false);
      wb_led_set(left_led, acceleration[1] > 0.0);
      wb_led_set(right_led, acceleration[1] < 0.0);
    } else {
      wb_led_set(front_led, acceleration[0] < 0.0);
      wb_led_set(back_led, acceleration[0] > 0.0);
      wb_led_set(left_led, false);
      wb_led_set(right_led, false);
    }
  };

  wb_robot_cleanup();

  return 0;
}
