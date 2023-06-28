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
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <stdlib.h>

static int timestep;
static WbDeviceTag twister;
static WbDeviceTag pivot_a;
static WbDeviceTag pivot_b;
static WbDeviceTag finger_a;
static WbDeviceTag finger_b;
static WbDeviceTag finger_c;
static WbDeviceTag distance_sensor;

static void step() {
  if (wb_robot_step(timestep) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void wait_a_while(double time_in_seconds) {
  const double start_time = wb_robot_get_time();
  while (start_time + time_in_seconds > wb_robot_get_time())
    step();
}

static void actuate_motors(double t, double p_a, double p_b, double g) {
  wb_motor_set_position(twister, t);
  wb_motor_set_position(pivot_a, p_a);
  wb_motor_set_position(pivot_b, p_b);
  wb_motor_set_position(finger_a, g);
  wb_motor_set_position(finger_b, g);
  wb_motor_set_position(finger_c, g);
}

int main(int argc, char **argv) {
  wb_robot_init();

  timestep = (int)wb_robot_get_basic_time_step();

  // Get the motors.
  twister = wb_robot_get_device("twister");
  pivot_a = wb_robot_get_device("pivot A");
  pivot_b = wb_robot_get_device("pivot B");
  finger_a = wb_robot_get_device("grabber finger A");
  finger_b = wb_robot_get_device("grabber finger B");
  finger_c = wb_robot_get_device("grabber finger C");

  // Reduce their max velocity.
  const int maxSpeed = 2.0;
  wb_motor_set_velocity(twister, maxSpeed);
  wb_motor_set_velocity(pivot_a, maxSpeed);
  wb_motor_set_velocity(pivot_b, maxSpeed);
  wb_motor_set_velocity(finger_a, maxSpeed);
  wb_motor_set_velocity(finger_b, maxSpeed);
  wb_motor_set_velocity(finger_c, maxSpeed);

  // Get distance sensor and enable it.
  distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, timestep);

  // Set LED colors.
  wb_led_set(wb_robot_get_device("twister led"), 0xFF0000);
  wb_led_set(wb_robot_get_device("pivot A led"), 0x00FF00);
  wb_led_set(wb_robot_get_device("pivot B led"), 0x0000FF);
  wb_led_set(wb_robot_get_device("grabber led"), 0x00FFFF);
  wb_led_set(wb_robot_get_device("distance sensor led"), 0xFF00FF);

  while (true) {
    wait_a_while(0.5);
    // Grab the object.
    actuate_motors(-1.57, 0.0, 0.0, 1.3);
    wait_a_while(1.0);
    actuate_motors(-1.57, -0.4, -1.1, 1.3);
    wait_a_while(1.0);
    actuate_motors(-1.57, -0.4, -1.1, 0.15);
    wait_a_while(1.0);
    // Move the object over the conveyor belt and leave it.
    actuate_motors(1.57, 0.0, 0.0, 0.15);
    wait_a_while(2.0);
    actuate_motors(1.57, 0.0, 0.0, 1.3);
    wait_a_while(1.0);
    // Wait until the vehicle robot is detected.
    actuate_motors(0.0, 1.49, 1.57, 0.15);
    while (wb_distance_sensor_get_value(distance_sensor) > 100)
      step();
    wb_led_set(wb_robot_get_device("distance sensor led"), 0xFF0000);
    while (wb_distance_sensor_get_value(distance_sensor) < 100)
      step();
    wb_led_set(wb_robot_get_device("distance sensor led"), 0xFF00FF);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
