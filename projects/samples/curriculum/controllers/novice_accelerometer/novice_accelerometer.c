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

// Included libraries
#include <math.h>  //for atan2f and sqrtf
#include <stdio.h>
#include <webots/accelerometer.h>  // the accelerometer header
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Global defines
#define LEFT 0                       // Left side
#define RIGHT 1                      // right side
#define TIME_STEP 32                 // [ms] // time step of the simulation
#define CST_RADIAN (180.0 / 3.1415)  // for converting a radian to a degree
#define SPEED_UNIT 0.00628

int speed[2] = {0, 0};

WbDeviceTag accelerometer;
const double *accelerometer_values;

// motors
WbDeviceTag left_motor, right_motor;

#define ON 1
#define OFF 0
#define NB_LEDS 10
WbDeviceTag led[NB_LEDS];

// get the norm (acceleration) having the three
// components of the acceleration vector (result in Newton)
double getAcceleration(double x, double y, double z) {
  return sqrtf(x * x + y * y + z * z);
}

// get the inclination of the acceleration vector in
// comparison to the vertical (result in degrees)
double getInclination(double x, double y, double z) {
  return 90.0 - atan2f(z, sqrtf(x * x + y * y)) * CST_RADIAN;
}

// get the orientation of the acceleration vector
// in comparison to the horizontal plan (result in degrees)
double getOrientation(double x, double y, double z) {
  if (getInclination(x, y, z) < 5)
    return 0.0f;
  else
    return (atan2f(x, y) * CST_RADIAN) + 180.0;
}

int main() {
  wb_robot_init();

  // get and enable the accelerometer
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, TIME_STEP);

  // get the leds
  char text[5] = "led0";
  int it;
  for (it = 0; it < NB_LEDS; it++) {
    led[it] = wb_robot_get_device(text);
    text[3]++;
  }

  // get a handler to the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    // get the values of the accelerometer
    accelerometer_values = wb_accelerometer_get_values(accelerometer);
    double fx = accelerometer_values[0];
    double fy = accelerometer_values[1];
    double fz = accelerometer_values[2];
    double acceleration = getAcceleration(fx, fy, fz);
    double inclination = getInclination(fx, fy, fz);
    double orientation = getOrientation(fx, fy, fz);

    // print them
    printf("x=%0.2f y=%0.2f z=%0.2f\n", fx, fy, fz);
    printf("a=%0.2f i=%0.2f o=%0.2f\n", acceleration, inclination, orientation);

    // switch off all the leds
    int i;
    for (i = 0; i < NB_LEDS; i++)
      wb_led_set(led[i], OFF);

    // switch on the lowest led
    /* ... */

    // switch on the body led when the e-puck falls
    /* ... */

    // the robot goes forward
    wb_motor_set_velocity(left_motor, SPEED_UNIT * 1000);
    wb_motor_set_velocity(right_motor, SPEED_UNIT * 1000);
  }

  wb_robot_cleanup();

  return 0;
}
