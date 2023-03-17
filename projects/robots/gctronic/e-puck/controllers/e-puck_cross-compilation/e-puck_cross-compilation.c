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

#include <stdio.h>
#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 64
#define ON 1
#define OFF 0
#define NB_LEDS 10
#define NB_LIGHT_SENS 8
#define NB_DIST_SENS 8

int main(int argc, char *argv[]) {
  WbDeviceTag accelerometer;
  WbDeviceTag led[NB_LEDS];
  WbDeviceTag ps[NB_DIST_SENS];
  WbDeviceTag ls[NB_LIGHT_SENS];
  WbDeviceTag cam;
  WbDeviceTag left_motor, right_motor;
  WbDeviceTag left_position_sensor, right_position_sensor;

  short direction = 1;
  int it, m, n;
  int camera_width, camera_height;
  double position_sensor_offest[2] = {0, 0};

  /* initialize Webots */
  wb_robot_init();

  /* get and enable devices */
  char text[5] = "led0";
  for (it = 0; it < NB_LEDS; it++) {
    led[it] = wb_robot_get_device(text);
    text[3]++;
    wb_led_set(led[it], OFF);
  }
  char textPS[] = "ps0";
  for (it = 0; it < NB_DIST_SENS; it++) {
    ps[it] = wb_robot_get_device(textPS);
    textPS[2]++;
    wb_distance_sensor_enable(ps[it], 2 * TIME_STEP);
  }

  char textLS[] = "ls0";
  for (it = 0; it < NB_LIGHT_SENS; it++) {
    ls[it] = wb_robot_get_device(textLS);
    textLS[2]++;
    wb_light_sensor_enable(ls[it], 2 * TIME_STEP);
  }
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, TIME_STEP);
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, 4 * TIME_STEP);
  camera_width = wb_camera_get_width(cam);
  camera_height = wb_camera_get_height(cam);

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    long int sum = 0;
    double count = 0.0;

    const unsigned char *im = wb_camera_get_image(cam);
    for (m = 0; m < camera_width; m++) {
      for (n = 0; n < camera_height; n++)
        sum += wb_camera_image_get_gray(im, camera_width, m, n);
    }

    const double *a = wb_accelerometer_get_values(accelerometer);

    for (it = 0; it < NB_DIST_SENS; it++)
      count += wb_distance_sensor_get_value(ps[it]);

    for (it = 0; it < NB_LEDS; it++)
      wb_led_set(led[it], OFF);

    if (wb_robot_get_mode() == 1)
      wb_led_set(led[0], 1);

    if (a[0] <= 0.0 && a[1] <= 0.0)
      wb_led_set(led[1], 1);
    if (a[0] <= 0.0 && a[1] > 0.0)
      wb_led_set(led[3], 1);
    if (a[0] > 0.0 && a[1] > 0.0)
      wb_led_set(led[5], 1);
    if (a[0] > 0.0 && a[1] <= 0.0)
      wb_led_set(led[7], 1);

    if (count > 7000)
      wb_led_set(led[8], 1);
    if (sum < 100000)
      wb_led_set(led[9], 1);

    if ((wb_position_sensor_get_value(left_position_sensor) - position_sensor_offest[0]) > (2 * M_PI) ||
        (wb_position_sensor_get_value(left_position_sensor) - position_sensor_offest[0]) < -(2 * M_PI)) {
      if (direction == 1)
        direction = -1;
      else
        direction = 1;
      position_sensor_offest[0] = wb_position_sensor_get_value(left_position_sensor);
      printf("Other direction\n");
    }

    wb_motor_set_velocity(left_motor, 1.88 * direction);
    wb_motor_set_velocity(right_motor, -1.88 * direction);
  }

  wb_robot_cleanup();

  return 0;
}
