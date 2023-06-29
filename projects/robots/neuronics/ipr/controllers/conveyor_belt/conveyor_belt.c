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
 * Description:  The controller moves the conveyor belt and
 *               changes the color of the object detectors.
 */

#include <stdio.h>
#include <stdlib.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 100

static WbDeviceTag belt_motor;

#define OBJECT_DETECTOR_NUMBER 2
typedef struct {
  WbDeviceTag led;
  WbDeviceTag sensor;
  /*
   * This is used to avoid changing the color of the led to the same color as
   * the current one.
   */
  bool detecting;
} ObjectDetector;
static ObjectDetector objectDetectors[OBJECT_DETECTOR_NUMBER];

static void initialize() {
  /* Intialize Webots */
  wb_robot_init();

  belt_motor = wb_robot_get_device("belt_motor");
  wb_motor_set_position(belt_motor, INFINITY);

  int i;
  for (i = 0; i < OBJECT_DETECTOR_NUMBER; ++i) {
    char device_name[32];
    sprintf(device_name, "object_detector_sensor%d", i + 1);
    objectDetectors[i].sensor = wb_robot_get_device(device_name);
    if (objectDetectors[i].sensor != 0)
      wb_distance_sensor_enable(objectDetectors[i].sensor, TIME_STEP);
    else
      printf("Error: Object detector %d sensor not found\n", i + 1);

    sprintf(device_name, "object_detector_led%d", i + 1);
    objectDetectors[i].led = wb_robot_get_device(device_name);
    if (objectDetectors[i].led != 0)
      wb_led_set(objectDetectors[i].led, 1);
    else
      printf("Error: Object detector %d led not found\n", i + 1);
    objectDetectors[i].detecting = false;
  }

  /* We set the belt velocity once for all. */
  if (belt_motor != 0)
    wb_motor_set_velocity(belt_motor, 0.15);
  else {
    printf("Fatal Error: Motor needed for the conveyor belt not found\n");
    exit(0);
  }
}

static void run() {
  /*
   * If we detect something we change the color of the object detector,
   * if we did not change it already.
   */
  int i;
  for (i = 0; i < OBJECT_DETECTOR_NUMBER; ++i) {
    if (objectDetectors[i].sensor != 0 && objectDetectors[i].led != 0) {
      const double sensor_value = wb_distance_sensor_get_value(objectDetectors[i].sensor);
      if (sensor_value < 600 && !objectDetectors[i].detecting) {
        wb_led_set(objectDetectors[i].led, 2);
        objectDetectors[i].detecting = true;
      } else if (sensor_value > 900 && objectDetectors[i].detecting) {
        wb_led_set(objectDetectors[i].led, 1);
        objectDetectors[i].detecting = false;
      }
    }
  }
}

int main() {
  initialize();
  while (wb_robot_step(TIME_STEP) != -1)
    run();
  wb_robot_cleanup();
  return 0;
}
