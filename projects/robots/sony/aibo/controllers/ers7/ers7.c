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

//
// Description: A controller for the Aibo ERS-7 robot which will first
//              play back an MTN file and then release the control.
//

#include <stdio.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include "../../libraries/mtn/mtn.h"

#define TIME_STEP 64
#define MTN_REPLAY 5

int main() {
  wb_robot_init();

  int loop = -1;

  // get devices
  WbDeviceTag camera = wb_robot_get_device("PRM:/r1/c1/c2/c3/i1-FbkImageSensor:F1");
  WbDeviceTag head_distance_near = wb_robot_get_device("PRM:/r1/c1/c2/c3/p1-Sensor:p1");
  WbDeviceTag head_distance_far = wb_robot_get_device("PRM:/r1/c1/c2/c3/p2-Sensor:p2");
  WbDeviceTag chest_distance_sensor = wb_robot_get_device("PRM:/p1-Sensor:p1");
  WbDeviceTag touch_sensor_fore_l = wb_robot_get_device("PRM:/r2/c1/c2/c3/c4-Sensor:24");
  WbDeviceTag touch_sensor_hind_l = wb_robot_get_device("PRM:/r3/c1/c2/c3/c4-Sensor:34");
  WbDeviceTag touch_sensor_fore_r = wb_robot_get_device("PRM:/r4/c1/c2/c3/c4-Sensor:44");
  WbDeviceTag touch_sensor_hind_r = wb_robot_get_device("PRM:/r5/c1/c2/c3/c4-Sensor:54");

  // enable camera and sensors
  wb_camera_enable(camera, TIME_STEP);
  wb_distance_sensor_enable(head_distance_near, TIME_STEP);
  wb_distance_sensor_enable(head_distance_far, TIME_STEP);
  wb_distance_sensor_enable(chest_distance_sensor, TIME_STEP);
  wb_touch_sensor_enable(touch_sensor_fore_l, TIME_STEP);
  wb_touch_sensor_enable(touch_sensor_hind_l, TIME_STEP);
  wb_touch_sensor_enable(touch_sensor_fore_r, TIME_STEP);
  wb_touch_sensor_enable(touch_sensor_hind_r, TIME_STEP);

  // read MTN motion sequence
  MTN *mtn = mtn_new("../../data/mtn/WWFWD.MTN");
  if (!mtn)
    fprintf(stderr, "MTN Error: %s\n", mtn_get_error());

  while (wb_robot_step(TIME_STEP) != -1) {
    // actuate MTN Motor motors
    mtn_step(TIME_STEP);

    // play mtn until enough loops
    if (mtn_is_over(mtn) && loop < MTN_REPLAY) {
      mtn_play(mtn);
      loop++;
    }
  }

  wb_robot_cleanup();
  mtn_delete(mtn);

  return 0;
}
