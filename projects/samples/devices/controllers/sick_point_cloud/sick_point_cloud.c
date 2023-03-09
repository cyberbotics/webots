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
 * Description:
 * This controller reads the lidar middle layer and use the z-axis of the point cloud to determine the number of objects in
 * front of the robot.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Define a threshold in meters from which the lidar stimuli is considered as an obstacle.
#define THRESHOLD 3.0

int main(int argc, char **argv) {
  wb_robot_init();

  // Define the refresh rate of this controller.
  const int time_step = 64;

  // Get and enable the devices.
  WbDeviceTag sick = wb_robot_get_device("sick");
  wb_lidar_enable(sick, time_step);
  wb_lidar_enable_point_cloud(sick);
  const int resolution = wb_lidar_get_horizontal_resolution(sick);
  const int layers = wb_lidar_get_number_of_layers(sick);

  // Main control loop.
  int previous_object_counter = 0;
  while (wb_robot_step(time_step) != -1) {
    int object_counter = 0;
    bool previous_obstacle = false;

    // For each point of the middle layer...
    const WbLidarPoint *layer = wb_lidar_get_layer_point_cloud(sick, layers / 2);
    int p;
    for (p = 0; p < resolution; ++p) {
      WbLidarPoint point = layer[p];

      // Determine if an obstacle is present or not.
      const bool obstacle = point.x < THRESHOLD;

      // Each time a new obstacle is detected, then increment the object counter.
      if (obstacle && !previous_obstacle)
        object_counter++;
      previous_obstacle = obstacle;
    }

    // Display the result on the Webots console (only when the result has changed).
    if (object_counter != previous_object_counter) {
      printf("I see %d can%c\n", object_counter, object_counter > 1 ? 's' : ' ');
      previous_object_counter = object_counter;
    }
  }

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
