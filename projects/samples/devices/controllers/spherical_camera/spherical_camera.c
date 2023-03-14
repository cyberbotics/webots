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
 * Description:  Simulation of a spherical camera
 */

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/utils/ansi_codes.h>

#include <math.h>
#include <stdio.h>

#define TIME_STEP 64
#define THRESHOLD 200

#define RED 0
#define GREEN 1
#define BLUE 2

#define LEFT 0
#define RIGHT 1

#define X 0
#define Y 1

double coord2D_to_angle(double x, double y) {
  if (x > 0.0 && y >= 0.0)
    return atan(y / x);
  else if (x > 0.0 && y < 0.0)
    return atan(y / x) + 2.0 * M_PI;
  else if (x < 0.0)
    return atan(y / x) + M_PI;
  else if (x == 0.0 && y > 0.0)
    return M_PI_2;
  else if (x == 0.0 && y < 0.0)
    return 3.0 * M_PI_2;
  else /* (x == 0.0 && y == 0.0) */
    return 0.0;
}

int main(int argc, char **argv) {
  // iterator used to parse loops
  int i, k;

  // init Webots stuff
  wb_robot_init();

  // init camera
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 2 * TIME_STEP);
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  int color_index[3][2] = {{0, 0}, {0, 0}, {0, 0}};
  int x, y, r, g, b;

  // init distance sensors
  WbDeviceTag us[2];
  double us_values[2];
  double coefficients[2][2] = {{6.0, -3.0}, {-5.0, 4.0}};
  us[LEFT] = wb_robot_get_device("us0");
  us[RIGHT] = wb_robot_get_device("us1");
  for (i = 0; i < 2; i++)
    wb_distance_sensor_enable(us[i], TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control)
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // init speed values
  double speed[2];

  while (wb_robot_step(TIME_STEP) != -1) {
    // read sensors
    const unsigned char *image = wb_camera_get_image(camera);
    for (i = 0; i < 2; i++)
      us_values[i] = wb_distance_sensor_get_value(us[i]);

    // compute speed
    for (i = 0; i < 2; i++) {
      speed[i] = 0.0;
      for (k = 0; k < 2; k++)
        speed[i] += us_values[k] * coefficients[i][k];
    }

    // compute blob direction
    for (y = 0; y < height; y++) {
      for (x = 0; x < width; x++) {
        r = wb_camera_image_get_red(image, width, x, y);
        g = wb_camera_image_get_green(image, width, x, y);
        b = wb_camera_image_get_blue(image, width, x, y);
        if (r > THRESHOLD && g < THRESHOLD && b < THRESHOLD) {
          color_index[RED][X] = x;
          color_index[RED][Y] = y;
        } else if (r < THRESHOLD && g > THRESHOLD && b < THRESHOLD) {
          color_index[GREEN][X] = x;
          color_index[GREEN][Y] = y;
        } else if (r < THRESHOLD && g < THRESHOLD && b > THRESHOLD) {
          color_index[BLUE][X] = x;
          color_index[BLUE][Y] = y;
        }
      }
    }

    // print results
    ANSI_CLEAR_CONSOLE();
    for (i = 0; i < 3; i++)
      // clang-format off
      // clang-format 11.0.0 is not compatible with previous versions with respect to nested conditional operators
      printf("last %s blob seen at (%d,%d) with an angle of %f\n",
             (i == GREEN) ? "Green" :
             (i == RED)   ? "Red" :
                            "Blue",
             color_index[i][X], color_index[i][Y],
             coord2D_to_angle((double)(color_index[i][X] + width / 2), (double)(color_index[i][Y] + height / 2)));
    // clang-format on

    // set actuators
    wb_motor_set_velocity(left_motor, 3.0 + speed[LEFT]);
    wb_motor_set_velocity(right_motor, 3.0 + speed[RIGHT]);
  }

  wb_robot_cleanup();

  return 0;
}
