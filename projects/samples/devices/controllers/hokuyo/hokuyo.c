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
 * Description:   Read the hokuyo sensor values and
 *                display them into Display devices
 */

#include <webots/display.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>

// duration in [ms] of a physics step
#define TIME_STEP 64

// colors used by the Display devices
#define FOREGROUND_COLOR 0xC030C0
#define AXIS_COLOR 0xA0A0A0
#define BLIND_SPOT_COLOR 0xC0C0C0
#define BACKGROUND_COLOR 0xFFFFFF

// max distance represented in the displayed circle
#define RADIUS 1.0

// function displaying lidar values on a display
static void display(WbDeviceTag d,   // display
                    int dw,          // display width
                    int dh,          // display height
                    const float *v,  // lidar values
                    int ns,          // number of samples
                    float fov        // field of view in radians
) {
  int dw2 = dw / 2;
  int dh2 = dh / 2;
  float fov2 = fov / 2;
  const int px[] = {dw2, dw2 + dw * cos(-fov2 - M_PI_2), dw2 + dw * cos(fov2 - M_PI_2)};
  const int py[] = {dh2, dh2 + dh * sin(-fov2 - M_PI_2), dh2 + dh * sin(fov2 - M_PI_2)};

  wb_display_set_color(d, BACKGROUND_COLOR);
  wb_display_fill_rectangle(d, 0, 0, dw, dh);
  wb_display_set_color(d, BLIND_SPOT_COLOR);
  wb_display_fill_polygon(d, px, py, 3);
  wb_display_set_color(d, AXIS_COLOR);
  wb_display_draw_polygon(d, px, py, 3);
  wb_display_draw_line(d, dw2, 0, dw2, dh);
  wb_display_draw_line(d, 0, dh2, dw, dh2);
  wb_display_draw_oval(d, dw2, dh2, dw2, dh2);
  wb_display_set_color(d, FOREGROUND_COLOR);

  int i;
  for (i = 0; i < ns; i++) {
    const float f = v[i];
    if (f != INFINITY && !isnan(f)) {
      const float alpha = -fov2 + fov * i / ns - M_PI_2;
      wb_display_draw_line(d, dw2, dh2, dw2 + f * cos(alpha) * dw2 / RADIUS, dh2 + f * sin(alpha) * dh2 / RADIUS);
    }
  }
}

// entry point function
int main(int argc, char **argv) {
  // init the Webots API
  wb_robot_init();

  // get the devices
  WbDeviceTag utm30lx = wb_robot_get_device("Hokuyo UTM-30LX");
  WbDeviceTag urg04lx = wb_robot_get_device("Hokuyo URG-04LX");

  WbDeviceTag display_a = wb_robot_get_device("displayA");
  WbDeviceTag display_b = wb_robot_get_device("displayB");

  // enable the devices
  wb_lidar_enable(utm30lx, TIME_STEP);
  wb_lidar_enable(urg04lx, TIME_STEP);

  // get the parameters of the devices
  int utm30lx_samples = wb_lidar_get_horizontal_resolution(utm30lx);
  int urg04lx_samples = wb_lidar_get_horizontal_resolution(urg04lx);

  double utm30lx_field_of_view = wb_lidar_get_fov(utm30lx);
  double urg04lx_field_of_view = wb_lidar_get_fov(urg04lx);

  int display_a_width = wb_display_get_width(display_a);
  int display_a_height = wb_display_get_height(display_a);
  int display_b_width = wb_display_get_width(display_b);
  int display_b_height = wb_display_get_height(display_b);

  // get a handler to the motors and set target position to infinity (speed control)
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // the robot turns round
  wb_motor_set_velocity(left_motor, 0.5);
  wb_motor_set_velocity(right_motor, -0.5);

  // main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // get the lidar values
    const float *utm30lx_values = wb_lidar_get_range_image(utm30lx);
    const float *urg04lx_values = wb_lidar_get_range_image(urg04lx);

    // display them on the Display devices
    display(display_a, display_a_width, display_a_height, utm30lx_values, utm30lx_samples, utm30lx_field_of_view);
    display(display_b, display_b_width, display_b_height, urg04lx_values, urg04lx_samples, urg04lx_field_of_view);
  }

  // cleanup
  wb_robot_cleanup();

  return 0;
}
