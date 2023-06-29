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
 * Description:  Boomer tractor example
 */

#include <stdio.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

// to be used as array indices
enum { X, Y, Z };

// This needs to be changed if the .wbt model changes
#define FRONT_WHEEL_RADIUS 0.38
#define REAR_WHEEL_RADIUS 0.6

// devices
WbDeviceTag left_steer, right_steer;
WbDeviceTag left_front_wheel, right_front_wheel;
WbDeviceTag left_rear_wheel, right_rear_wheel;

// lights
WbDeviceTag left_flasher, right_flasher, tail_lights;
WbDeviceTag work_head_lights, road_head_lights;

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// SICK laser
WbDeviceTag sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

// GPS
WbDeviceTag gps;
double gps_coords[3];
double gps_speed = 0.0;

// misc variables
int time_step = -1;
double speed = 0.0;
double steering_angle = 0.0;
double manual_steering = 0.0;

void blink_lights() {
  int on = (int)wb_robot_get_time() % 2;
  wb_led_set(left_flasher, on);
  wb_led_set(right_flasher, on);
  wb_led_set(tail_lights, on);
  wb_led_set(work_head_lights, on);
  wb_led_set(road_head_lights, on);
}

void print_help() {
  printf("You can drive this vehicle!\n");
  printf("Select the 3D window and then use the cursor keys to:\n");
  printf("[LEFT]/[RIGHT] - steer\n");
  printf("[UP]/[DOWN] - accelerate/slow down\n");
}

// set target speed
void set_speed(double kmh) {
  if (kmh > 30.0)
    kmh = 30.0;

  speed = kmh;

  printf("setting speed to %g km/h\n", kmh);

  double front_ang_vel = kmh * 1000.0 / 3600.0 / FRONT_WHEEL_RADIUS;
  double rear_ang_vel = kmh * 1000.0 / 3600.0 / REAR_WHEEL_RADIUS;

  // set motor rotation speed
  wb_motor_set_velocity(left_front_wheel, front_ang_vel);
  wb_motor_set_velocity(right_front_wheel, front_ang_vel);
  wb_motor_set_velocity(left_rear_wheel, rear_ang_vel);
  wb_motor_set_velocity(right_rear_wheel, rear_ang_vel);
}

// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle) {
  steering_angle = wheel_angle;
  wb_motor_set_position(left_steer, steering_angle);
  wb_motor_set_position(right_steer, steering_angle);
}

void change_manual_steer_angle(double inc) {
  double new_manual_steering = manual_steering + inc;
  printf("steer %f\n", new_manual_steering);
  if (new_manual_steering <= 0.94 && new_manual_steering >= -0.94) {
    manual_steering = new_manual_steering;
    set_steering_angle(manual_steering);
  }

  if (manual_steering == 0)
    printf("going straight\n");
  else
    printf("turning %.2f rad (%s)\n", steering_angle, steering_angle < 0 ? "left" : "right");
}

void check_keyboard() {
  int key = wb_keyboard_get_key();
  switch (key) {
    case WB_KEYBOARD_UP:
      set_speed(speed + 1.0);
      break;
    case WB_KEYBOARD_DOWN:
      set_speed(speed - 1.0);
      break;
    case ' ':
      set_speed(0.0);
      break;
    case WB_KEYBOARD_RIGHT:
      change_manual_steer_angle(+0.02);
      break;
    case WB_KEYBOARD_LEFT:
      change_manual_steer_angle(-0.02);
      break;
  }
}

void compute_gps_speed() {
  const double *coords = wb_gps_get_values(gps);
  const double vel[3] = {coords[X] - gps_coords[X], coords[Y] - gps_coords[Y], coords[Z] - gps_coords[Z]};
  const double dist = sqrt(vel[X] * vel[X] + vel[Y] * vel[Y] + vel[Z] * vel[Z]);

  // store into global variables
  gps_speed = dist / time_step * 3600.0;
  memcpy(gps_coords, coords, sizeof(gps_coords));

  // printf("gps speed: %g km/h\n", gps_speed);
}

int main(int argc, char **argv) {
  wb_robot_init();

  time_step = (int)wb_robot_get_basic_time_step();

  // find wheels
  left_front_wheel = wb_robot_get_device("left_front_wheel");
  right_front_wheel = wb_robot_get_device("right_front_wheel");
  left_rear_wheel = wb_robot_get_device("left_rear_wheel");
  right_rear_wheel = wb_robot_get_device("right_rear_wheel");
  wb_motor_set_position(left_front_wheel, INFINITY);
  wb_motor_set_position(right_front_wheel, INFINITY);
  wb_motor_set_position(left_rear_wheel, INFINITY);
  wb_motor_set_position(right_rear_wheel, INFINITY);

  // get steering motors
  left_steer = wb_robot_get_device("left_steer");
  right_steer = wb_robot_get_device("right_steer");

  // camera device
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);
  camera_width = wb_camera_get_width(camera);
  camera_height = wb_camera_get_height(camera);
  camera_fov = wb_camera_get_fov(camera);

  // SICK sensor
  sick = wb_robot_get_device("Sick LMS 291");
  wb_lidar_enable(sick, time_step);
  sick_width = wb_lidar_get_horizontal_resolution(sick);
  sick_range = wb_lidar_get_max_range(sick);
  sick_fov = wb_lidar_get_fov(sick);

  // initialize gps
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, time_step);

  // find lights
  left_flasher = wb_robot_get_device("left_flasher");
  right_flasher = wb_robot_get_device("right_flasher");
  tail_lights = wb_robot_get_device("tail_lights");
  work_head_lights = wb_robot_get_device("work_head_lights");
  road_head_lights = wb_robot_get_device("road_head_lights");

  // start engine
  set_speed(10.0);  // km/h

  print_help();

  // allow to switch to manual control
  wb_keyboard_enable(time_step);

  // main loop
  while (wb_robot_step(time_step) != -1) {
    // get user input
    check_keyboard();

    // read sensors
    // const unsigned char *camera_image = wb_camera_get_image(camera);
    // const float *sick_data = wb_lidar_get_range_image(sick);

    // update stuff
    compute_gps_speed();
    blink_lights();
  }

  wb_robot_cleanup();

  return 0;  // ignored
}
