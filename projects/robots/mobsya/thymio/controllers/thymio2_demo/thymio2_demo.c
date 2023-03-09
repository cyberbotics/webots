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
 * Description:   A simple controller for the Thymio II robot
 */

#include <webots/accelerometer.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/plugins/robot_window/default.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include <math.h>
#include <stdio.h>

#define TIME_STEP 10

#define LED_FREQ 4.0

#define MAX_SPEED 4.0

#define LED_BUTTON_FORWARD 0
#define LED_BUTTON_RIGHT 1
#define LED_BUTTON_BACKWARD 2
#define LED_BUTTON_LEFT 3
#define N_LEDS_BUTTONS 4
static WbDeviceTag leds_buttons[N_LEDS_BUTTONS];
#define N_LEDS_CIRCLE 8
static WbDeviceTag leds_circle[N_LEDS_CIRCLE];
#define N_LEDS_PROX_H 8
static WbDeviceTag leds_prox_h[N_LEDS_PROX_H];
#define N_LEDS_PROX_V 2
static WbDeviceTag leds_prox_v[N_LEDS_PROX_V];
static WbDeviceTag leds_sound;
static WbDeviceTag leds_rc;
#define TEMPERATURE_RED 0
#define TEMPERATURE_BLUE 1
#define N_LEDS_TEMPERATURE 2
static WbDeviceTag leds_temperature[N_LEDS_TEMPERATURE];
static WbDeviceTag leds_top;
static WbDeviceTag leds_bottom_left;
static WbDeviceTag leds_bottom_right;

#define BUTTON_BACKWARD 0
#define BUTTON_LEFT 1
#define BUTTON_CENTER 2
#define BUTTON_FORWARD 3
#define BUTTON_RIGHT 4
#define N_BUTTONS 5
static WbDeviceTag buttons[N_BUTTONS];
static bool buttons_pressed[N_BUTTONS];

static WbDeviceTag motor_left;
static WbDeviceTag motor_right;

static WbDeviceTag acc;

#define N_PROX_HORIZONTAL 7
static WbDeviceTag prox_horizontal[N_PROX_HORIZONTAL];

#define N_PROX_GROUND 2
static WbDeviceTag prox_ground[N_PROX_GROUND];

static void init_devices() {
  int i;
  char device_name[64];
  for (i = 0; i < N_LEDS_BUTTONS; ++i) {
    sprintf(device_name, "leds.buttons.led%c", '0' + i);
    leds_buttons[i] = wb_robot_get_device(device_name);
  }
  for (i = 0; i < N_LEDS_CIRCLE; ++i) {
    sprintf(device_name, "leds.circle.led%c", '0' + i);
    leds_circle[i] = wb_robot_get_device(device_name);
  }
  for (i = 0; i < N_LEDS_PROX_H; ++i) {
    sprintf(device_name, "leds.prox.h.led%c", '0' + i);
    leds_prox_h[i] = wb_robot_get_device(device_name);
  }
  for (i = 0; i < N_LEDS_PROX_V; ++i) {
    sprintf(device_name, "leds.prox.v.led%c", '0' + i);
    leds_prox_v[i] = wb_robot_get_device(device_name);
  }

  leds_sound = wb_robot_get_device("leds.sound");
  leds_rc = wb_robot_get_device("leds.rc");
  leds_temperature[TEMPERATURE_RED] = wb_robot_get_device("leds.temperature.red");
  leds_temperature[TEMPERATURE_BLUE] = wb_robot_get_device("leds.temperature.blue");
  wb_led_set(leds_temperature[TEMPERATURE_RED], 32);
  wb_led_set(leds_temperature[TEMPERATURE_BLUE], 32);

  leds_top = wb_robot_get_device("leds.top");
  leds_bottom_left = wb_robot_get_device("leds.bottom.left");
  leds_bottom_right = wb_robot_get_device("leds.bottom.right");

  buttons[BUTTON_BACKWARD] = wb_robot_get_device("button.backward");
  buttons[BUTTON_LEFT] = wb_robot_get_device("button.left");
  buttons[BUTTON_CENTER] = wb_robot_get_device("button.center");
  buttons[BUTTON_FORWARD] = wb_robot_get_device("button.forward");
  buttons[BUTTON_RIGHT] = wb_robot_get_device("button.right");

  for (i = 0; i < N_BUTTONS; ++i)
    wb_touch_sensor_enable(buttons[i], TIME_STEP);

  motor_left = wb_robot_get_device("motor.left");
  motor_right = wb_robot_get_device("motor.right");
  wb_motor_set_position(motor_left, INFINITY);
  wb_motor_set_position(motor_right, INFINITY);

  acc = wb_robot_get_device("acc");
  wb_accelerometer_enable(acc, TIME_STEP);

  for (i = 0; i < N_PROX_HORIZONTAL; ++i) {
    sprintf(device_name, "prox.horizontal.%c", '0' + i);
    prox_horizontal[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(prox_horizontal[i], TIME_STEP);
  }

  for (i = 0; i < N_PROX_GROUND; ++i) {
    sprintf(device_name, "prox.ground.%c", '0' + i);
    prox_ground[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(prox_ground[i], TIME_STEP);
  }
}

static int generate_color_from_time(double time) {
  static double _2_PI = 2.0 * M_PI;
  static double _PI_OVER_3 = M_PI / 3.0;
  static double _2_PI_OVER_3 = 2.0 * M_PI / 3.0;

  double red = 0.5 * sin(_2_PI / LED_FREQ * time) + 0.5;
  double green = 0.5 * sin(_2_PI / LED_FREQ * time + _PI_OVER_3) + 0.5;
  double blue = 0.5 * sin(_2_PI / LED_FREQ * time + _2_PI_OVER_3) + 0.5;
  return ((int)(red * 0xff) << 16) + ((int)(green * 0xff) << 8) + (int)(blue * 0xff);
}

int main(int argc, char **argv) {
  wb_robot_init();
  init_devices();
  wb_keyboard_enable(TIME_STEP);
  wb_motor_set_velocity(motor_left, 0);
  wb_motor_set_velocity(motor_right, 0);
  int i;
  for (i = 0; i < N_BUTTONS; i++)
    buttons_pressed[i] = false;
  while (wb_robot_step(TIME_STEP) != -1) {
    const char *message;
    while ((message = wb_robot_wwi_receive_text())) {
      if (strcmp(message, "configure") == 0)
        wbu_default_robot_window_configure();
      else if (strncmp(message, "mousedown ", 10) == 0) {
        if (strncmp(&message[10], "backward", 8) == 0)
          buttons_pressed[BUTTON_BACKWARD] = true;
        else if (strncmp(&message[10], "forward", 7) == 0)
          buttons_pressed[BUTTON_FORWARD] = true;
        else if (strncmp(&message[10], "center", 6) == 0)
          buttons_pressed[BUTTON_CENTER] = true;
        else if (strncmp(&message[10], "right", 5) == 0)
          buttons_pressed[BUTTON_RIGHT] = true;
        else if (strncmp(&message[10], "left", 4) == 0)
          buttons_pressed[BUTTON_LEFT] = true;
        else
          printf("%s\n", message);
      } else if (strncmp(message, "mouseup", 7) == 0) {
        for (i = 0; i < N_BUTTONS; i++)
          buttons_pressed[i] = false;
      } else
        printf("received unknown message from robot window: %s\n", message);
    }
    double time = wb_robot_get_time();

    int enlighted_cicle_led_index = ((int)time) % N_LEDS_CIRCLE;
    for (i = 0; i < N_LEDS_CIRCLE; ++i)
      wb_led_set(leds_circle[i], (enlighted_cicle_led_index == i) ? 32 : 0);

    buttons_pressed[BUTTON_BACKWARD] |= (wb_touch_sensor_get_value(buttons[BUTTON_BACKWARD]) == 1);
    buttons_pressed[BUTTON_FORWARD] |= (wb_touch_sensor_get_value(buttons[BUTTON_FORWARD]) == 1);
    buttons_pressed[BUTTON_CENTER] |= (wb_touch_sensor_get_value(buttons[BUTTON_CENTER]) == 1);
    buttons_pressed[BUTTON_RIGHT] |= (wb_touch_sensor_get_value(buttons[BUTTON_RIGHT]) == 1);
    buttons_pressed[BUTTON_LEFT] |= (wb_touch_sensor_get_value(buttons[BUTTON_LEFT]) == 1);

    wb_led_set(leds_buttons[LED_BUTTON_BACKWARD],
               (buttons_pressed[BUTTON_BACKWARD] || buttons_pressed[BUTTON_CENTER]) ? 32 : 0);
    wb_led_set(leds_buttons[LED_BUTTON_FORWARD], (buttons_pressed[BUTTON_FORWARD] || buttons_pressed[BUTTON_CENTER]) ? 32 : 0);
    wb_led_set(leds_buttons[LED_BUTTON_RIGHT], (buttons_pressed[BUTTON_RIGHT] || buttons_pressed[BUTTON_CENTER]) ? 32 : 0);
    wb_led_set(leds_buttons[LED_BUTTON_LEFT], (buttons_pressed[BUTTON_LEFT] || buttons_pressed[BUTTON_CENTER]) ? 32 : 0);

    if (buttons_pressed[BUTTON_CENTER]) {
      const double *acc_values = wb_accelerometer_get_values(acc);
      printf("acc: %f %f %f\n", acc_values[0], acc_values[1], acc_values[2]);
    }

    wb_led_set(leds_top, generate_color_from_time(time));
    wb_led_set(leds_bottom_left, generate_color_from_time(time + 0.333 * LED_FREQ));
    wb_led_set(leds_bottom_right, generate_color_from_time(time + 0.666 * LED_FREQ));

    char sensor_message[256];
    // we need to fit the sensor values in the range [0;100], hence the divisions
    snprintf(sensor_message, 256, "%d %d %d %d %d %d %d %d %d", (int)(wb_distance_sensor_get_value(prox_horizontal[0]) / 43.08),
             (int)(wb_distance_sensor_get_value(prox_horizontal[1]) / 43.08),
             (int)(wb_distance_sensor_get_value(prox_horizontal[2]) / 43.08),
             (int)(wb_distance_sensor_get_value(prox_horizontal[3]) / 43.08),
             (int)(wb_distance_sensor_get_value(prox_horizontal[4]) / 43.08),
             (int)(wb_distance_sensor_get_value(prox_horizontal[5]) / 43.08),
             (int)(wb_distance_sensor_get_value(prox_horizontal[6]) / 43.08),
             (int)(wb_distance_sensor_get_value(prox_ground[0]) / 10),
             (int)(wb_distance_sensor_get_value(prox_ground[1]) / 10));
    wb_robot_wwi_send_text(sensor_message);

    for (i = 0; i < N_PROX_HORIZONTAL; ++i) {
      if (i == 2) {
        wb_led_set(leds_prox_h[i], wb_distance_sensor_get_value(prox_horizontal[i]) * 32 / 4400);
        wb_led_set(leds_prox_h[i + 1], wb_distance_sensor_get_value(prox_horizontal[i]) * 32 / 4400);
      } else if (i > 2)
        wb_led_set(leds_prox_h[i + 1], wb_distance_sensor_get_value(prox_horizontal[i]) * 32 / 4400);
      else
        wb_led_set(leds_prox_h[i], wb_distance_sensor_get_value(prox_horizontal[i]) * 32 / 4400);
    }
    for (i = 0; i < N_PROX_GROUND; ++i)
      wb_led_set(leds_prox_v[i], wb_distance_sensor_get_value(prox_ground[i]) * 32 / 1000);

    double left_speed = 0.0;
    double right_speed = 0.0;
    while (true) {
      int key = wb_keyboard_get_key();
      if (key == -1)
        break;
      switch (key) {
        case WB_KEYBOARD_UP:
          left_speed += MAX_SPEED;
          right_speed += MAX_SPEED;
          break;
        case WB_KEYBOARD_DOWN:
          left_speed -= MAX_SPEED;
          right_speed -= MAX_SPEED;
          break;
        case WB_KEYBOARD_LEFT:
          left_speed -= MAX_SPEED;
          right_speed += MAX_SPEED;
          break;
        case WB_KEYBOARD_RIGHT:
          left_speed += MAX_SPEED;
          right_speed -= MAX_SPEED;
          break;
      }
    }
    if (buttons_pressed[BUTTON_FORWARD]) {
      left_speed = MAX_SPEED;
      right_speed = MAX_SPEED;
    } else if (buttons_pressed[BUTTON_BACKWARD]) {
      left_speed = -MAX_SPEED;
      right_speed = -MAX_SPEED;
    } else if (buttons_pressed[BUTTON_LEFT]) {
      left_speed = -MAX_SPEED;
      right_speed = MAX_SPEED;
    } else if (buttons_pressed[BUTTON_RIGHT]) {
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    }
    wb_motor_set_velocity(motor_left, left_speed);
    wb_motor_set_velocity(motor_right, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
