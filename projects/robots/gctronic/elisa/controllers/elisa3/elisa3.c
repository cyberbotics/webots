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
#include <stdlib.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64
#define RANGE (127 / 2)

#define LEDS_NUMBER 8
static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {
  "led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7",
};

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void init_devices() {
  int i;
  for (i = 0; i < LEDS_NUMBER; i++)
    leds[i] = wb_robot_get_device(leds_names[i]);

  step();
}

int main(int argc, char *argv[]) {
  /* define variables */
  long int currentColor = 0;
  if (argc == 4)
    currentColor = atoi(argv[1]) * 65536 + atoi(argv[2]) * 256 + atoi(argv[3]);
  WbDeviceTag rgbLed;
  WbDeviceTag distance_sensor[8];
  WbDeviceTag left_motor, right_motor;
  int i, j;
  double speed[2] = {0, 0};
  double sensors_value[8];
  double braitenberg_coefficients[8][2] = {{10, 8}, {7, -1.5}, {5, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, 5}, {-1.5, 7}};

  /* initialize Webots */
  wb_robot_init();

  /* get and enable devices */
  init_devices();

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  rgbLed = wb_robot_get_device("ledrgb");

  for (i = 0; i < 8; i++) {
    char device_name[4];
    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], TIME_STEP * 4);
  }

  for (i = 0; i < LEDS_NUMBER; i++)
    wb_led_set(leds[i], true);

  wb_led_set(rgbLed, currentColor);  // 0x0000ff);

  // wait until first distance sensor values are available
  if (wb_robot_step(TIME_STEP * 4) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }

  /* main loop */
  while (true) {
    /* get sensors values */
    for (i = 0; i < 8; i++)
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);

    /*
     printf("prox values = %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f\n",
     sensors_value[0], sensors_value[1], sensors_value[2], sensors_value[3],
     sensors_value[4], sensors_value[5], sensors_value[6], sensors_value[7]);
    */

    /* compute speed values*/
    for (i = 0; i < 2; i++) {
      speed[i] = 0.0;
      for (j = 0; j < 8; j++)
        speed[i] += braitenberg_coefficients[j][i] * (1.0 - (sensors_value[j] / RANGE));
    }

    /* set speed values */
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);

    step();
  }

  wb_robot_cleanup();

  return 0;
}
