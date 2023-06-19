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
 * Description:  An example of the use of an ODE physics plugin.
 */

#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#define TIME_STEP 64

int main() {
  /* initialize Webots */
  wb_robot_init();

  /*
   * We get a handler to the distance sensors, the motors, the emitter and
   * the receivers and we activate the ones which need to.
   */
  WbDeviceTag ds0 = wb_robot_get_device("ds0");
  WbDeviceTag ds1 = wb_robot_get_device("ds1");

  WbDeviceTag rotor0 = wb_robot_get_device("lower_rotor");
  WbDeviceTag rotor1 = wb_robot_get_device("upper_rotor");

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  WbDeviceTag data_receiver = wb_robot_get_device("data_receiver");
  WbDeviceTag color_receiver = wb_robot_get_device("color_receiver");

  WbDeviceTag led = wb_robot_get_device("led");

  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  wb_receiver_enable(data_receiver, TIME_STEP);
  wb_receiver_enable(color_receiver, TIME_STEP);

  /* This allows the motors to turn endlessly in the same direction. */
  wb_motor_set_position(rotor1, INFINITY);
  wb_motor_set_position(rotor0, -INFINITY);

  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * We send the values of our sensors to the ODE plugin which is responsible to
     * manages our movements.
     */
    double ds0_value = wb_distance_sensor_get_value(ds0);
    double ds1_value = wb_distance_sensor_get_value(ds1);

    /* sending an array of 2 floats */
    float emitter_buffer[2] = {ds0_value, ds1_value};
    wb_emitter_send(emitter, emitter_buffer, sizeof(emitter_buffer));

    /*
     * We also look on the data receiver to see if there are some data as the
     * plugin will inform us on its actions.
     */
    while (wb_receiver_get_queue_length(data_receiver) > 0) {
      /* receiving an array of 2 ints */
      const int *data_receiver_buffer = wb_receiver_get_data(data_receiver);
      printf("Avoiding %d obstacles when turning on the %s\n", data_receiver_buffer[0],
             (data_receiver_buffer[1] == 0) ? "left" : "right");
      wb_receiver_next_packet(data_receiver);
    }

    while (wb_receiver_get_queue_length(color_receiver) > 0) {
      /* receiving 1 int */
      const int *color_receiver_buffer = wb_receiver_get_data(color_receiver);
      wb_receiver_next_packet(color_receiver);
      int led_color = color_receiver_buffer[0] != 0 ? 0xFF0000 : 0x00FF00;
      wb_led_set(led, led_color);
    }
  }

  return 0;
}
