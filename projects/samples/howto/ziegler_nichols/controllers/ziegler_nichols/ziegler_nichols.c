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
 * Description:  This controller exemplifies the use of wb_motor_set_pid()
 *               It adapts its PID parameters as specified by the Ziegler-Nichols tuning method.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/display.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TARGET_POSITION M_PI_2
#define INCREMENT 10
#define SUFFICIENT_NUMBER_OF_PERIODS 10
#define THRESHOLD 0.001
#define SAMPLING_PERIOD 400
#define LABEL_X 0.05
#define LABEL_Y 0.95
#define BLACK 0x000000
#define WHITE 0xFFFFFF
#define RED 0xFF0000

// Display variables
static WbDeviceTag display;
static int width = 0;
static int height = 0, half_height = 0;
static double display_middle_y = 0.0;
static double display_unit_x = 0.0;
static double display_unit_y = 0.0;
static int time_step = 0;
// Error related variables
static double error = TARGET_POSITION, previous_error = TARGET_POSITION;
static double max = 0.0, min = 0.0;
static int number_of_sign_changes = 0, number_of_periods = 0, previous_sign = -1;
static int time = 0, t_0 = 0;
// Ziegler-Nichols's ultimate amplitude
static double amplitude_u = 0.0, previous_amplitude_u = 0.0;

static void reset_display() {
  wb_display_set_color(display, BLACK);
  wb_display_fill_rectangle(display, 0, 0, width - 1, height - 1);
  wb_robot_step(time_step);
  wb_display_set_color(display, RED);
  wb_display_draw_line(display, 0, half_height, width, half_height);
  wb_robot_step(time_step);
  wb_display_set_color(display, WHITE);  // draw position error graph in white
}

static void reset() {
  number_of_sign_changes = 0;
  number_of_periods = 0;
  previous_sign = -1;
  t_0 = 0;
  time = 0;
  amplitude_u = 0.0;
  previous_amplitude_u = 0.0;
  previous_error = TARGET_POSITION;
  max = 0.0;
  min = 0.0;
  reset_display();
}

static void draw_error() {
  const double previous_t = (time - 1) * display_unit_x;
  const double previous_err = display_middle_y - previous_error * display_unit_y;
  const double current_t = previous_t + display_unit_x;
  const double current_err = display_middle_y - error * display_unit_y;
  wb_display_draw_line(display, (int)previous_t, (int)previous_err, (int)current_t, (int)current_err);
}

int main() {
  wb_robot_init();
  time_step = wb_robot_get_basic_time_step();

  printf("----- Ziegler-Nichols PID tuning method -----\n");
  printf("Each P-controller is tested during a period of %d time steps, i.e. %g seconds\n", SAMPLING_PERIOD,
         0.001 * SAMPLING_PERIOD * time_step);
  printf("A P-controller 'succeeds' if a constant error amplitude repeats at least %d times during the test period\n",
         SUFFICIENT_NUMBER_OF_PERIODS);
  printf("Otherwise the proportional gain P is incremented by %d and the experiment restarts\n", INCREMENT);
  printf("The tuned Ziegler-Nichols 'ultimate' gains will be computed for the first successful controller\n");

  // P parameter to be tuned
  double p = INCREMENT;
  //  Ziegler-Nichols' ultimate oscillation period
  double t_u = 0.0;

  // Initializes motor
  const WbDeviceTag motor = wb_robot_get_device("rotational motor");
  wb_motor_set_control_pid(motor, p, 0.0, 0.0);
  // Initializes position sensor
  const WbDeviceTag position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, time_step);

  // Initializes display content and display variables
  display = wb_robot_get_device("display");
  height = wb_display_get_height(display);
  half_height = height / 2;
  width = wb_display_get_width(display);
  display_middle_y = 0.5 * height;
  display_unit_x = ((double)width) / SAMPLING_PERIOD;
  display_unit_y = ((double)height) / M_PI;

  // Initialize supervisor label
  char buffer[50];
  sprintf(buffer, "P: %g", p);
  wb_supervisor_set_label(0, buffer, LABEL_X, LABEL_Y, 0.1, 0, 0, "Arial");
  wb_motor_set_position(motor, TARGET_POSITION);
  reset();

  // Gets the position field to supervise the tuning
  const WbNodeRef hinge_parameters = wb_supervisor_node_get_from_def("HINGE_PARAMETERS");
  const WbFieldRef position_field = wb_supervisor_node_get_field(hinge_parameters, "position");

  while (wb_robot_step(time_step) != -1) {
    ++time;

    if (time == SAMPLING_PERIOD) {
      wb_supervisor_field_set_sf_float(position_field, 0.0);
      p += INCREMENT;
      wb_motor_set_control_pid(motor, p, 0.0, 0.0);
      wb_motor_set_position(motor, TARGET_POSITION);
      wb_supervisor_set_label(0, buffer, LABEL_X, LABEL_Y, 0.1, 0, 0, "Arial");
      reset();
      continue;
    }

    error = TARGET_POSITION - wb_position_sensor_get_value(position_sensor);
    draw_error();
    const int sign = error < 0.0 ? -1 : 1;
    previous_error = error;

    if (error == 0.0 || sign != previous_sign) {
      previous_sign = sign;
      if (++number_of_sign_changes % 2 == 0) {
        t_u = time - t_0;
        t_0 = time;
        amplitude_u = max - min;
        previous_amplitude_u = amplitude_u;
        if (fabs(amplitude_u - previous_amplitude_u) < THRESHOLD) {
          ++number_of_periods;
          if (number_of_periods > SUFFICIENT_NUMBER_OF_PERIODS)
            break;
        } else
          number_of_periods = 0;
      }
    }

    if (error < min)
      min = error;
    else if (error > max)
      max = error;
  }

  // Runs the tuned PID controller
  const double k_p = 0.6 * p;
  const double k_i = 2.0 * k_p / t_u;
  const double k_d = k_p * t_u / 8.0;
  wb_motor_set_control_pid(motor, k_p, k_i, k_d);
  wb_motor_set_position(motor, TARGET_POSITION);
  wb_supervisor_field_set_sf_float(position_field, 0.0);
  sprintf(buffer, "P: %g I: %g D: %g", k_p, k_i, k_d);
  wb_supervisor_set_label(0, buffer, LABEL_X, LABEL_Y, 0.1, 0, 0, "Arial");
  reset();
  wb_display_draw_text(display, "PID with tuned gains", width / 3, 3 * height / 4);
  while (wb_robot_step(time_step) != -1 && time <= SAMPLING_PERIOD) {
    ++time;
    error = TARGET_POSITION - wb_position_sensor_get_value(position_sensor);
    draw_error();
    previous_error = error;
  }
  wb_display_draw_text(display, "(Controller exited)", width / 3, 7 * height / 8);
  wb_robot_step(time_step);
  return 0;
}
