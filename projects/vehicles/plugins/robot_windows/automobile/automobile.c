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
 * Description:  Defines the entry point of the robot window library
 */

#include <webots/plugins/robot_window/default.h>
#include <webots/plugins/robot_window/generic_robot_window/generic.h>
#include <webots/robot.h>
#include <webots/utils/string.h>

#include <webots/motor.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static const char *wheelMotorNames[] = {"right_front_wheel", "left_front_wheel", "right_rear_wheel", "left_rear_wheel"};
static WbDeviceTag *wheelMotors = NULL;

enum DriverInfo {
  NONE = 0,
  OVERVIEW_INFO = 1,
  SPEED_INFO,
  STEERING_INFO,
  ENCODERS_INFO,
  BRAKE_INFO,
  THROTTLE_INFO,
  RPM_INFO,
  COUNT
};
static bool driverInfoEnabled[COUNT];

static char *buffer = NULL;
static int buffer_size = 0;

static void free_buffer() {
  free(buffer);
  buffer = NULL;
}

static void create_buffer(int size) {
  atexit(free_buffer);
  buffer_size = size + 1;
  buffer = malloc(buffer_size);
}

static void buffer_append(const char *string) {
  if (string == NULL || string[0] == '\0')
    return;
  const int l = strlen(string);
  if (buffer == NULL) {
    create_buffer(l);
    memcpy(buffer, string, buffer_size);
    return;
  }
  buffer = realloc(buffer, buffer_size + l);
  if (buffer == NULL) {
    fprintf(stderr, "Error creating message to be sent to the robot window: "
                    "not enough memory.\n");
    exit(EXIT_FAILURE);
  }
  memcpy(&buffer[buffer_size - 1], string, l + 1);
  buffer_size += l;
}

static enum DriverInfo driver_info_from_string(const char *s) {
  if (strcmp(s, "Overview") == 0)
    return OVERVIEW_INFO;
  if (strcmp(s, "Speed") == 0)
    return SPEED_INFO;
  if (strcmp(s, "Steering") == 0)
    return STEERING_INFO;
  if (strcmp(s, "Encoders") == 0)
    return ENCODERS_INFO;
  if (strcmp(s, "Brake") == 0)
    return BRAKE_INFO;
  if (strcmp(s, "Throttle") == 0)
    return THROTTLE_INFO;
  if (strcmp(s, "RPM") == 0)
    return RPM_INFO;
  return NONE;
}

static void enable_driver_info(enum DriverInfo info, bool enable) {
  if (info <= NONE || info >= COUNT) {
    assert(0);
    return;
  }
  driverInfoEnabled[info] = enable;
}

// JavaScript -> C protocol description:
//   [deviceName:commandTag[=commadState][,]]
// example:
//   "myMotor0:value=1.2" or "Throttle:enable"
static void parse_command(char *token, char *tokens) {
  // first token is driver info, device or robot name
  char *name0 = wbu_string_replace(token, "\\:", ":");
  const char *name = wbu_string_replace(name0, "\\,", ",");
  enum DriverInfo info = driver_info_from_string(name);
  if (info == NONE) {
    wbu_generic_robot_window_parse_device_command(token, tokens);
    return;
  }

  token = wbu_string_strsep(&tokens, ":");
  while (token && token[0] != '\0') {
    if (strcmp(token, "enable") == 0)
      enable_driver_info(info, true);
    else if (strcmp(token, "disable") == 0)
      enable_driver_info(info, false);
    token = wbu_string_strsep(&tokens, ":");
  }
}

static void configure_automobile_robot_window() {
  wbu_driver_init();

  // send vehicle config data
  char buf[32];
  buffer_append("configure-vehicle { \"front-track\": ");
  snprintf(buf, 32, "%.17g", wbu_car_get_track_front());
  buffer_append(buf);
  buffer_append(",\"rear-track\": ");
  snprintf(buf, 32, "%.17g", wbu_car_get_track_rear());
  buffer_append(buf);
  buffer_append(",\"wheelbase\": ");
  snprintf(buf, 32, "%.17g", wbu_car_get_wheelbase());
  buffer_append(buf);
  buffer_append(",\"front-wheel-radius\": ");
  snprintf(buf, 32, "%.17g", wbu_car_get_front_wheel_radius());
  buffer_append(buf);
  buffer_append(",\"rear-wheel-radius\": ");
  snprintf(buf, 32, "%.17g", wbu_car_get_rear_wheel_radius());
  buffer_append(buf);
  buffer_append(",\"gear-number\": ");
  snprintf(buf, 32, "%d", wbu_driver_get_gear_number());
  buffer_append(buf);
  buffer_append(",\"transmission\": ");
  const int transmission_type = wbu_car_get_type();
  if (transmission_type == WBU_CAR_TRACTION)
    buffer_append("\"traction\"");
  else if (transmission_type == WBU_CAR_PROPULSION)
    buffer_append("\"propulsion\"");
  else if (transmission_type == WBU_CAR_FOUR_BY_FOUR)
    buffer_append("\"four by four\"");
  else
    buffer_append("\"unknown\"");
  buffer_append(",\"engine\": ");
  const int engine_type = wbu_car_get_engine_type();
  if (engine_type == WBU_CAR_COMBUSTION_ENGINE)
    buffer_append("\"combustion\"");
  else if (engine_type == WBU_CAR_ELECTRIC_ENGINE)
    buffer_append("\"electric\"");
  else if (engine_type == WBU_CAR_PARALLEL_HYBRID_ENGINE)
    buffer_append("\"parallel hybrid\"");
  else if (engine_type == WBU_CAR_POWER_SPLIT_HYBRID_ENGINE)
    buffer_append("\"Power-split hybrid\"");
  else
    buffer_append("\"unknown\"");
  buffer_append("}");
  wb_robot_wwi_send_text(buffer);
  free_buffer();
}

static void append_overview_data(WbuDriverControlMode control_mode) {
  char buf[32];
  buffer_append(", \"overview\": {\"speed\": ");
  snprintf(buf, 32, "%.17g", wbu_driver_get_current_speed());
  buffer_append(buf);
  buffer_append(",\"steering\": [");
  snprintf(buf, 32, "%.17g", wbu_driver_get_steering_angle());
  buffer_append(buf);
  snprintf(buf, 32, ",%.17g", wbu_car_get_right_steering_angle());
  buffer_append(buf);
  snprintf(buf, 32, ",%.17g", wbu_car_get_left_steering_angle());
  buffer_append(buf);
  if (control_mode == TORQUE) {
    buffer_append("],\"rpm\": ");
    snprintf(buf, 32, "%.17g", wbu_driver_get_rpm());
    buffer_append(buf);
    buffer_append(",\"gearbox\": ");
    snprintf(buf, 32, "%d", wbu_driver_get_gear());
    buffer_append(buf);
    buffer_append(",\"target-speed\": 0");
  } else {
    buffer_append("],\"rpm\": 0, \"gearbox\": 0");
    buffer_append(",\"target-speed\": ");
    snprintf(buf, 32, "%.17g", wbu_driver_get_target_cruising_speed());
    buffer_append(buf);
  }
  if (control_mode == UNDEFINED_CONTROL_MODE && wheelMotors == NULL) {
    wheelMotors = malloc(WBU_CAR_WHEEL_NB * sizeof(WbDeviceTag));
    for (int i = WBU_CAR_WHEEL_FRONT_RIGHT; i < WBU_CAR_WHEEL_NB; ++i)
      wheelMotors[i] = wb_robot_get_device(wheelMotorNames[i]);
  }
  for (int i = WBU_CAR_WHEEL_FRONT_RIGHT; i < WBU_CAR_WHEEL_NB; ++i) {
    buffer_append(",\"wheel");
    snprintf(buf, 32, "%d", i + 1);
    buffer_append(buf);
    buffer_append("\": { \"speed\": ");
    if (control_mode != UNDEFINED_CONTROL_MODE)
      snprintf(buf, 32, "%.17g", wbu_car_get_wheel_speed(i));
    else
      snprintf(buf, 32, "%.17g", wb_motor_get_velocity(wheelMotors[i]));
    buffer_append(buf);
    buffer_append(", \"encoder\": ");
    const double encoder_value = wbu_car_get_wheel_encoder(i);
    if (encoder_value == encoder_value) {
      snprintf(buf, 32, "%.17g", encoder_value);
      buffer_append(buf);
    } else  // NaN
      buffer_append("0");
    buffer_append("}");
  }
  buffer_append("}");
}

static void append_speed_data(WbuDriverControlMode control_mode) {
  char buf[32];
  buffer_append(",\"speed\": {");
  if (control_mode == SPEED) {
    buffer_append("\"cruising-speed\": ");
    snprintf(buf, 32, "%.17g,", wbu_driver_get_target_cruising_speed());
    buffer_append(buf);
  }
  buffer_append("\"update\":[{\"time\":");
  snprintf(buf, 32, "%.3f", wb_robot_get_time());
  buffer_append(buf);
  buffer_append(",\"value\":");
  snprintf(buf, 32, "%.17g", wbu_driver_get_current_speed());
  buffer_append(buf);
  buffer_append("}]}");
}

static void append_steering_data() {
  char buf[32];
  buffer_append(", \"steering\": {\"time\": ");
  snprintf(buf, 32, "%.3f", wb_robot_get_time());
  buffer_append(buf);
  buffer_append(",\"value\": [");
  snprintf(buf, 32, "%.17g,", wbu_driver_get_steering_angle());
  buffer_append(buf);
  snprintf(buf, 32, "%.17g,", wbu_car_get_right_steering_angle());
  buffer_append(buf);
  snprintf(buf, 32, "%.17g", wbu_car_get_left_steering_angle());
  buffer_append(buf);
  buffer_append("]}");
}

static void append_throttle_data() {
  char buf[32];
  buffer_append(", \"throttle\": {\"time\": ");
  snprintf(buf, 32, "%.3f", wb_robot_get_time());
  buffer_append(buf);
  buffer_append(",\"value\": ");
  snprintf(buf, 32, "%.17g", wbu_driver_get_throttle());
  buffer_append(buf);
  buffer_append("}");
}

static void append_rpm_data() {
  char buf[32];
  buffer_append(", \"rpm\": {\"time\": ");
  snprintf(buf, 32, "%.3f", wb_robot_get_time());
  buffer_append(buf);
  buffer_append(",\"value\": ");
  snprintf(buf, 32, "%.17g", wbu_driver_get_rpm());
  buffer_append(buf);
  buffer_append("}");
}

static void append_brake_data() {
  char buf[32];
  buffer_append(", \"brake\": {\"time\": ");
  snprintf(buf, 32, "%.3f", wb_robot_get_time());
  buffer_append(buf);
  buffer_append(",\"value\": ");
  snprintf(buf, 32, "%.17g", wbu_driver_get_brake_intensity());
  buffer_append(buf);
  buffer_append("}");
}

static void append_encoders_data() {
  char buf[32];
  buffer_append(", \"encoders\": {\"time\": ");
  snprintf(buf, 32, "%.3f", wb_robot_get_time());
  buffer_append(buf);
  buffer_append(",\"value\": [");
  snprintf(buf, 32, "%.17g,", wbu_car_get_wheel_encoder(WBU_CAR_WHEEL_FRONT_RIGHT));
  buffer_append(buf);
  snprintf(buf, 32, "%.17g,", wbu_car_get_wheel_encoder(WBU_CAR_WHEEL_FRONT_LEFT));
  buffer_append(buf);
  snprintf(buf, 32, "%.17g,", wbu_car_get_wheel_encoder(WBU_CAR_WHEEL_REAR_RIGHT));
  buffer_append(buf);
  snprintf(buf, 32, "%.17g", wbu_car_get_wheel_encoder(WBU_CAR_WHEEL_REAR_LEFT));
  buffer_append(buf);
  buffer_append("]}");
}

void wb_robot_window_init() {
  wbu_generic_robot_window_init();
}

void wb_robot_window_step(int time_step) {
  const char *message;
  while ((message = wb_robot_wwi_receive_text())) {
    if (!wbu_generic_robot_window_handle_messages(message)) {
      char *tokens = strdup(message);
      char *token = NULL;
      while ((token = wbu_string_strsep(&tokens, ","))) {
        char *command = strdup(token);
        char *first_word = wbu_string_strsep(&command, ":");
        if (!wbu_generic_robot_window_parse_device_control_command(first_word, command))
          parse_command(first_word, command);
      }
    }
    if (strncmp(message, "configure", 9) == 0)
      // Additional configuration for the automobile robot window
      configure_automobile_robot_window();
  }

  if (!wbu_generic_robot_window_needs_update())
    return;

  wbu_default_robot_window_update();

  char buf[32];
  buffer_append("update { \"vehicle\": {");
  buffer_append("\"control-mode\": ");
  WbuDriverControlMode control_mode = wbu_driver_get_control_mode();
  snprintf(buf, 32, "%d", control_mode);
  buffer_append(buf);
  if (driverInfoEnabled[SPEED_INFO])
    append_speed_data(control_mode);
  if (driverInfoEnabled[STEERING_INFO])
    append_steering_data();
  if (driverInfoEnabled[THROTTLE_INFO] && control_mode == TORQUE)
    append_throttle_data();
  if (driverInfoEnabled[RPM_INFO] && control_mode == TORQUE)
    append_rpm_data();
  if (driverInfoEnabled[BRAKE_INFO])
    append_brake_data();
  if (driverInfoEnabled[ENCODERS_INFO])
    append_encoders_data();
  if (driverInfoEnabled[OVERVIEW_INFO])
    append_overview_data(control_mode);
  buffer_append("}}");
  wb_robot_wwi_send_text(buffer);
  free_buffer();
}

void wb_robot_window_cleanup() {
  free(wheelMotors);
}
