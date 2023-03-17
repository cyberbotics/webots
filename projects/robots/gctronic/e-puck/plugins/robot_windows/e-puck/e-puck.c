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

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/gyro.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/plugins/robot_window/default.h>
#include <webots/plugins/robot_window/robot_wwi.h>
#include <webots/position_sensor.h>
#include <webots/remote_control.h>
#include <webots/robot.h>
#include <webots/utils/system.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include "../../remote_controls/e-puck_bluetooth/UploaderData.hpp"

static WbDeviceTag ps[8], ls[8], tof, accelerometer, gyro, camera, gs[3], motors[2], position_sensors[2];
static const int N_SENSORS = sizeof(ps) / sizeof(WbDeviceTag);
static int gs_sensors_count = 0;
static bool configured = false;
static int n_ports;
static char **ports = NULL;

static void cleanup_ports() {
  if (ports)
    free(ports[0]);
  free(ports);
}

static void get_ports() {
  cleanup_ports();
  char *p = (char *)wb_remote_control_custom_function(NULL);
  // return a list of COM port files separated by `\n`
  if (!p)
    return;
  int l = strlen(p);
  if (l == 0) {
    n_ports = 0;
    ports = NULL;
    free(p);
    return;
  }
  ports = (char **)malloc(sizeof(char *) * l);
  ports[0] = p;
  n_ports = 1;
  int i;
  for (i = 0; i < l; i++) {
    if (p[i] == '\n') {
      p[i] = '\0';
      ports[n_ports] = &p[i + 1];
      n_ports++;
    }
  }
}

static void send_ports() {
  get_ports();
  char text[1024];
  text[0] = '\0';
  strcat(text, "ports");
  int i, j, k = strlen(text);
  for (i = 0; i < n_ports; i++) {
    text[k++] = ' ';
    for (j = 0; j < strlen(ports[i]); j++) {
      if (ports[i][j] == '\\')
        text[k++] = '\\';  // escape character
      text[k++] = ports[i][j];
    }
  }
  text[k++] = '\0';
  wb_robot_wwi_send_text(text);
}

void wb_robot_window_init() {
  char device[32];
  int i;
  for (i = 0; i < N_SENSORS; i++) {
    snprintf(device, 32, "ps%d", i);
    ps[i] = wb_robot_get_device(device);
    snprintf(device, 32, "ls%d", i);
    ls[i] = wb_robot_get_device(device);
  }

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0)
    tof = wb_robot_get_device("tof");
  else
    tof = 0;

  accelerometer = wb_robot_get_device("accelerometer");
  gyro = wb_robot_get_device("gyro");
  camera = wb_robot_get_device("camera");
  motors[0] = wb_robot_get_device("left wheel motor");
  motors[1] = wb_robot_get_device("right wheel motor");
  position_sensors[0] = wb_robot_get_device("left wheel sensor");
  position_sensors[1] = wb_robot_get_device("right wheel sensor");

  // optional ground sensors
  for (i = 0; i < 3; i++)
    gs[i] = 0;
  int groundSensorIndex;
  int numberOfDevices = wb_robot_get_number_of_devices();
  for (i = 0; i < numberOfDevices; i++) {
    WbDeviceTag tag = wb_robot_get_device_by_index(i);
    WbNodeType deviceType = wb_device_get_node_type(tag);
    if (deviceType == WB_NODE_DISTANCE_SENSOR) {
      const char *deviceName = wb_device_get_name(tag);
      int matchedItems = sscanf(deviceName, "gs%d", &groundSensorIndex);
      if (matchedItems > 0) {
        // init ground sensors
        if (groundSensorIndex < 3 && gs[groundSensorIndex] == 0) {
          gs[groundSensorIndex] = tag;
          ++gs_sensors_count;
        }
      }
    }
  }
  if (gs_sensors_count != 3)
    // all the 3 ground sensors have to be available
    gs_sensors_count = 0;
}

static void upload_progress_callback(int i, int j) {
  static int jj = -1;
  if (i == 2)
    wb_robot_wwi_send_text("upload reset");
  else {
    if (j != jj) {
      char buffer[64];
      sprintf(buffer, "upload %d", j);
      wb_robot_wwi_send_text(buffer);
      if (j == 99)
        wb_robot_wwi_send_text("upload complete");
      jj = j;
    }
  }
}

void wb_robot_window_step(int time_step) {
  int i;
  const char *message;
  while ((message = wb_robot_wwi_receive_text())) {
    if (strcmp(message, "configure") == 0) {
      send_ports();
      wbu_default_robot_window_configure();
      configured = true;
    } else if (strcmp(message, "enable") == 0) {
      wb_camera_enable(camera, time_step);
      wb_accelerometer_enable(accelerometer, time_step);
      wb_gyro_enable(gyro, time_step);
      wb_position_sensor_enable(position_sensors[0], time_step);
      wb_position_sensor_enable(position_sensors[1], time_step);
      for (i = 0; i < N_SENSORS; i++) {
        wb_distance_sensor_enable(ps[i], time_step);
        wb_light_sensor_enable(ls[i], time_step);
      }
      if (tof != 0)
        wb_distance_sensor_enable(tof, time_step);
      // optional ground sensors
      for (i = 0; i < gs_sensors_count; i++)
        wb_distance_sensor_enable(gs[i], time_step);
    } else if (strcmp(message, "refresh") == 0)
      send_ports();
    else if (strcmp(message, "simulation") == 0)
      wb_robot_set_mode(WB_MODE_SIMULATION, NULL);
    else if (strncmp(message, "remote control ", 15) == 0)
      wb_robot_set_mode(WB_MODE_REMOTE_CONTROL, &message[15]);
    else if (strncmp(message, "upload ", 7) == 0) {
      char *port;
      const char *p = &message[7];
      int n = strchr(p, ' ') - p;
      port = (char *)malloc(n + 1);
      strncpy(port, p, n);
      port[n] = '\0';
      struct UploaderData upload;
      upload.command = UPLOADER_DATA_CONNECT;
      upload.data = port;
      wb_remote_control_custom_function(&upload);
      free(port);
      const char *data = &message[8 + n];
      const char *path = wbu_system_short_path(wbu_system_webots_instance_path(false));
      const char *filename = "e-puck.hex";
      char *full_path = (char *)malloc(strlen(path) + strlen(filename) + 1);
      sprintf(full_path, "%s%s", path, filename);
      FILE *fd = fopen(full_path, "wb");
      if (fd == NULL)
        fprintf(stderr, "Cannot open %s for writting\n", full_path);
      else {
        fwrite(data, 1, strlen(data), fd);
        fclose(fd);
        upload.command = UPLOADER_DATA_SEND_FILE;
        upload.robot_id = 100;
        upload.data = full_path;
        upload.progress_callback = upload_progress_callback;
        wb_remote_control_custom_function(&upload);
        upload.command = UPLOADER_DATA_DISCONNECT;
        wb_remote_control_custom_function(&upload);
      }
      free(full_path);
    } else if (strncmp(message, "connect ", 8) == 0) {
      wb_robot_set_mode(WB_MODE_REMOTE_CONTROL, &message[8]);
      fprintf(stderr, "Connected to %s\n", &message[8]);
    } else if (strncmp(message, "disconnect", 10) == 0) {
      wb_robot_set_mode(WB_MODE_SIMULATION, NULL);
      fprintf(stderr, "Disconnected from e-puck2\n");
    } else
      fprintf(stderr, "received unknown message from robot window: %s\n", message);
  }
  if (!configured)
    return;
  const int UPDATE_MESSAGE_SIZE = 1024;
  const int UPDATE_SIZE = 256;
  char *update_message = malloc(UPDATE_MESSAGE_SIZE);
  char *update = malloc(UPDATE_SIZE);
  update_message[0] = '\0';
  for (i = 0; i < 8; i++) {
    double v;
    if (wb_distance_sensor_get_sampling_period(ps[i]))
      v = wb_distance_sensor_get_value(ps[i]);
    else
      v = NAN;
    if (isnan(v))
      snprintf(update, UPDATE_SIZE, "ps%d ", i);
    else
      snprintf(update, UPDATE_SIZE, "%d ", (int)v);
    if (strlen(update) + strlen(update_message) < UPDATE_MESSAGE_SIZE)
      strcat(update_message, update);
  }
  for (i = 0; i < 8; i++) {
    double v;
    if (wb_light_sensor_get_sampling_period(ls[i]))
      v = wb_light_sensor_get_value(ls[i]);
    else
      v = NAN;
    if (isnan(v))
      snprintf(update, UPDATE_SIZE, "ls%d ", i);
    else
      snprintf(update, UPDATE_SIZE, "%d ", (int)v);
    if (strlen(update) + strlen(update_message) < UPDATE_MESSAGE_SIZE)
      strcat(update_message, update);
  }

  if (tof == 0 || wb_distance_sensor_get_sampling_period(tof) == 0)
    snprintf(update, UPDATE_SIZE, "tof ");
  else {
    double tof_distance = wb_distance_sensor_get_value(tof);
    if (isnan(tof_distance))
      snprintf(update, UPDATE_SIZE, "tof ");
    else
      snprintf(update, UPDATE_SIZE, "%.0lf ", tof_distance);
  }
  if (strlen(update) + strlen(update_message) < UPDATE_MESSAGE_SIZE)
    strcat(update_message, update);

  bool areDevicesReady = true;
  double left_speed = wb_motor_get_velocity(motors[0]);
  if (isnan(left_speed)) {
    snprintf(update, UPDATE_SIZE, "speed ");
    areDevicesReady = false;
  } else
    snprintf(update, UPDATE_SIZE, "%.3lf ", left_speed);
  if (strlen(update) + strlen(update_message) < UPDATE_MESSAGE_SIZE)
    strcat(update_message, update);

  double right_speed = wb_motor_get_velocity(motors[1]);
  if (isnan(right_speed)) {
    snprintf(update, UPDATE_SIZE, "speed ");
    areDevicesReady = false;
  } else
    snprintf(update, UPDATE_SIZE, "%.3lf ", right_speed);
  if (strlen(update) + strlen(update_message) < UPDATE_MESSAGE_SIZE)
    strcat(update_message, update);

  if (areDevicesReady && wb_position_sensor_get_sampling_period(position_sensors[0])) {
    const double value = wb_position_sensor_get_value(position_sensors[0]);
    if (isnan(value))
      snprintf(update, UPDATE_SIZE, "left_wheel_position ");
    else
      snprintf(update, UPDATE_SIZE, "%.3lf ", value);
  } else
    snprintf(update, UPDATE_SIZE, "left_wheel_position ");
  if (strlen(update) + strlen(update_message) < UPDATE_MESSAGE_SIZE)
    strcat(update_message, update);

  if (areDevicesReady && wb_position_sensor_get_sampling_period(position_sensors[1])) {
    const double value = wb_position_sensor_get_value(position_sensors[1]);
    if (isnan(value))
      snprintf(update, UPDATE_SIZE, "right_wheel_position ");
    else
      snprintf(update, UPDATE_SIZE, "%.3lf ", value);
  } else
    snprintf(update, UPDATE_SIZE, "right_wheel_position ");
  if (strlen(update) + strlen(update_message) < UPDATE_MESSAGE_SIZE)
    strcat(update_message, update);

  if (wb_accelerometer_get_sampling_period(accelerometer)) {
    const double *values = wb_accelerometer_get_values(accelerometer);
    const char name[3] = "XYZ";
    update[0] = '\0';
    for (i = 0; i < 3; ++i) {
      char s[9];  // "[+-]\d\d\.\d\d\d \0"
      if (isnan(values[i]))
        sprintf(s, "%c ", name[i]);
      else
        sprintf(s, "%.3f ", values[i]);
      strcat(update, s);
    }
  } else
    snprintf(update, UPDATE_SIZE, "X Y Z ");
  if (strlen(update) + strlen(update_message) < UPDATE_MESSAGE_SIZE)
    strcat(update_message, update);

  if (wb_gyro_get_sampling_period(gyro)) {
    const double *values = wb_gyro_get_values(gyro);
    const char name[4] = "Gyro";
    update[0] = '\0';
    for (i = 0; i < 3; ++i) {
      char s[12];  // "[+-]\d\d\.\d\d\d \0"
      if (isnan(values[i]))
        sprintf(s, "%c ", name[i]);
      else
        sprintf(s, "%.3f ", values[i]);
      strcat(update, s);
    }
  } else
    snprintf(update, UPDATE_SIZE, "gX gY gZ ");
  if (strlen(update) + strlen(update_message) < UPDATE_MESSAGE_SIZE)
    strcat(update_message, update);

  if (areDevicesReady && wb_camera_get_sampling_period(camera))
    wbu_default_robot_window_update();  // we send all the update to get the image in base64.

  for (i = 0; i < gs_sensors_count; i++) {
    double v;
    if (wb_distance_sensor_get_sampling_period(gs[i]))
      v = wb_distance_sensor_get_value(gs[i]);
    else
      v = NAN;
    if (isnan(v))
      snprintf(update, UPDATE_SIZE, "-1 ");
    else {
      int c = (v - 300.0) * 255.0 / 700.0;
      if (c > 255)
        c = 255;
      else if (c < 0)
        c = 0;
      snprintf(update, UPDATE_SIZE, "%d ", c);
    }
    if (strlen(update) + strlen(update_message) < UPDATE_MESSAGE_SIZE)
      strcat(update_message, update);
  }
  wb_robot_wwi_send_text(update_message);
  free(update);
  free(update_message);
}

void wb_robot_window_cleanup() {
  cleanup_ports();
}
