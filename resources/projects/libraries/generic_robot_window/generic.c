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

#include <webots/plugins/robot_window/default.h>
#include <webots/plugins/robot_window/generic_robot_window/generic.h>

#include <webots/device.h>
#include <webots/robot.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/lidar.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/radar.h>
#include <webots/range_finder.h>
#include <webots/touch_sensor.h>
#include <webots/utils/string.h>
#include <webots/vacuum_gripper.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool configured = false;
static int time_step = 0;
static bool is_hidden = false;
static double refresh_rate = 0.032;  // s
static double last_update_time = 0;  // s
static bool isDeviceTypeControlEnabled[WB_NODE_VACUUM_GRIPPER - WB_NODE_ACCELEROMETER + 1];

static WbNodeType stringToDeviceType(const char *typeString) {
  if (strcmp(typeString, "Accelerometer") == 0)
    return WB_NODE_ACCELEROMETER;
  if (strcmp(typeString, "Brake") == 0)
    return WB_NODE_BRAKE;
  if (strcmp(typeString, "Camera") == 0)
    return WB_NODE_CAMERA;
  if (strcmp(typeString, "Compass") == 0)
    return WB_NODE_COMPASS;
  if (strcmp(typeString, "Connector") == 0)
    return WB_NODE_CONNECTOR;
  if (strcmp(typeString, "Display") == 0)
    return WB_NODE_DISPLAY;
  if (strcmp(typeString, "DistanceSensor") == 0)
    return WB_NODE_DISTANCE_SENSOR;
  if (strcmp(typeString, "Emitter") == 0)
    return WB_NODE_EMITTER;
  if (strcmp(typeString, "GPS") == 0)
    return WB_NODE_GPS;
  if (strcmp(typeString, "Gyro") == 0)
    return WB_NODE_GYRO;
  if (strcmp(typeString, "InertialUnit") == 0)
    return WB_NODE_INERTIAL_UNIT;
  if (strcmp(typeString, "LED") == 0)
    return WB_NODE_LED;
  if (strcmp(typeString, "Lidar") == 0)
    return WB_NODE_LIDAR;
  if (strcmp(typeString, "LightSensor") == 0)
    return WB_NODE_LIGHT_SENSOR;
  if (strcmp(typeString, "LinearMotor") == 0)
    return WB_NODE_LINEAR_MOTOR;
  if (strcmp(typeString, "Pen") == 0)
    return WB_NODE_PEN;
  if (strcmp(typeString, "PositionSensor") == 0)
    return WB_NODE_POSITION_SENSOR;
  if (strcmp(typeString, "Propeller") == 0)
    return WB_NODE_PROPELLER;
  if (strcmp(typeString, "Radar") == 0)
    return WB_NODE_RADAR;
  if (strcmp(typeString, "RangeFinder") == 0)
    return WB_NODE_RANGE_FINDER;
  if (strcmp(typeString, "Receiver") == 0)
    return WB_NODE_RECEIVER;
  if (strcmp(typeString, "RotationalMotor") == 0)
    return WB_NODE_ROTATIONAL_MOTOR;
  if (strcmp(typeString, "Speaker") == 0)
    return WB_NODE_SPEAKER;
  if (strcmp(typeString, "TouchSensor") == 0)
    return WB_NODE_TOUCH_SENSOR;
  if (strcmp(typeString, "VacuumGripper") == 0)
    return WB_NODE_VACUUM_GRIPPER;
  return WB_NODE_NO_NODE;
}

static void enable_device(WbDeviceTag tag, bool enable) {
  const WbNodeType type = wb_device_get_node_type(tag);
  if (!isDeviceTypeControlEnabled[type - WB_NODE_ACCELEROMETER])
    return;

  const int enableRate = enable ? time_step : 0;
  switch (type) {
    case WB_NODE_ACCELEROMETER:
      wb_accelerometer_enable(tag, enableRate);
      break;
    case WB_NODE_CAMERA:
      wb_camera_enable(tag, enableRate);
      break;
    case WB_NODE_COMPASS:
      wb_compass_enable(tag, enableRate);
      break;
    case WB_NODE_DISTANCE_SENSOR:
      wb_distance_sensor_enable(tag, enableRate);
      break;
    case WB_NODE_GPS:
      wb_gps_enable(tag, enableRate);
      break;
    case WB_NODE_GYRO:
      wb_gyro_enable(tag, enableRate);
      break;
    case WB_NODE_INERTIAL_UNIT:
      wb_inertial_unit_enable(tag, enableRate);
      break;
    case WB_NODE_LIDAR:
      wb_lidar_enable(tag, enableRate);
      break;
    case WB_NODE_LIGHT_SENSOR:
      wb_light_sensor_enable(tag, enableRate);
      break;
    case WB_NODE_POSITION_SENSOR:
      wb_position_sensor_enable(tag, enableRate);
      break;
    case WB_NODE_RADAR:
      wb_radar_enable(tag, enableRate);
      break;
    case WB_NODE_RANGE_FINDER:
      wb_range_finder_enable(tag, enableRate);
      break;
    case WB_NODE_TOUCH_SENSOR:
      wb_touch_sensor_enable(tag, enableRate);
      break;
    case WB_NODE_VACUUM_GRIPPER:
      wb_vacuum_gripper_enable_presence(tag, enableRate);
      break;
    default:
      assert(0);
  }
}

// JavaScript -> C protocol description:
//   [deviceName:commandTag[=commadState][,]]
// example:
//   "e-puck:forward", "ds0:enable" or "myMotor0:value=1.2"
void wbu_generic_robot_window_parse_device_command(char *token, char *tokens) {
  WbDeviceTag tag = 0;
  bool robot = false;

  while (token && token[0] != '\0') {
    if (tag == 0 && !robot) {  // first token = device or robot name
      char *name0 = wbu_string_replace(token, "\\:", ":");
      char *name = wbu_string_replace(name0, "\\,", ",");
      if (strcmp(name, wb_robot_get_name()) == 0)
        robot = true;
      else
        tag = wb_robot_get_device(name);
      free(name);
      free(name0);
    } else if (strcmp(token, "enable") == 0)
      enable_device(tag, true);
    else if (strcmp(token, "disable") == 0)
      enable_device(tag, false);
    else if (strncmp(token, "position=", 9) == 0 && tag > 0) {
      double position = atof(&token[9]);
      wb_motor_set_position(tag, position);
    } else if (strcmp(token, "recognitionEnable") == 0)
      wb_camera_recognition_enable(tag, time_step);
    else if (strcmp(token, "recognitionDisable") == 0)
      wb_camera_recognition_disable(tag);
    else if (strcmp(token, "segmentationEnable") == 0)
      wb_camera_recognition_enable_segmentation(tag);
    else if (strcmp(token, "segmentationDisable") == 0)
      wb_camera_recognition_disable_segmentation(tag);
    else if (strcmp(token, "pointCloudEnable") == 0)
      wb_lidar_enable_point_cloud(tag);
    else if (strcmp(token, "pointCloudDisable") == 0)
      wb_lidar_disable_point_cloud(tag);
    else if (strcmp(token, "vacuumGripperTurnOn") == 0) {
      if (isDeviceTypeControlEnabled[WB_NODE_VACUUM_GRIPPER - WB_NODE_ACCELEROMETER])
        wb_vacuum_gripper_turn_on(tag);
    } else if (strcmp(token, "vacuumGripperTurnOff") == 0) {
      if (isDeviceTypeControlEnabled[WB_NODE_VACUUM_GRIPPER - WB_NODE_ACCELEROMETER])
        wb_vacuum_gripper_turn_off(tag);
    } else
      assert(0);  // protocol issue

    token = wbu_string_strsep(&tokens, ":");
  };
}

bool wbu_generic_robot_window_parse_device_control_command(char *first_token, char *tokens) {
  if (strcmp(first_token, "device-control-mode") != 0)
    return false;
  WbNodeType type = WB_NODE_NO_NODE;

  char *token = NULL;
  while ((token = wbu_string_strsep(&tokens, ":"))) {
    if (type == WB_NODE_NO_NODE) {
      type = stringToDeviceType(token);
      if (type == WB_NODE_NO_NODE) {
        assert(0);  // protocol issue
        return false;
      }
    } else {
      if (strcmp(token, "1") == 0) {
        isDeviceTypeControlEnabled[type - WB_NODE_ACCELEROMETER] = true;
        return true;
      } else if (strcmp(token, "0") == 0) {
        isDeviceTypeControlEnabled[type - WB_NODE_ACCELEROMETER] = false;
        return true;
      }
    }
  }
  assert(0);  // protocol issue
  return false;
}

void wbu_generic_robot_window_init() {
  time_step = (int)wb_robot_get_basic_time_step();

  const int device_size = WB_NODE_VACUUM_GRIPPER - WB_NODE_ACCELEROMETER + 1;
  for (int i = 0; i < device_size; ++i)
    isDeviceTypeControlEnabled[i] = false;
}

bool wbu_generic_robot_window_handle_messages(const char *message) {
  if (strncmp(message, "configure", 9) == 0) {
    int max_image_height = -1;
    int max_image_width = -1;
    int hidden = 1;
    if (sscanf(message, "configure { \"imageMaxWidth\": %d, \"imageMaxHeight\": %d, \"hidden\": %d }", &max_image_width,
               &max_image_height, &hidden) != 3) {
      fprintf(stderr, "Wrong 'configure' message received from the robot window.\n");
      assert(0);
      return false;
    }
    is_hidden = hidden == 0 ? false : true;
    wbu_default_robot_window_set_images_max_size(max_image_width, max_image_height);
    wbu_default_robot_window_configure();
    configured = true;
    return true;
  } else if (strncmp(message, "refresh-rate", 12) == 0) {
    int rate = -1;
    if (sscanf(message, "refresh-rate %d", &rate) != 1) {
      fprintf(stderr, "Wrong 'refresh-rate' message received from the robot window.\n");
      assert(0);
      return false;
    }
    refresh_rate = 0.001 * rate;
    return true;
  } else if (strncmp(message, "window", 6) == 0) {
    is_hidden = strstr(message, "hidden") != NULL;
    return true;
  }
  return false;
}

void wbu_generic_robot_window_update() {
  last_update_time = wb_robot_get_time();
  wbu_default_robot_window_update();
}

bool wbu_generic_robot_window_is_hidden() {
  return is_hidden;
}

double wbu_generic_robot_window_refresh_rate() {
  return refresh_rate;
}

bool wbu_generic_robot_window_needs_update() {
  return configured && !is_hidden && (refresh_rate == 0 || (wb_robot_get_time() - last_update_time) > refresh_rate);
}
