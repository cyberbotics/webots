#include <webots/device.h>
#include <webots/robot.h>
#include <webots/robot_wwi.h>
#include <webots/utils/default_robot_window.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/differential_wheels.h>
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

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "string_utils.h"

static int time_step = 0;
static double max_speed = 0.0;

static void enable_device(WbDeviceTag tag, bool enable) {
  WbNodeType type = wb_device_get_node_type(tag);
  int enableRate = enable ? time_step : 0;
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
    default:
      assert(0);
  }
}

// javascript -> C protocol description:
//   [deviceName:commandTag[=commadState][,]]*
// example:
//   "e-puck:forward,ds0:enable,myMotor0:value=1.2"
static void apply_command(const char *command) {
  char *tokens = strdup(command);
  char *token = NULL;

  WbDeviceTag tag = 0;
  bool robot = false;

  while ((token = string_utils_strsep(&tokens, ":"))) {
    if (tag == 0 && !robot) {  // first token = device or robot name
      char *name0 = string_utils_replace(token, "\\:", ":");
      char *name = string_utils_replace(name0, "\\,", ",");
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
    else if (strcmp(token, "stop") == 0 && robot)
      wb_differential_wheels_set_speed(0.0, 0.0);
    else if (strcmp(token, "forward") == 0 && robot)
      wb_differential_wheels_set_speed(max_speed, max_speed);
    else if (strcmp(token, "backward") == 0 && robot)
      wb_differential_wheels_set_speed(-max_speed, -max_speed);
    else if (strcmp(token, "left") == 0 && robot)
      wb_differential_wheels_set_speed(-0.5 * max_speed, 0.5 * max_speed);
    else if (strcmp(token, "left_forward") == 0 && robot)
      wb_differential_wheels_set_speed(0.5 * max_speed, max_speed);
    else if (strcmp(token, "left_backward") == 0 && robot)
      wb_differential_wheels_set_speed(-0.5 * max_speed, -max_speed);
    else if (strcmp(token, "right") == 0 && robot)
      wb_differential_wheels_set_speed(0.5 * max_speed, -0.5 * max_speed);
    else if (strcmp(token, "right_forward") == 0 && robot)
      wb_differential_wheels_set_speed(max_speed, 0.5 * max_speed);
    else if (strcmp(token, "right_backward") == 0 && robot)
      wb_differential_wheels_set_speed(-max_speed, -0.5 * max_speed);
    else if (strncmp(token, "position=", 9) == 0 && tag > 0) {
      double position = atof(&token[9]);
      wb_motor_set_position(tag, position);
    } else if (strcmp(token, "pointCloudEnable") == 0)
      wb_lidar_enable_point_cloud(tag);
    else if (strcmp(token, "pointCloudDisable") == 0)
      wb_lidar_disable_point_cloud(tag);
    else
      assert(0);  // protocol issue
  }
}

static void apply_commands(const char *commands) {
  char *tokens = strdup(commands);
  char *token = NULL;
  while ((token = string_utils_strsep(&tokens, ",")))
    apply_command(token);
}

void wb_robot_window_init() {
  time_step = (int)wb_robot_get_basic_time_step();
  WbNodeType robot_type = wb_device_get_node_type(0);
  if (robot_type == WB_NODE_DIFFERENTIAL_WHEELS)
    max_speed = wb_differential_wheels_get_max_speed() / wb_differential_wheels_get_speed_unit();
}

void wb_robot_window_step(int time_step) {
  static bool configured = false;
  const char *message = wb_robot_wwi_receive_text();
  if (message) {
    if (strncmp(message, "configure", 9) == 0) {
      int max_image_height = -1;
      int max_image_width = -1;
      if (sscanf(message, "configure { \"imageMaxWidth\": %d, \"imageMaxHeight\": %d }", &max_image_width, &max_image_height) !=
          2) {
        fprintf(stderr, "Wrong 'configure' message received from the robot window.\n");
        assert(0);
        return;
      }
      wbu_default_robot_window_set_images_max_size(max_image_width, max_image_height);
      wbu_default_robot_window_configure();
      configured = true;
    } else
      apply_commands(message);
  }

  if (!configured)
    return;

  wbu_default_robot_window_update();
}

void wb_robot_window_cleanup() {
}
