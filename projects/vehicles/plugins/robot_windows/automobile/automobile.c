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

#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "string_utils.h"

static int time_step = 0;
static double max_speed = 0.0;
clock_t last_update_time = 0;

enum DriverInfo { NONE = 0, OVERVIEW_INFO = 1, SPEED_INFO, STEERING_INFO, ENCODERS_INFO, BRAKE_INFO, THROTTLE_INFO, RPM_INFO, COUNT };
static bool driverInfoEnabled[COUNT];

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
  // if (buffer == NULL)
  //  throw_realloc_error();
  memcpy(&buffer[buffer_size - 1], string, l + 1);
  buffer_size += l;
}

static void enable_driver_info(enum DriverInfo info, bool enable) {
  if (info <= NONE || info >= COUNT) {
    assert(0);
    return;
  }
  driverInfoEnabled[info] = enable;
}

// JavaScript -> C protocol description:
//   [deviceName:commandTag[=commadState][,]]*
// example:
//   "e-puck:forward,ds0:enable,myMotor0:value=1.2"
static void apply_command(const char *command) {
  char *tokens = strdup(command);
  char *token = NULL;

  enum DriverInfo info = NONE;
  WbDeviceTag tag = 0;
  bool robot = false;

  while ((token = string_utils_strsep(&tokens, ":"))) {
    if (info == 0 && tag == 0 && !robot) {  // first token = device or robot name
      char *name0 = string_utils_replace(token, "\\:", ":");
      char *name = string_utils_replace(name0, "\\,", ",");
      if (strcmp(name, wb_robot_get_name()) == 0)
        robot = true;
      else {
        info = driver_info_from_string(name);
        if (info == NONE)
          tag = wb_robot_get_device(name);
      }
      fprintf(stderr, "info %d tag %d\n", info, tag);
      free(name);
      free(name0);
    } else if (strcmp(token, "enable") == 0) {
      if (info != NONE)
        enable_driver_info(info, true);
      else
        enable_device(tag, true);
    } else if (strcmp(token, "disable") == 0) {
      if (tag)
        enable_device(tag, false);
      else
        enable_driver_info(info, false);
    } else if (strcmp(token, "stop") == 0 && robot)
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
      
      // send vehicle config data
      char buf[32];
      buffer_append("configure-vehicle { \"front-track\": ");
      snprintf(buf, 32, "%.4g", wbu_car_get_track_front());
      buffer_append(buf);
      buffer_append(",\"rear-track\": ");
      snprintf(buf, 32, "%.4g", wbu_car_get_track_rear());
      buffer_append(buf);
      buffer_append(",\"wheel-base\": ");
      snprintf(buf, 32, "%.4g", wbu_car_get_wheelbase());
      buffer_append(buf);
      buffer_append(",\"front-wheel-radius\": ");
      snprintf(buf, 32, "%.4g", wbu_car_get_front_wheel_radius());
      buffer_append(buf);
      buffer_append(",\"rear-wheel-radius\": ");
      snprintf(buf, 32, "%.4g", wbu_car_get_rear_wheel_radius());
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
        buffer_append("\"Epower-split hybrid\"");
      else
        buffer_append("\"unknown\"");
      buffer_append("}");
      wb_robot_wwi_send_text(buffer);
      free_buffer();      
      
      configured = true;
    } else
      apply_commands(message);
  }

  if (!configured)
    return;

  wbu_default_robot_window_update();
  
  clock_t current_time = clock();
  double time_spent = (double)(current_time - last_update_time) / CLOCKS_PER_SEC;
  if (last_update_time != 0 && time_spent < 0.004)
    return;
  last_update_time = current_time;
  bool vehicle_message = false;
  char buf[32];
  buffer_append("update { \"vehicle\": {");
  buffer_append("\"control-mode\": ");
  WbuDriverControlMode control_mode = wbu_driver_get_control_mode();
  snprintf(buf, 32, "%d", control_mode);
  buffer_append(buf);
  if (driverInfoEnabled[SPEED_INFO]) {
    buffer_append(",\"speed\": {");
    if (control_mode == SPEED) {
      buffer_append("\"cruising-speed\": ");
      snprintf(buf, 32, "%.17g,", wbu_driver_get_target_cruising_speed());
      buffer_append(buf);
    }
    buffer_append("\"update\":[{\"time\":");
    snprintf(buf, 32, "%.17g", wb_robot_get_time());
    buffer_append(buf);
    buffer_append(",\"value\":");
    snprintf(buf, 32, "%.17g", wbu_driver_get_current_speed());
    buffer_append(buf);
    buffer_append("}]}");
    vehicle_message = true;
  }

  if (driverInfoEnabled[STEERING_INFO]) {
    buffer_append(", \"steering\": {\"time\": ");
    snprintf(buf, 32, "%.17g", wb_robot_get_time());
    buffer_append(buf);
    buffer_append(",\"value\": [");
    snprintf(buf, 32, "%.17g,", wbu_driver_get_steering_angle());
    buffer_append(buf);
    snprintf(buf, 32, "%.17g,", wbu_car_get_right_steering_angle());
    buffer_append(buf);
    snprintf(buf, 32, "%.17g", wbu_car_get_right_steering_angle());
    buffer_append(buf);
    buffer_append("]}");
    vehicle_message = true;
  }
  if (driverInfoEnabled[THROTTLE_INFO]  && control_mode == TORQUE) {
    buffer_append(", \"throttle\": {\"time\": ");
    snprintf(buf, 32, "%.17g", wb_robot_get_time());
    buffer_append(buf);
    buffer_append(",\"value\": ");
    snprintf(buf, 32, "%.17g", wbu_driver_get_throttle());
    buffer_append(buf);
    buffer_append("}");
    vehicle_message = true;
  }
  if (driverInfoEnabled[RPM_INFO] && control_mode == TORQUE) {
    buffer_append(", \"rpm\": {\"time\": ");
    snprintf(buf, 32, "%.17g", wb_robot_get_time());
    buffer_append(buf);
    buffer_append(",\"value\": ");
    snprintf(buf, 32, "%.17g", wbu_driver_get_rpm());
    buffer_append(buf);
    buffer_append("}");
    vehicle_message = true;
  }
  if (driverInfoEnabled[BRAKE_INFO]) {
    buffer_append(", \"brake\": {\"time\": ");
    snprintf(buf, 32, "%.17g", wb_robot_get_time());
    buffer_append(buf);
    buffer_append(",\"value\": ");
    snprintf(buf, 32, "%.17g", wbu_driver_get_brake_intensity());
    buffer_append(buf);
    buffer_append("}");
    vehicle_message = true;
  }
  if (driverInfoEnabled[ENCODERS_INFO]) {
    double encoder[WBU_CAR_WHEEL_NB];
    for (int i = WBU_CAR_WHEEL_FRONT_RIGHT; i < WBU_CAR_WHEEL_NB; ++i)
      encoder[i] = wbu_car_get_wheel_encoder(i);
    buffer_append(", \"encoders\": {\"time\": ");
    snprintf(buf, 32, "%.17g", wb_robot_get_time());
    buffer_append(buf);
    buffer_append(",\"value\": [");
    snprintf(buf, 32, "%.17g,", encoder[WBU_CAR_WHEEL_FRONT_RIGHT]);
    buffer_append(buf);
    snprintf(buf, 32, "%.17g,", encoder[WBU_CAR_WHEEL_FRONT_LEFT]);
    buffer_append(buf);
    snprintf(buf, 32, "%.17g,", encoder[WBU_CAR_WHEEL_REAR_RIGHT]);
    buffer_append(buf);
    snprintf(buf, 32, "%.17g", encoder[WBU_CAR_WHEEL_REAR_LEFT]);
    buffer_append(buf);
    buffer_append("]}");
    vehicle_message = true;
  }
  if (driverInfoEnabled[OVERVIEW_INFO]) {
    // TODO check refresh rate
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
      buffer_append("],\"target-speed\": 0");
    } else {
      buffer_append("],\"rpm\": 0, \"gearbox\": 0");
      buffer_append(",\"target-speed\": ");
      snprintf(buf, 32, "%.17g", wbu_driver_get_target_cruising_speed());
      buffer_append(buf);
    }
    for (int i = WBU_CAR_WHEEL_FRONT_RIGHT; i < WBU_CAR_WHEEL_NB; ++i) {
      buffer_append(",\"wheel");
      snprintf(buf, 32, "%d", i+1);
      buffer_append(buf);
      buffer_append("\": { \"speed\": ");
      snprintf(buf, 32, "%.17g", wbu_car_get_wheel_speed(i));
      buffer_append(buf);
      buffer_append(", \"encoder\": ");
      snprintf(buf, 32, "%.17g", wbu_car_get_wheel_encoder(i));
      buffer_append(buf);
      buffer_append("}");
    }
    buffer_append("}");
    vehicle_message = true;
  }
  buffer_append("}}");
  if (vehicle_message)
    wb_robot_wwi_send_text(buffer);
  free_buffer();
}

void wb_robot_window_cleanup() {
}
