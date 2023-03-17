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

// This module implements utility functions for managing the default HTML
// robot windows. These functions can be re-used for implementing specific
// user robot windows.
// Public functions are prefixed with wbu_default_robot_window_
// This module is not yet complete: only a few devices are implemented.

#include <webots/device.h>
#include <webots/plugins/robot_window/default.h>

#include <webots/accelerometer.h>
#include <webots/altimeter.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/lidar.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/position_sensor.h>
#include <webots/radar.h>
#include <webots/range_finder.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/touch_sensor.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "base64.h"
#include "g_image.h"
#include "robot_private.h"

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

// Buffer management.

static char *buffer = NULL;
static int buffer_size = 0;
static int images_max_width = -1;
static int images_max_height = -1;

static void free_buffer() {
  free(buffer);
  buffer = NULL;
}

static void create_buffer(int size) {
  atexit(free_buffer);
  buffer_size = size + 1;
  buffer = malloc(buffer_size);
}

static void throw_realloc_error() {
  fprintf(stderr, "Error creating message to be sent to the robot window: not enough memory.\n");
  exit(EXIT_FAILURE);
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
  if (buffer == NULL)
    throw_realloc_error();
  memcpy(&buffer[buffer_size - 1], string, l + 1);
  buffer_size += l;
}

static void buffer_append_escaped_string(const char *string) {
  if (string == NULL || string[0] == '\0')
    return;
  const int string_length = strlen(string);
  int i, count = 0;
  for (i = 0; i < string_length; ++i) {
    if (string[i] == '"')
      count++;
  }

  const int l = string_length + count * 5;
  if (buffer == NULL) {
    create_buffer(l);
    if (count == 0) {
      memcpy(buffer, string, buffer_size);
      return;
    }
  } else {
    buffer = realloc(buffer, buffer_size + l);
    if (buffer == NULL)
      throw_realloc_error();
    if (count == 0) {
      memcpy(&buffer[buffer_size - 1], string, l + 1);
      buffer_size += l;
      return;
    }
    buffer_size += l;
  }

  // copy and escape special characters
  int j = buffer_size - l - 1;
  for (i = 0; i < string_length; ++i) {
    if (string[i] == '"') {
      buffer[j++] = '&';
      buffer[j++] = 'q';
      buffer[j++] = 'u';
      buffer[j++] = 'o';
      buffer[j++] = 't';
      buffer[j++] = ';';
    } else
      buffer[j++] = string[i];
  }
}

static void buffer_append_double(double d) {
  char buf[32];
  snprintf(buf, 32, "%.17g", d);  // so that we don't loose any precision
  buffer_append(buf);
}

static void buffer_append_int(int i) {
  char buf[32];
  snprintf(buf, 32, "%d", i);
  buffer_append(buf);
}

// Update structure management.

static double update_period_by_type(WbNodeType type) {
  switch (type) {
    case WB_NODE_CAMERA:
    case WB_NODE_LIDAR:
    case WB_NODE_RANGE_FINDER:
      return 0.1;  // 100ms
    default:
      return 0.04;  // 40ms
  }
}

static double number_of_components(WbDeviceTag tag) {
  WbNodeType type = wb_device_get_node_type(tag);
  switch (type) {
    case WB_NODE_ALTIMETER:
    case WB_NODE_DISTANCE_SENSOR:
    case WB_NODE_POSITION_SENSOR:
    case WB_NODE_LIGHT_SENSOR:
    case WB_NODE_LINEAR_MOTOR:
    case WB_NODE_ROTATIONAL_MOTOR:
      return 1;
    case WB_NODE_ACCELEROMETER:
    case WB_NODE_COMPASS:
    case WB_NODE_GPS:
    case WB_NODE_GYRO:
    case WB_NODE_INERTIAL_UNIT:
      return 3;
    case WB_NODE_TOUCH_SENSOR:
      return wb_touch_sensor_get_type(tag) == WB_TOUCH_SENSOR_FORCE3D ? 3 : 1;
    default:
      return 0;
  }
}

struct UpdateElement {
  double last_update;

  int n_values;
  int n_components;  // 1 for 1D value, 3 for 3D values, etc.
  double **values;
  double *times;
};
static unsigned int number_of_update_elements = 0;
static struct UpdateElement **update_elements = NULL;

static void ue_init(struct UpdateElement *ue, int n_components) {
  ue->last_update = 0.0;
  ue->n_values = 0;
  ue->n_components = n_components;
  ue->values = NULL;
  ue->times = NULL;
}

static void ue_append(struct UpdateElement *ue, double update_time, const double *value) {
  if (value == NULL)
    return;

  const int last_index = ue->n_values;
  ue->n_values++;
  if (ue->values == NULL)
    ue->values = (double **)malloc(sizeof(double *));
  else
    ue->values = (double **)realloc(ue->values, ue->n_values * sizeof(double *));
  if (ue->times == NULL)
    ue->times = (double *)malloc(sizeof(double));
  else
    ue->times = (double *)realloc(ue->times, ue->n_values * sizeof(double));
  ue->values[last_index] = malloc(ue->n_components * sizeof(double *));
  ue->times[last_index] = update_time;
  int v;
  for (v = 0; v < ue->n_components; ++v)
    // cppcheck-suppress objectIndex
    ue->values[last_index][v] = value[v];
}

static int ue_number_of_values(struct UpdateElement *ue) {
  return ue->n_values;
}

static double *ue_value_at(struct UpdateElement *ue, int v) {
  assert(v < ue->n_values);
  return ue->values[v];
}

static double ue_time_at(struct UpdateElement *ue, int v) {
  assert(v < ue->n_values);
  return ue->times[v];
}

static void ue_clear(struct UpdateElement *ue) {
  int v;
  for (v = 0; v < ue_number_of_values(ue); ++v)
    free(ue->values[v]);
  free(ue->values);
  free(ue->times);
  ue->values = NULL;
  ue->times = NULL;
  ue->n_values = 0;
}

static void ue_write_values(struct UpdateElement *ue) {
  if (ue_number_of_values(ue) == 0)
    return;
  int v, c;
  buffer_append("\"update\":[");
  for (v = 0; v < ue_number_of_values(ue); ++v) {
    if (v != 0)
      buffer_append(",");
    buffer_append("{");
    buffer_append("\"time\":");
    buffer_append_double(ue_time_at(ue, v));
    buffer_append(",\"value\":");
    if (ue->n_components == 1) {
      double value = *ue_value_at(ue, v);
      if (isinf(value)) {
        if (value < 0)
          buffer_append("\"-Inf\"");
        else
          buffer_append("\"Inf\"");
      } else if (isnan(value))
        buffer_append("\"NaN\"");
      else
        buffer_append_double(value);
    } else {
      double *values = ue_value_at(ue, v);
      buffer_append("[");
      for (c = 0; c < ue->n_components; ++c) {
        if (c != 0)
          buffer_append(",");
        double value = values[c];
        if (isinf(value)) {
          if (value < 0)
            buffer_append("\"-Inf\"");
          else
            buffer_append("\"Inf\"");
        } else if (isnan(value))
          buffer_append("\"NaN\"");
        else
          buffer_append_double(value);
      }
      buffer_append("]");
    }
    buffer_append("}");
  }
  buffer_append("]");
  ue_clear(ue);
}

static void initialize_update_elements() {
  const int number_of_devices = wb_robot_get_number_of_devices();
  if (update_elements) {
    assert(number_of_devices > number_of_update_elements);
    update_elements = realloc(update_elements, number_of_devices * sizeof(struct UpdateElement));
    if (!update_elements) {
      fprintf(stderr, "Error reinitializing list of devices.\n");
      exit(EXIT_FAILURE);
    }
  } else
    update_elements = malloc(number_of_devices * sizeof(struct UpdateElement));

  int i;
  for (i = number_of_update_elements; i < number_of_devices; ++i) {
    WbDeviceTag tag = wb_robot_get_device_by_index(i);
    struct UpdateElement *update_element = malloc(sizeof(struct UpdateElement));
    ue_init(update_element, number_of_components(tag));
    update_elements[i] = update_element;
  }
  number_of_update_elements = wb_robot_get_number_of_devices();
}

void default_robot_window_cleanup() {
  int i;
  for (i = 0; i < number_of_update_elements; ++i)
    ue_clear(update_elements[i]);
  free(update_elements);
  update_elements = NULL;
}

// Device configure functions.

static void camera_configure(WbDeviceTag tag) {
  buffer_append(",\"width\":");
  buffer_append_int(wb_camera_get_width(tag));
  buffer_append(",\"height\":");
  buffer_append_int(wb_camera_get_height(tag));
  buffer_append(",\"recognition\":");
  buffer_append_int(wb_camera_has_recognition(tag));
  buffer_append(",\"segmentation\":");
  buffer_append_int(wb_camera_recognition_has_segmentation(tag));
}

static void lidar_configure(WbDeviceTag tag) {
  buffer_append(",\"width\":");
  buffer_append_int(wb_lidar_get_horizontal_resolution(tag));
  buffer_append(",\"height\":");
  buffer_append_int(wb_lidar_get_number_of_layers(tag));
}

static void range_finder_configure(WbDeviceTag tag) {
  buffer_append(",\"width\":");
  buffer_append_int(wb_range_finder_get_width(tag));
  buffer_append(",\"height\":");
  buffer_append_int(wb_range_finder_get_height(tag));
}

static void distance_sensor_configure(WbDeviceTag tag) {
  buffer_append(",\"sensorType\":\"");
  switch (wb_distance_sensor_get_type(tag)) {
    case WB_DISTANCE_SENSOR_GENERIC:
      buffer_append("generic");
      break;
    case WB_DISTANCE_SENSOR_INFRA_RED:
      buffer_append("infra-red");
      break;
    case WB_DISTANCE_SENSOR_LASER:
      buffer_append("laser");
      break;
    case WB_DISTANCE_SENSOR_SONAR:
      buffer_append("sonar");
      break;
    default:
      buffer_append("unknown");
      break;
  }
  buffer_append("\",\"minValue\":");
  buffer_append_double(wb_distance_sensor_get_min_value(tag));
  buffer_append(",\"maxValue\":");
  buffer_append_double(wb_distance_sensor_get_max_value(tag));
  buffer_append(",\"aperture\":");
  buffer_append_double(wb_distance_sensor_get_aperture(tag));
}

static void motor_configure(WbDeviceTag tag) {
  buffer_append(",\"minPosition\":");
  buffer_append_double(wb_motor_get_min_position(tag));
  buffer_append(",\"maxPosition\":");
  buffer_append_double(wb_motor_get_max_position(tag));
  buffer_append(",\"maxVelocity\":");
  buffer_append_double(wb_motor_get_max_velocity(tag));
  switch (wb_device_get_node_type(tag)) {
    case WB_NODE_ROTATIONAL_MOTOR:
      buffer_append(",\"maxTorque\":");
      buffer_append_double(wb_motor_get_max_torque(tag));
      break;
    case WB_NODE_LINEAR_MOTOR:
      buffer_append(",\"maxForce\":");
      buffer_append_double(wb_motor_get_max_force(tag));
      break;
    default:
      assert(0);
  }
}

static void radar_configure(WbDeviceTag tag) {
  buffer_append(",\"fieldOfView\":");
  buffer_append_double(wb_radar_get_horizontal_fov(tag));
  buffer_append(",\"minRange\":");
  buffer_append_double(wb_radar_get_min_range(tag));
  buffer_append(",\"maxRange\":");
  buffer_append_double(wb_radar_get_max_range(tag));
}

static void touch_sensor_configure(WbDeviceTag tag) {
  buffer_append(",\"sensorType\":\"");
  switch (wb_touch_sensor_get_type(tag)) {
    case WB_TOUCH_SENSOR_BUMPER:
      buffer_append("bumper");
      break;
    case WB_TOUCH_SENSOR_FORCE:
      buffer_append("force");
      break;
    case WB_TOUCH_SENSOR_FORCE3D:
      buffer_append("force-3d");
      break;
    default:
      assert(0);
  }
  buffer_append("\"");
}

void wbu_default_robot_window_configure() {
  buffer_append("configure {\"name\":\"");
  buffer_append_escaped_string(wb_robot_get_name());
  buffer_append("\",\"model\":\"");
  buffer_append_escaped_string(wb_robot_get_model());
  buffer_append("\",\"basicTimeStep\":");
  buffer_append_double(0.001 * wb_robot_get_basic_time_step());
  int n = wb_robot_get_number_of_devices();
  if (n) {
    buffer_append(",\"devices\":[");
    int i;
    for (i = 0; i < n; ++i) {
      WbDeviceTag tag = wb_robot_get_device_by_index(i);
      if (i > 0)
        buffer_append(",");
      buffer_append("{\"type\":\"");
      WbNodeType type = wb_device_get_node_type(tag);
      buffer_append_escaped_string(wb_node_get_name(type));
      buffer_append("\",\"name\":\"");
      buffer_append_escaped_string(wb_device_get_name(tag));
      buffer_append("\",\"model\":\"");
      buffer_append_escaped_string(wb_device_get_model(tag));
      buffer_append("\"");
      switch (type) {
        case WB_NODE_CAMERA:
          camera_configure(tag);
          break;
        case WB_NODE_DISTANCE_SENSOR:
          distance_sensor_configure(tag);
          break;
        case WB_NODE_LIDAR:
          lidar_configure(tag);
          break;
        case WB_NODE_RADAR:
          radar_configure(tag);
          break;
        case WB_NODE_RANGE_FINDER:
          range_finder_configure(tag);
          break;
        case WB_NODE_ROTATIONAL_MOTOR:
        case WB_NODE_LINEAR_MOTOR:
          motor_configure(tag);
          break;
        case WB_NODE_TOUCH_SENSOR:
          touch_sensor_configure(tag);
          break;
        default:
          break;
      }
      buffer_append("}");
    }
    buffer_append("]");
  }
  buffer_append("}");
  wb_robot_wwi_send_text(buffer);
  free_buffer();
}

static void append_rescaled_image_to_buffer_and_free_data(GImage *img, int new_width, int new_height, float max_range) {
  unsigned char *jpeg_data = NULL;
  char *base64_data = NULL;

  // 1. Downscale image.
  int success = g_image_downscale(img, new_width, new_height, max_range);
  if (img->data == NULL || success != 0)
    assert(0);
  else {
    // 2. Convert to JPEG.
    unsigned long jpeg_data_size = 0;
    success = g_image_save_to_jpeg_buffer(img, &jpeg_data, &jpeg_data_size, 100);
    if (jpeg_data == NULL || success != 0)
      assert(0);
    else {
      // 3. Convert to base64.
      size_t output_length;
      base64_data = base64_encode(jpeg_data, jpeg_data_size, &output_length);
      if (base64_data == NULL)
        assert(0);
      else {
        // 4. Send to robot window.
        buffer_append("\"image\":\"data:image/jpg;base64,");
        buffer_append(base64_data);
        buffer_append("\"");
      }
    }
  }

  // 5. Cleanup
  free(jpeg_data);
  free(base64_data);
  free(img->data);
  img->data = NULL;
}

// Device update functions.

static void camera_update(WbDeviceTag tag) {
  if (wb_camera_get_sampling_period(tag) <= 0)
    return;

  if (wb_camera_has_recognition(tag)) {
    buffer_append("\"recognitionEnabled\":");
    buffer_append(wb_camera_recognition_get_sampling_period(tag) > 0 ? "true" : "false");
    if (wb_camera_recognition_has_segmentation(tag)) {
      buffer_append(",\"segmentationEnabled\":");
      buffer_append(wb_camera_recognition_is_segmentation_enabled(tag) ? "true" : "false");
    }
    buffer_append(",");
  }

  const unsigned char *image = wb_camera_get_image(tag);
  if (!image)
    return;

  int camera_width = wb_camera_get_width(tag);
  int camera_height = wb_camera_get_height(tag);

  int new_width = camera_width;
  int new_height = camera_height;

  if (images_max_width > 0) {
    if (new_width > images_max_width) {
      double ratio = (double)images_max_width / new_width;
      new_height = ratio * new_height;
      new_width = ratio * new_width;
    }
    if (new_height > images_max_height) {
      double ratio = (double)images_max_height / new_height;
      new_height = ratio * new_height;
      new_width = ratio * new_width;
    }
  }

  GImage img;
  img.width = camera_width;
  img.height = camera_height;
  img.data_format = G_IMAGE_DATA_FORMAT_BGRA;
  img.float_data = NULL;
  img.data = (unsigned char *)image;
  img.failed = 0;
  img.flipped = 0;

  append_rescaled_image_to_buffer_and_free_data(&img, new_width, new_height, 0.0f);
}

static void lidar_update(WbDeviceTag tag) {
  buffer_append("\"cloudPointEnabled\":");
  buffer_append(wb_lidar_is_point_cloud_enabled(tag) ? "true" : "false");

  if (wb_lidar_get_sampling_period(tag) <= 0)
    return;

  const float *image = wb_lidar_get_range_image(tag);
  if (!image)
    return;

  buffer_append(",");

  int lidar_width = wb_lidar_get_horizontal_resolution(tag);
  int lidar_height = wb_lidar_get_number_of_layers(tag);
  int new_width = lidar_width;
  int new_height = lidar_height;

  if (images_max_width > 0) {
    // assumption: no need to rescale along the height, because nLayers is small enough.
    assert(lidar_height < images_max_height);
    new_width = MIN(lidar_width, images_max_width);
  }

  GImage img;
  img.width = lidar_width;
  img.height = lidar_height;
  img.data_format = G_IMAGE_DATA_FORMAT_F;
  img.float_data = (float *)image;
  img.data = NULL;
  img.failed = 0;
  img.flipped = 0;

  append_rescaled_image_to_buffer_and_free_data(&img, new_width, new_height, wb_lidar_get_max_range(tag));
}

static void range_finder_update(WbDeviceTag tag) {
  if (wb_range_finder_get_sampling_period(tag) <= 0)
    return;

  const float *image = wb_range_finder_get_range_image(tag);
  if (!image)
    return;

  int range_finder_width = wb_range_finder_get_width(tag);
  int range_finder_height = wb_range_finder_get_height(tag);

  int new_width = range_finder_width;
  int new_height = range_finder_height;

  if (images_max_width > 0) {
    if (new_width > images_max_width) {
      double ratio = (double)images_max_width / new_width;
      new_height = ratio * new_height;
      new_width = ratio * new_width;
    }
    if (new_height > images_max_height) {
      double ratio = (double)images_max_height / new_height;
      new_height = ratio * new_height;
      new_width = ratio * new_width;
    }
  }

  GImage img;
  img.width = range_finder_width;
  img.height = range_finder_height;
  img.data_format = G_IMAGE_DATA_FORMAT_F;
  img.float_data = (float *)image;
  img.data = NULL;
  img.failed = 0;
  img.flipped = 0;

  append_rescaled_image_to_buffer_and_free_data(&img, new_width, new_height, wb_range_finder_get_max_range(tag));
}

static void accelerometer_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  if (wb_accelerometer_get_sampling_period(tag) <= 0)
    return;
  const double *values = wb_accelerometer_get_values(tag);
  ue_append(ue, update_time, values);
}

static void altimeter_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  if (wb_altimeter_get_sampling_period(tag) <= 0)
    return;
  const double value = wb_altimeter_get_value(tag);
  ue_append(ue, update_time, &value);
}

static void compass_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  if (wb_compass_get_sampling_period(tag) <= 0)
    return;
  const double *values = wb_compass_get_values(tag);
  ue_append(ue, update_time, values);
}

static void distance_sensor_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  if (wb_distance_sensor_get_sampling_period(tag) <= 0)
    return;
  double value = wb_distance_sensor_get_value(tag);
  ue_append(ue, update_time, &value);
}

static void gps_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  if (wb_gps_get_sampling_period(tag) <= 0)
    return;
  const double *values = wb_gps_get_values(tag);
  ue_append(ue, update_time, values);
}

static void gyro_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  if (wb_gyro_get_sampling_period(tag) <= 0)
    return;
  const double *values = wb_gyro_get_values(tag);
  ue_append(ue, update_time, values);
}

static void inertial_unit_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  if (wb_inertial_unit_get_sampling_period(tag) <= 0)
    return;
  const double *values = wb_inertial_unit_get_roll_pitch_yaw(tag);
  ue_append(ue, update_time, values);
}

static void light_sensor_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  if (wb_light_sensor_get_sampling_period(tag) <= 0)
    return;
  double value = wb_light_sensor_get_value(tag);
  ue_append(ue, update_time, &value);
}

static void motor_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  double value = wb_motor_get_target_position(tag);
  ue_append(ue, update_time, &value);
}

static void position_sensor_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  if (wb_position_sensor_get_sampling_period(tag) <= 0)
    return;
  double value = wb_position_sensor_get_value(tag);
  ue_append(ue, update_time, &value);
}

static void radar_update(WbDeviceTag tag) {
  if (wb_radar_get_sampling_period(tag) <= 0)
    return;
  buffer_append("\"targets\":[");
  int t;
  for (t = 0; t < wb_radar_get_number_of_targets(tag); ++t) {
    if (t != 0)
      buffer_append(",");
    buffer_append("{\"distance\":");
    buffer_append_double(wb_radar_get_targets(tag)[t].distance);
    buffer_append(",\"azimuth\":");
    buffer_append_double(wb_radar_get_targets(tag)[t].azimuth);
    buffer_append("}");
  }
  buffer_append("]");
}

static void touch_sensor_collect_value(WbDeviceTag tag, struct UpdateElement *ue, double update_time) {
  if (wb_touch_sensor_get_sampling_period(tag) <= 0)
    return;
  if (wb_touch_sensor_get_type(tag) == WB_TOUCH_SENSOR_FORCE3D) {
    const double *values = wb_touch_sensor_get_values(tag);
    ue_append(ue, update_time, values);
  } else {
    double value = wb_touch_sensor_get_value(tag);
    ue_append(ue, update_time, &value);
  }
}

void wbu_default_robot_window_update() {
  if (buffer != NULL)
    return;  // prevent to mix 2 updates.

  const int n = wb_robot_get_number_of_devices();
  if (update_elements == NULL || n > number_of_update_elements) {
    const bool new_devices = update_elements != NULL;
    initialize_update_elements();
    if (new_devices)
      wbu_default_robot_window_configure();
  }

  buffer_append("update {");
  const double simulated_time = wb_robot_get_time();
  buffer_append("\"time\":");
  buffer_append_double(simulated_time);
  if (n) {
    buffer_append(",\"devices\":{");
    int i;
    int updated_device = 0;
    for (i = 0; i < n; ++i) {
      const WbDeviceTag tag = wb_robot_get_device_by_index(i);
      const WbNodeType type = wb_device_get_node_type(tag);
      struct UpdateElement *update_element = update_elements[i];

      // store values to be sent later if required.
      switch (type) {
        case WB_NODE_ROTATIONAL_MOTOR:
        case WB_NODE_LINEAR_MOTOR:
          motor_collect_value(tag, update_element, simulated_time);
          break;
        case WB_NODE_ACCELEROMETER:
          accelerometer_collect_value(tag, update_element, simulated_time);
          break;
        case WB_NODE_ALTIMETER:
          altimeter_collect_value(tag, update_element, simulated_time);
          break;
        case WB_NODE_COMPASS:
          compass_collect_value(tag, update_element, simulated_time);
          break;
        case WB_NODE_DISTANCE_SENSOR:
          distance_sensor_collect_value(tag, update_element, simulated_time);
          break;
        case WB_NODE_GPS:
          gps_collect_value(tag, update_element, simulated_time);
          break;
        case WB_NODE_GYRO:
          gyro_collect_value(tag, update_element, simulated_time);
          break;
        case WB_NODE_INERTIAL_UNIT:
          inertial_unit_collect_value(tag, update_element, simulated_time);
          break;
        case WB_NODE_LIGHT_SENSOR:
          light_sensor_collect_value(tag, update_element, simulated_time);
          break;
        case WB_NODE_POSITION_SENSOR:
          position_sensor_collect_value(tag, update_element, simulated_time);
          break;
        case WB_NODE_TOUCH_SENSOR:
          touch_sensor_collect_value(tag, update_element, simulated_time);
          break;
        default:
          break;
      }

      if (robot_get_simulation_mode() == WB_SUPERVISOR_SIMULATION_MODE_PAUSE ||
          update_element->last_update + update_period_by_type(type) < simulated_time) {
        // send the stored values if any.
        update_element->last_update = simulated_time;
        if (updated_device > 0)
          buffer_append(",");
        ++updated_device;
        buffer_append("\"");
        buffer_append_escaped_string(wb_device_get_name(tag));
        buffer_append("\":{");
        switch (type) {
          case WB_NODE_ACCELEROMETER:
          case WB_NODE_ALTIMETER:
          case WB_NODE_COMPASS:
          case WB_NODE_DISTANCE_SENSOR:
          case WB_NODE_GPS:
          case WB_NODE_GYRO:
          case WB_NODE_INERTIAL_UNIT:
          case WB_NODE_LIGHT_SENSOR:
          case WB_NODE_LINEAR_MOTOR:
          case WB_NODE_POSITION_SENSOR:
          case WB_NODE_ROTATIONAL_MOTOR:
          case WB_NODE_TOUCH_SENSOR:
            ue_write_values(update_element);
            break;
          case WB_NODE_CAMERA:
            camera_update(tag);
            break;
          case WB_NODE_LIDAR:
            lidar_update(tag);
            break;
          case WB_NODE_RADAR:
            radar_update(tag);
            break;
          case WB_NODE_RANGE_FINDER:
            range_finder_update(tag);
            break;
          default:
            break;
        }
        buffer_append("}");
      }
    }
    buffer_append("}");
  }
  buffer_append("}");
  wb_robot_wwi_send_text(buffer);
  free_buffer();
}

void wbu_default_robot_window_set_images_max_size(int max_width, int max_height) {
  assert(max_width > 0);
  assert(max_height > 0);
  images_max_width = max_width;
  images_max_height = max_height;
}
