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
 * Description: This controller should be used with a supervisor who owns a radar device,
 *              it will then display the targets visible by this radar.
 */

#include <webots/radar.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <math.h>
#include <stdio.h>

#define TIME_STEP 10
#define MAX_TARGETS 25
#define RENDERING_DEVICE_NUMBER 5

const char *rendering_devices_name[RENDERING_DEVICE_NUMBER] = {"FRONT_RIGHT_LIDAR", "FRONT_LEFT_LIDAR", "REAR_RIGHT_LIDAR",
                                                               "REAR_LEFT_LIDAR", "TOP_CAMERA"};

int main(int argc, char **argv) {
  wb_robot_init();

  // get and enable the radar
  WbDeviceTag radar = wb_robot_get_device("radar");
  wb_radar_enable(radar, TIME_STEP);

  // import radar targets
  WbFieldRef translation_fields[MAX_TARGETS];
  WbFieldRef color_fields[MAX_TARGETS];
  WbFieldRef size_fields[MAX_TARGETS];
  WbFieldRef children_field = wb_supervisor_node_get_field(wb_supervisor_node_get_self(), "children");

  // target string defines the target node
  const char *target_string = "Pose {\n"
                              "  translation 0 0 -10.0\n"
                              "  children [\n"
                              "    Shape {\n"
                              "      appearance Appearance {\n"
                              "        material Material {\n"
                              "          diffuseColor 0.8 0 0.0117647\n"
                              "          transparency 0.7\n"
                              "        }\n"
                              "      }\n"
                              "      geometry Box {\n"
                              "        size 5.0 2.5 2.0\n"
                              "      }\n"
                              "    }\n"
                              "  ]\n"
                              "}\n";

  // import the targets and make them invisible by the radars and cameras
  int i;
  for (i = 0; i < MAX_TARGETS; ++i) {
    wb_supervisor_field_import_mf_node_from_string(children_field, -1, target_string);
    WbNodeRef node = wb_supervisor_field_get_mf_node(children_field, -1);
    translation_fields[i] = wb_supervisor_node_get_field(node, "translation");
    WbNodeRef shape_node = wb_supervisor_field_get_mf_node(wb_supervisor_node_get_field(node, "children"), 0);
    WbNodeRef appearance_node = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(shape_node, "appearance"));
    WbNodeRef material_node = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(appearance_node, "material"));
    color_fields[i] = wb_supervisor_node_get_field(material_node, "diffuseColor");
    WbNodeRef box_node = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(shape_node, "geometry"));
    size_fields[i] = wb_supervisor_node_get_field(box_node, "size");

    int j;
    for (j = 0; j < RENDERING_DEVICE_NUMBER; ++j)
      wb_supervisor_node_set_visibility(node, wb_supervisor_node_get_from_def(rendering_devices_name[j]), false);
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    // update the position of every radar target
    const WbRadarTarget *targets = wb_radar_get_targets(radar);
    for (i = 0; i < MAX_TARGETS; ++i) {
      if (i > 0 && i < wb_radar_get_number_of_targets(radar) && targets[i].distance > 2.0) {
        double x = targets[i].distance;
        double y = -targets[i].distance * sin(targets[i].azimuth);
        double translation[3] = {x, y, -0.5};
        wb_supervisor_field_set_sf_vec3f(translation_fields[i], translation);
        double colorFactor = (targets[i].received_power + 60.0) / 50.0;
        colorFactor = fmax(fmin(colorFactor, 1), 0);
        colorFactor = pow(colorFactor, 0.33);  // linearize the value
        double color[3] = {colorFactor, 0, 1 - colorFactor};
        wb_supervisor_field_set_sf_color(color_fields[i], color);
        // get original received power in order to guess which type of object it is
        double power = pow(10, targets[i].received_power / 10.0) * 0.001 * pow(targets[i].distance, 4.0);
        double size[3] = {5, 2.5, 2};  // default car case
        if (power < 0.005) {           // motorcycle case
          size[0] = 3.0;
          size[1] = 1.0;
          size[2] = 2.0;
        } else if (power > 0.015) {  // bus case
          size[0] = 10.0;
          size[1] = 3.5;
          size[2] = 6.0;
        }
        wb_supervisor_field_set_sf_vec3f(size_fields[i], size);
      } else {
        double translation[3] = {0, 0, -10.0};
        wb_supervisor_field_set_sf_vec3f(translation_fields[i], translation);
      }
    }
  };

  wb_robot_cleanup();

  return 0;
}
