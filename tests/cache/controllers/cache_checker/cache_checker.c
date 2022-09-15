#include <stdio.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64
#define NB_TEXTURES 4
#define NB_MESHES 3

static WbDeviceTag cameras[NB_TEXTURES];
static WbDeviceTag distance_sensors[NB_MESHES];

void test_camera_color(int test_number, const int expected_color[3]) {
  int r, g, b;
  const int width = wb_camera_get_width(cameras[0]);

  for (int i = 0; i < NB_TEXTURES; ++i) {
    const unsigned char *image = wb_camera_get_image(cameras[i]);
    r = wb_camera_image_get_red(image, width, 32, 32);
    g = wb_camera_image_get_green(image, width, 32, 32);
    b = wb_camera_image_get_blue(image, width, 32, 32);
    ts_assert_color_in_delta(
      r, g, b, expected_color[0], expected_color[1], expected_color[2], 2,
      "In test %d: wrong texture color in object %d, received color = (%d, %d, %d) but expected color = (%d, %d, %d)",
      test_number, i, r, g, b, expected_color[0], expected_color[1], expected_color[2]);
  }
}

void test_distance(int test_number, const double expected_distance) {
  for (int i = 0; i < NB_MESHES; ++i) {
    const double distance = wb_distance_sensor_get_value(distance_sensors[i]);
    ts_assert_double_in_delta(distance, expected_distance, 1e-5,
                              "In test %d: wrong mesh %d is applied (distance is %f instead of %f)", test_number, i, distance,
                              expected_distance);
  }
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef node;
  WbFieldRef texture_fields[NB_TEXTURES];
  // get reference of texture in proto
  node = wb_supervisor_node_get_from_def("TEXTURE_IN_PROTO");
  texture_fields[0] = wb_supervisor_node_get_field(node, "textureUrl");
  // get reference of texture used in camera shape
  node = wb_supervisor_node_get_from_def("TEXTURE_IN_CAMERA.CAMERA.TEXTURED_BOX_SHAPE");
  texture_fields[1] = wb_supervisor_node_get_field(node, "textureUrl");
  // get reference of texture in nested proto
  node = wb_supervisor_node_get_from_def("TEXTURE_NESTED.NESTED_PROTO");
  texture_fields[2] = wb_supervisor_node_get_field(node, "url");
  // get reference of texture in base node
  node = wb_supervisor_node_get_from_def("TEXTURE_BASE_NODE.SHAPE.APPEARANCE.BASE_COLOR_MAP");
  texture_fields[3] = wb_supervisor_node_get_field(node, "url");

  WbFieldRef mesh_fields[NB_MESHES];
  // get reference of texture in base node
  node = wb_supervisor_node_get_from_def("MESH_BASE_NODE.SHAPE.MESH");
  mesh_fields[0] = wb_supervisor_node_get_field(node, "url");
  // get reference of texture in proto
  node = wb_supervisor_node_get_from_def("MESH_IN_PROTO.PROTO_SHAPE");
  mesh_fields[1] = wb_supervisor_node_get_field(node, "meshUrl");
  // get reference of texture in nested proto
  node = wb_supervisor_node_get_from_def("MESH_NESTED.NESTED_PROTO");
  mesh_fields[2] = wb_supervisor_node_get_field(node, "meshUrl");

  // initialize devices
  char device_name[20];

  for (int i = 0; i < NB_TEXTURES; ++i) {
    sprintf(device_name, "camera%d", i);
    cameras[i] = wb_robot_get_device(device_name);
    wb_camera_enable(cameras[i], TIME_STEP);
  }

  for (int i = 0; i < NB_MESHES; ++i) {
    sprintf(device_name, "distance_sensor_%d", i);
    distance_sensors[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
  }

  wb_robot_step(TIME_STEP);

  // test textures
  const int expected_red_color[3] = {210, 32, 40};
  test_camera_color(0, expected_red_color);

  wb_robot_step(TIME_STEP);

  for (int i = 0; i < NB_TEXTURES; ++i)
    wb_supervisor_field_set_mf_string(texture_fields[i], 0, "https://cyberbotics.com/test_suite_assets/blue_texture.jpg");

  wb_robot_step(TIME_STEP);

  const int expected_blue_color[3] = {27, 48, 237};
  test_camera_color(1, expected_blue_color);

  // test meshes
  test_distance(2, 125.0);

  wb_robot_step(TIME_STEP);

  for (int i = 0; i < NB_MESHES; ++i)
    wb_supervisor_field_set_mf_string(mesh_fields[i], 0, "https://cyberbotics.com/test_suite_assets/cube_0.2m.obj");

  wb_robot_step(TIME_STEP);

  test_distance(3, 50.0);

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
