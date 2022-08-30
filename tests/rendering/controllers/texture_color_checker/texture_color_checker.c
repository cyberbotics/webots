#include <stdio.h>
#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64
#define NB_CAMERAS 11

static const int black[3] = {0, 0, 0};
static const int red[3] = {203, 0, 0};
static const int green[3] = {35, 203, 0};
static const int blue[3] = {0, 18, 203};
static const int yellow[3] = {203, 196, 0};
static const int cyan[3] = {6, 176, 203};
static const int orange[3] = {203, 85, 0};
static const int pink[3] = {203, 0, 175};

static WbDeviceTag cameras[NB_CAMERAS];

void test_camera_color(int i, const int expected_color[3]) {
  int r, g, b;
  const int width = wb_camera_get_width(cameras[i]);

  const unsigned char *image = wb_camera_get_image(cameras[i]);
  r = wb_camera_image_get_red(image, width, 0, 0);
  g = wb_camera_image_get_green(image, width, 0, 0);
  b = wb_camera_image_get_blue(image, width, 0, 0);
  ts_assert_color_in_delta(r, g, b, expected_color[0], expected_color[1], expected_color[2], 5,
                           "Camera %d received color = [%d, %d, %d] but expected color = [%d, %d, %d]", i, r, g, b,
                           expected_color[0], expected_color[1], expected_color[2]);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  ts_set_test_name(wb_robot_get_world_path());

  wb_robot_step(TIME_STEP);

  char device_name[10];
  for (int i = 0; i < NB_CAMERAS; ++i) {
    sprintf(device_name, "camera%d", i);
    cameras[i] = wb_robot_get_device(device_name);
    wb_camera_enable(cameras[i], TIME_STEP);
  }

  wb_robot_step(TIME_STEP);

  // ensure the texture map was loaded correctly
  const int *expected_color[NB_CAMERAS][3] = {{pink}, {red},    {green}, {blue},   {red}, {green},
                                              {blue}, {yellow}, {cyan},  {orange}, {pink}};
  for (int i = 0; i < NB_CAMERAS; ++i)
    test_camera_color(i, *expected_color[i]);

  // set the url of the visible blocks to block
  WbNodeRef simple_default = wb_supervisor_node_get_from_def("SIMPLE_DEFAULT");
  WbNodeRef simple_overwritten = wb_supervisor_node_get_from_def("SIMPLE_OVERWRITTEN");
  WbFieldRef field_simple_default = wb_supervisor_node_get_field(simple_default, "exposed_url");
  WbFieldRef field_simple_overwritten = wb_supervisor_node_get_field(simple_overwritten, "exposed_url");
  const char *simple_default_initial_value = wb_supervisor_field_get_mf_string(field_simple_default, 0);
  const char *simple_overwritten_initial_value = wb_supervisor_field_get_mf_string(field_simple_overwritten, 0);

  // set the url relative to the world since the field is visible (with random directory movements)
  wb_supervisor_field_set_mf_string(field_simple_default, 0, "../nonexistant_folder/../colors/black_texture.jpg");
  wb_supervisor_field_set_mf_string(field_simple_overwritten, 0, "../nonexistant_folder/../colors/black_texture.jpg");

  wb_robot_step(TIME_STEP);

  // ensure they turned black
  test_camera_color(2, black);
  test_camera_color(3, black);

  wb_robot_step(TIME_STEP);

  // restore initial value (ensure the correct scope is re-established)
  wb_supervisor_field_set_mf_string(field_simple_default, 0, simple_default_initial_value);
  wb_supervisor_field_set_mf_string(field_simple_overwritten, 0, simple_overwritten_initial_value);

  wb_robot_step(TIME_STEP);

  // ensure the color
  test_camera_color(2, green);
  test_camera_color(3, blue);

  // do the same thing for the longer chain
  WbNodeRef complex_default = wb_supervisor_node_get_from_def("COMPLEX_DEFAULT");
  WbNodeRef complex_overwritten = wb_supervisor_node_get_from_def("COMPLEX_OVERWRITTEN");
  WbFieldRef field_complex_default = wb_supervisor_node_get_field(complex_default, "highly_nested_url");
  WbFieldRef field_complex_overwritten = wb_supervisor_node_get_field(complex_overwritten, "highly_nested_url");
  const char *complex_default_initial_value = wb_supervisor_field_get_mf_string(field_complex_default, 0);
  const char *complex_overwritten_initial_value = wb_supervisor_field_get_mf_string(field_complex_overwritten, 0);

  // set the url relative to the world since the field is visible (with random directory movements)
  wb_supervisor_field_set_mf_string(field_complex_default, 0, "../nonexistant_folder/../colors/black_texture.jpg");
  wb_supervisor_field_set_mf_string(field_complex_overwritten, 0, "../nonexistant_folder/../colors/black_texture.jpg");

  wb_robot_step(TIME_STEP);

  // ensure they turned black

  test_camera_color(9, black);
  test_camera_color(10, black);

  wb_robot_step(TIME_STEP);

  // restore initial value (ensure the correct scope is re-established)
  wb_supervisor_field_set_mf_string(field_complex_default, 0, complex_default_initial_value);
  wb_supervisor_field_set_mf_string(field_complex_overwritten, 0, complex_overwritten_initial_value);

  wb_robot_step(TIME_STEP);

  // ensure they turned black
  test_camera_color(9, orange);
  test_camera_color(10, pink);

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
