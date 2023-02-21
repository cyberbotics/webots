#include <webots/camera.h>
#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <math.h>
#include <stdio.h>
#include <unistd.h>

static bool file_exists(const char *filename) {
  FILE *file = fopen(filename, "r");
  if (file) {
    fclose(file);
    return true;
  }
  return false;
}

static void test_exported_image_correct(const char *color_def, const unsigned char *expected_rgb, double angle_radians) {
  const int time_step = wb_robot_get_basic_time_step();

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 1);

  char file_name_png[BUFSIZ];
  char file_name_jpg[BUFSIZ];
  sprintf(file_name_png, "%s.png", color_def);
  sprintf(file_name_jpg, "%s.jpg", color_def);

  remove(file_name_png);
  remove(file_name_jpg);

  ts_assert_boolean_not_equal(file_exists(file_name_png), "Impossible to remove the PNG file");
  ts_assert_boolean_not_equal(file_exists(file_name_jpg), "Impossible to remove the JPG file");

  // Turn to face the requested direction
  WbFieldRef rotation_field = wb_supervisor_node_get_field(wb_supervisor_node_get_from_def("TEST"), "rotation");
  double rotation[4] = {0.0, 0.0, 1.0, angle_radians};
  wb_supervisor_field_set_sf_rotation(rotation_field, rotation);

  wb_robot_step(time_step);

  wb_supervisor_export_image(file_name_png, 100);
  wb_supervisor_export_image(file_name_jpg, 100);

  // Allow some time for the files to be written. 1 second should be plenty.
  sleep(1);

  ts_assert_boolean_equal(file_exists(file_name_png), "wb_supervisor_export_image() failed to create the %s file.",
                          file_name_png);

  ts_assert_boolean_equal(file_exists(file_name_jpg), "wb_supervisor_export_image() failed to create the %s file.",
                          file_name_jpg);

  // Remove the emissive color and set the texture to be the image we just exported
  WbNodeRef appearance = wb_supervisor_node_get_from_def(color_def);
  WbNodeRef material = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(appearance, "material"));
  WbFieldRef emissive_color = wb_supervisor_node_get_field(material, "emissiveColor");
  const double black[3] = {0.0, 0.0, 0.0};
  wb_supervisor_field_set_sf_color(emissive_color, black);
  WbNodeRef texture = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(appearance, "texture"));
  WbFieldRef texture_url = wb_supervisor_node_get_field(texture, "url");
  sprintf(file_name_png, "../controllers/supervisor_export_image/%s.png", color_def);
  wb_supervisor_field_insert_mf_string(texture_url, 0, file_name_png);

  wb_robot_step(time_step);

  // Use the camera to take a picture and confirm that the color is correct
  const unsigned char *image = wb_camera_get_image(camera);
  const int width = wb_camera_get_width(camera);
  const int height = wb_camera_get_width(camera);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      unsigned char rgb[3] = {0, 0, 0};
      rgb[0] = wb_camera_image_get_red(image, width, x, y);
      rgb[1] = wb_camera_image_get_green(image, width, x, y);
      rgb[2] = wb_camera_image_get_blue(image, width, x, y);

      ts_assert_color_in_delta(rgb[0], rgb[1], rgb[2], expected_rgb[0], expected_rgb[1], expected_rgb[2], 1,
                               "At position (%d, %d), color (%d, %d, %d) is not close enough to expected color (%d, %d, %d)", x,
                               y, rgb[0], rgb[1], rgb[2], expected_rgb[0], expected_rgb[1], expected_rgb[2]);
    }
  }
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  const unsigned char red[3] = {255, 0, 0};
  const unsigned char green[3] = {0, 255, 0};
  const unsigned char blue[3] = {0, 0, 255};
  test_exported_image_correct("RED", red, M_PI);
  test_exported_image_correct("GREEN", green, -M_PI / 2);
  test_exported_image_correct("BLUE", blue, 0.0);
  ts_send_success();
  return EXIT_SUCCESS;
}
