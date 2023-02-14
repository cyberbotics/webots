#include <math.h>
#include <unistd.h>

#include <webots/camera.h>
#include <webots/supervisor.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define FILE_NAME_PNG "image.png"
#define FILE_NAME_JPG "image.jpg"

bool file_exists(const char *filename) {
  FILE *file = fopen(filename, "r");
  if (file) {
    fclose(file);
    return true;
  }
  return false;
}

void test_exported_image_correct(char *colorDef, unsigned char *expectedRgb, double angleRadians) {
  int time_step = wb_robot_get_basic_time_step();

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 1);

  char fileNamePng[BUFSIZ];
  char fileNameJpg[BUFSIZ];
  sprintf(fileNamePng, "%s.png", colorDef);
  sprintf(fileNameJpg, "%s.jpg", colorDef);

  remove(fileNamePng);
  remove(fileNameJpg);

  ts_assert_boolean_not_equal(file_exists(fileNamePng), "Impossible to remove the PNG file");
  ts_assert_boolean_not_equal(file_exists(fileNameJpg), "Impossible to remove the JPG file");

  // Turn to face the requested direction
  WbFieldRef rotationField = wb_supervisor_node_get_field(wb_supervisor_node_get_from_def("Test"), "rotation");
  double rotation[4] = {0.0, 0.0, 1.0, 0.0};
  rotation[3] = angleRadians;
  wb_supervisor_field_set_sf_rotation(rotationField, rotation);

  wb_robot_step(time_step);

  wb_supervisor_export_image(fileNamePng, 100);
  wb_supervisor_export_image(fileNameJpg, 100);

  // Allow some time for the files to be written. 1 second should be plenty.
  // Why isn't this all that is needed?
  sleep(1);

  // Why do we need to wait a steps for this to work in real-time or fast mode but not when stepping?
  wb_robot_step(time_step);

  ts_assert_boolean_equal(file_exists(fileNamePng),
                          "wb_supervisor_export_image() failed to create the %s file.", fileNamePng);

  ts_assert_boolean_equal(file_exists(fileNameJpg),
                          "wb_supervisor_export_image() failed to create the %s file.", fileNameJpg);

  // Remove the emissive color and set the texture to be the image we just exported
  WbNodeRef appearance = wb_supervisor_node_get_from_def(colorDef);
  WbNodeRef material = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(appearance, "material"));
  WbFieldRef emissiveColor = wb_supervisor_node_get_field(material, "emissiveColor");
  double black[3] = {0.0, 0.0, 0.0};
  wb_supervisor_field_set_sf_color(emissiveColor, black);
  WbNodeRef texture = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(appearance, "texture"));
  WbFieldRef textureUrl = wb_supervisor_node_get_field(texture, "url");
  sprintf(fileNamePng, "../controllers/supervisor_export_image/%s.png", colorDef);
  wb_supervisor_field_insert_mf_string(textureUrl, 0, fileNamePng);

  wb_robot_step(time_step);

  // Use the camera to take a picture and confirm that the color is correct
  const unsigned char *image = wb_camera_get_image(camera);
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_width(camera);
  unsigned char rgb[3] = {0, 0, 0};
  rgb[0] = wb_camera_image_get_red(image, width, width / 2, height / 2);
  rgb[1] = wb_camera_image_get_green(image, width, width / 2, height / 2);
  rgb[2] = wb_camera_image_get_blue(image, width, width / 2, height / 2);

  ts_assert_color_in_delta(rgb[0], rgb[1], rgb[2], expectedRgb[0], expectedRgb[1], expectedRgb[2], 1,
                           "Color (%d, %d, %d) is not close enough to expected color (%d, %d, %d)", rgb[0], rgb[1], rgb[2],
                           expectedRgb[0], expectedRgb[1], expectedRgb[2]);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  unsigned char red[3] = {255, 0, 0};
  unsigned char green[3] = {0, 255, 0};
  unsigned char blue[3] = {0, 0, 255};
  test_exported_image_correct("RED", red, M_PI);
  test_exported_image_correct("GREEN", green, -M_PI / 2);
  test_exported_image_correct("BLUE", blue, 0.0);
  ts_send_success();
  return EXIT_SUCCESS;
}
