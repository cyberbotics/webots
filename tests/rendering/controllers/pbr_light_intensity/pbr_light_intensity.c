#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

static void read_color(WbDeviceTag camera, int color[3]) {
  // Wait for field changes to reach the renderer and the camera image to be refreshed.
  wb_robot_step(2 * TIME_STEP);
  const unsigned char *image = wb_camera_get_image(camera);
  ts_assert_pointer_not_null(image, "Camera image is missing.");
  const int width = wb_camera_get_width(camera);
  const int x = width / 2;
  const int y = wb_camera_get_height(camera) / 2;
  color[0] = wb_camera_image_get_red(image, width, x, y);
  color[1] = wb_camera_image_get_green(image, width, x, y);
  color[2] = wb_camera_image_get_blue(image, width, x, y);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  const WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  const WbFieldRef metalness = wb_supervisor_node_get_field(wb_supervisor_node_get_from_def("MATERIAL"), "metalness");
  const char *light_names[] = {"DIRECTIONAL", "POINT", "SPOT"};
  const double intensities[] = {0.25, 0.5, 2.0};

  for (int shadows = 0; shadows < 2; ++shadows) {
    for (int light = 0; light < 3; ++light) {
      const WbNodeRef node = wb_supervisor_node_get_from_def(light_names[light]);
      const WbFieldRef on = wb_supervisor_node_get_field(node, "on");
      const WbFieldRef intensity = wb_supervisor_node_get_field(node, "intensity");
      wb_supervisor_field_set_sf_bool(on, true);
      wb_supervisor_field_set_sf_bool(wb_supervisor_node_get_field(node, "castShadows"), shadows);

      for (int metallic = 0; metallic < 2; ++metallic) {
        wb_supervisor_field_set_sf_float(metalness, metallic);
        wb_supervisor_field_set_sf_float(intensity, 1.0);
        wb_camera_set_exposure(camera, 1.0);
        int reference[3];
        read_color(camera, reference);
        for (int channel = 0; channel < 3; ++channel)
          ts_assert_boolean_equal(reference[channel] > 10 && reference[channel] < 240,
                                  "%s: reference channel %d is black or saturated (%d).", light_names[light], channel,
                                  reference[channel]);

        for (int i = 0; i < 3; ++i) {
          wb_supervisor_field_set_sf_float(intensity, intensities[i]);
          // Linear direct lighting gives the same image when exposure compensates for light intensity.
          wb_camera_set_exposure(camera, 1.0 / intensities[i]);
          int color[3];
          read_color(camera, color);
          ts_assert_color_in_delta(color[0], color[1], color[2], reference[0], reference[1], reference[2], 2,
                                   "%s, shadows=%d, metalness=%d, intensity=%g: got [%d, %d, %d], expected [%d, %d, %d].",
                                   light_names[light], shadows, metallic, intensities[i], color[0], color[1], color[2],
                                   reference[0], reference[1], reference[2]);
        }

        wb_supervisor_field_set_sf_float(intensity, 0.0);
        wb_camera_set_exposure(camera, 1.0);
        int dark[3];
        read_color(camera, dark);
        ts_assert_color_in_delta(dark[0], dark[1], dark[2], 0, 0, 0, 1, "%s: zero intensity should give a black image.",
                                 light_names[light]);
      }
      wb_supervisor_field_set_sf_bool(on, false);
    }
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
