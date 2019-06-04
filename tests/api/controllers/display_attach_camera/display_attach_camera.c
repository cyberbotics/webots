#include <webots/camera.h>
#include <webots/display.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

#define WHITE 0x00FFFFFF
#define RED 0x00FF0000
#define GREEN 0x0000FF00
#define BLUE 0x000000B4

static void quick_assert_color(WbDeviceTag camera, int px, int py, int expected, const char *error_msg) {
  const unsigned char *image = wb_camera_get_image(camera);
  int w = wb_camera_get_width(camera);

  ts_assert_color_in_delta(wb_camera_image_get_red(image, w, px, py), wb_camera_image_get_green(image, w, px, py),
                           wb_camera_image_get_blue(image, w, px, py), (expected >> 16) & 0xFF, (expected >> 8) & 0xFF,
                           expected & 0xFF, 1, error_msg);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag display = wb_robot_get_device("display");
  WbDeviceTag camera = wb_robot_get_device("camera");
  WbDeviceTag attached_camera = wb_robot_get_device("attached camera");

  wb_camera_enable(camera, TIME_STEP);
  wb_camera_enable(attached_camera, TIME_STEP);
  wb_display_attach_camera(display, attached_camera);

  wb_robot_step(2 * TIME_STEP);

  quick_assert_color(camera, 40, 40, BLUE, "Wrong display attaching the camera (blue shape not detected)");

  wb_robot_step(TIME_STEP);

  wb_display_detach_camera(display);
  wb_display_set_color(display, WHITE);
  wb_display_fill_rectangle(display, 0, 0, 80, 80);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera, 40, 40, 0xCFCFCF, "Wrong initial display color.");

  wb_display_set_alpha(display, 0.0);
  wb_display_fill_rectangle(display, 0, 0, 80, 80);
  wb_display_set_alpha(display, 1.0);
  wb_display_set_color(display, GREEN);
  wb_display_fill_rectangle(display, 0, 0, 40, 80);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera, 20, 40, 0x00CF00, "Wrong display green color after 'wb_display_draw_rectangle'.");
  quick_assert_color(camera, 60, 40, RED, "Wrong display red color after 'wb_display_draw_rectangle'.");

  wb_display_attach_camera(display, attached_camera);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera, 20, 40, 0x00CF00, "Wrong display attaching the camera (green shape not detected)");
  quick_assert_color(camera, 60, 40, BLUE, "Wrong display attaching the camera (blue shape not detected)");

  wb_display_detach_camera(display);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera, 20, 40, 0x00CF00, "Wrong display green color after detatching the camera.");
  quick_assert_color(camera, 60, 40, RED, "Wrong display red color after detaching the camera.");

  wb_display_set_color(display, WHITE);
  wb_display_fill_rectangle(display, 0, 0, 80, 80);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera, 40, 40, 0xCFCFCF, "Wrong display color after 'wb_display_draw_rectangle'.");

  ts_send_success();
  return EXIT_SUCCESS;
}
