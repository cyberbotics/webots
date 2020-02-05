#include <webots/camera.h>
#include <webots/display.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

#define BLACK 0x00000000
#define WHITE 0x00FFFFFF
#define RED 0x00FF0000
#define GREEN 0x0000CF06
#define BLUE 0x000000FF
#define MAGENTA 0x00FF00FF
#define GRAY 0x00808080

static void reset_display(WbDeviceTag d) {
  int w = wb_display_get_width(d);
  int h = wb_display_get_height(d);

  // set white background
  wb_display_set_color(d, WHITE);
  wb_display_set_opacity(d, 1.0);
  wb_display_set_alpha(d, 1.0);
  wb_display_fill_rectangle(d, 0, 0, w, h);
  // set color to black
  wb_display_set_color(d, BLACK);
}

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

  // check dimension getters
  int camera_width = wb_camera_get_width(camera);
  int camera_height = wb_camera_get_height(camera);
  int display_width = wb_display_get_width(display);
  int display_height = wb_display_get_height(display);
  ts_assert_boolean_equal(
    camera_width == display_width && camera_width == 80,
    "The display or the camera are not able to get their width. Expected: 80 Received: %d (display) and %d (camera)\n",
    display_width, camera_width);
  ts_assert_boolean_equal(
    camera_height == display_height && camera_height == 80,
    "The display or the camera are not able to get their height. Expected: 60 Received: %d (display) and %d (camera)\n",
    display_height, camera_height);

  wb_camera_enable(camera, TIME_STEP);

  wb_robot_step(TIME_STEP);

  // check image paste and draw with alpha on top of it
  WbImageRef image_ref = wb_display_image_load(display, "./image.png");
  wb_display_image_paste(display, image_ref, 0, 0, false);
  wb_robot_step(TIME_STEP);
  quick_assert_color(camera, 5, 5, GREEN, "image paste failed");
  wb_display_set_alpha(display, 0.0);
  wb_display_fill_rectangle(display, 0, 0, display_width, display_height);
  wb_robot_step(TIME_STEP);
  quick_assert_color(camera, 5, 5, MAGENTA, "display clear failed");

  // check color, alpha, and opacity
  reset_display(display);
  wb_display_set_color(display, RED);
  wb_display_draw_pixel(display, 0, 0);
  wb_display_set_color(display, GREEN);
  wb_display_draw_pixel(display, 1, 0);
  wb_display_set_color(display, BLUE);
  wb_display_draw_pixel(display, 2, 0);
  wb_display_set_color(display, BLACK);
  wb_display_set_opacity(display, 0.5);
  wb_display_draw_pixel(display, 3, 0);
  wb_display_set_font(display, "wrong font", 8, false);  // should display warning and keep using default font
  wb_display_draw_text(display, "Bàzi\nBga", 30, 30);
  wb_display_set_opacity(display, 1.0);
  wb_display_set_alpha(display, 0.0);
  wb_display_draw_pixel(display, 4, 0);
  wb_display_set_alpha(display, 1.0);
  wb_display_set_font(display, "Palatino Linotype", 32, false);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera, 0, 0, 0xCF0000, "set red color failed");
  quick_assert_color(camera, 1, 0, 0x00B40E, "set green color failed");
  quick_assert_color(camera, 2, 0, 0x0000cf, "set blue color failed");
  quick_assert_color(camera, 3, 0, 0x797979, "set opacity failed");
  quick_assert_color(camera, 4, 0, MAGENTA, "set alpha color failed");                // background color expected
  quick_assert_color(camera, 31, 33, 0x797979, "draw text failed (text not found)");  // center of the first character
  // middle of the hole in the first character
  quick_assert_color(camera, 32, 32, 0xCFCFCF, "draw text failed (text found where it should not)");

  // check primitives
  int px1[4] = {13, 15, 15, 13};
  int px2[4] = {17, 19, 19, 17};
  int py[4] = {1, 1, 3, 3};

  reset_display(display);
  wb_display_draw_line(display, 1, 1, 3, 3);
  wb_display_draw_rectangle(display, 5, 1, 3, 3);
  wb_display_fill_rectangle(display, 9, 1, 3, 3);
  wb_display_draw_polygon(display, px1, py, 4);
  wb_display_fill_polygon(display, px2, py, 4);
  wb_display_draw_oval(display, 3, 7, 2, 2);
  wb_display_fill_oval(display, 9, 7, 2, 2);
  // due to the size the text should exceed the display size but not cause any problem
  wb_display_draw_text(display, "Bàzinga", 1, 32);

  wb_robot_step(TIME_STEP);

  quick_assert_color(camera, 0, 0, 0XCFCFCF, "drawing primitives failed");
  quick_assert_color(camera, 1, 1, BLACK, "draw line failed");
  quick_assert_color(camera, 3, 3, BLACK, "draw line failed");
  quick_assert_color(camera, 5, 1, BLACK, "draw rectangle failed");
  quick_assert_color(camera, 7, 3, BLACK, "draw rectangle failed");
  quick_assert_color(camera, 6, 2, 0XCFCFCF, "draw rectangle failed");
  // center of the first character
  quick_assert_color(camera, 14, 48, BLACK, "draw text failed after changing the font (text not found)");
  // middle of the hole in the first character
  quick_assert_color(camera, 17, 41, 0XCFCFCF, "draw text failed after changing the font (text found where it should not)");
  // try to draw rectangles outside of the bounds of the display to
  // ensure webots doesn't crash because of a buffer overflow/underflow
  wb_display_fill_rectangle(display, -display_width, -display_height, display_width * 2, display_height * 2);
  wb_display_draw_rectangle(display, -display_width, -display_height, display_width * 2, display_height * 2);

  // check attach/detach camera
  // reset display to default value
  wb_display_set_color(display, BLACK);
  wb_display_set_opacity(display, 1.0);
  wb_display_set_alpha(display, 1.0);
  wb_display_fill_rectangle(display, 0, 0, display_width, display_height);
  WbDeviceTag mirror_camera = wb_robot_get_device("mirror camera");
  wb_camera_enable(mirror_camera, TIME_STEP);
  wb_robot_step(TIME_STEP);

  quick_assert_color(camera, 0, 0, BLACK, "reset failed");
  wb_display_attach_camera(display, camera);
  wb_robot_step(TIME_STEP);

  quick_assert_color(mirror_camera, 0, 0, MAGENTA, "attach camera failed");
  wb_display_detach_camera(display);
  wb_robot_step(TIME_STEP);

  // after detaching the camera, the display is transparent
  // and the camera records the green plane behind the display
  quick_assert_color(camera, camera_width * 0.5, camera_height * 0.5, 0x0000CF00, "detach camera failed");

  ts_send_success();
  return EXIT_SUCCESS;
}
