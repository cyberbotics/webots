/*
 * Description:  Test painting with Pen on a scaled Mesh.
 */

#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/pen.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

WbDeviceTag camera;
const int width = 64;
const int height = 64;
const int size = 4096;

typedef struct {
  int r;
  int g;
  int b;
} Color;

bool isGray(Color color) {
  return (color.r == color.g) && (color.g == color.b);
}

int computeNumColorPixels() {
  int x, y;
  int numColorPixels = 0;
  Color color = {0, 0, 0};

  const unsigned char *image = wb_camera_get_image(camera);
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      color.r = wb_camera_image_get_red(image, width, x, y);
      color.g = wb_camera_image_get_green(image, width, x, y);
      color.b = wb_camera_image_get_blue(image, width, x, y);
      if (!isGray(color)) {
        numColorPixels++;
        // printf("Pixel(%d, %d): r=%d, g=%d, b=%d\n", x, y, wb_camera_image_get_red(image, width, x, y),
        //        wb_camera_image_get_green(image, width, x, y), wb_camera_image_get_blue(image, width, x, y));
      }
    }
  }

  return numColorPixels;
}

Color getPixelColor(int x, int y) {
  Color color = {0, 0, 0};

  const unsigned char *image = wb_camera_get_image(camera);
  color.r = wb_camera_image_get_red(image, width, x, y);
  color.g = wb_camera_image_get_green(image, width, x, y);
  color.b = wb_camera_image_get_blue(image, width, x, y);
  return color;
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag pen, motor, position_sensor;
  Color color;
  int numColorPixels = -1;

  pen = wb_robot_get_device("pen");
  wb_pen_set_ink_color(pen, 0xFA8A0A, 1.0);
  wb_pen_write(pen, true);

  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  motor = wb_robot_get_device("linear motor");

  position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  wb_robot_step(TIME_STEP);

  // 1) Test initial painted area location
  numColorPixels = computeNumColorPixels();
  ts_assert_boolean_equal(numColorPixels <= 20,
                          "The number of pixels painted after the first step should be lower than 20 not %d", numColorPixels);

  color = getPixelColor(46, 26);
  ts_assert_boolean_equal(isGray(color), "The pixel (46,26) should be gray at first step");

  wb_robot_step(TIME_STEP);

  // move pen
  wb_motor_set_position(motor, wb_position_sensor_get_value(position_sensor) + 0.04);
  wb_robot_step(5 * TIME_STEP);

  // 2) Test third painted area location
  color = getPixelColor(46, 26);
  ts_assert_boolean_equal(!isGray(color), "The pixel (46,26) should not be gray at last step");

  ts_send_success();
  return EXIT_SUCCESS;
}
