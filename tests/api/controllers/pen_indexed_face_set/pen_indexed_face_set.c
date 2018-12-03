/*
 * Description:  Test painting with Pen on a rotated indexed face set.
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
        // printf("Pixel x %d y %d: r=%d, g=%d, b=%d\n", x, y, wb_camera_image_get_red(image, width, x, y),
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
  double pos;
  int oldValue, numColorPixels = -1;

  pen = wb_robot_get_device("pen");
  wb_pen_set_ink_color(pen, 0xFA8A0A, 1.0);
  wb_pen_write(pen, true);

  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  motor = wb_robot_get_device("linear motor");

  position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  wb_robot_step(TIME_STEP);

  wb_pen_write(pen, false);

  // 1) Test initial painted area location
  // ts_send_error_and_exit
  numColorPixels = computeNumColorPixels();
  ts_assert_boolean_equal(numColorPixels <= 50,
                          "The number of pixels painted after the first step should be lower than 50 not %d", numColorPixels);

  color = getPixelColor(10, 48);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 206, 171, 58, 15,
    "Pixel (10, 48) should be painted with color [r=%d, g=%d, b=%d] not [r=%d, g=%d, b=%d] after first paint call", 206, 171,
    58, color.r, color.g, color.b);

  wb_robot_step(TIME_STEP);

  // move pen
  pos = wb_position_sensor_get_value(position_sensor);
  pos = pos + 0.06;
  wb_motor_set_position(motor, pos);
  while (pos != wb_position_sensor_get_value(position_sensor)) {
    wb_robot_step(TIME_STEP);
  }

  wb_robot_step(TIME_STEP);
  // print
  wb_pen_write(pen, true);

  wb_robot_step(TIME_STEP);
  wb_pen_write(pen, false);

  // 2) Test second painted area location
  oldValue = numColorPixels;
  numColorPixels = computeNumColorPixels();

  ts_assert_boolean_equal(
    oldValue < numColorPixels && numColorPixels <= 150,
    "The number of pixels painted after the second step should be greater than before (%d) and lower than 150 not %d", oldValue,
    numColorPixels);

  color = getPixelColor(22, 30);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 206, 171, 58, 15,
    "Pixel (22, 30) should be painted with color [r=%d, g=%d, b=%d] not [r=%d, g=%d, b=%d] after second paint call", 206, 171,
    58, color.r, color.g, color.b);

  wb_robot_step(TIME_STEP);

  // move pen
  pos = wb_position_sensor_get_value(position_sensor);
  pos = pos + 0.07;
  wb_motor_set_position(motor, pos);
  while (pos != wb_position_sensor_get_value(position_sensor)) {
    wb_robot_step(TIME_STEP);
  }

  // print
  wb_pen_write(pen, true);

  wb_robot_step(TIME_STEP);
  wb_pen_write(pen, false);

  // 3) Test third painted area location
  oldValue = numColorPixels;
  numColorPixels = computeNumColorPixels();

  ts_assert_boolean_equal(
    oldValue < numColorPixels && numColorPixels <= 230,
    "The number of pixels painted after the third step should be greater than before (%d) and lower than 230 not %d", oldValue,
    numColorPixels);

  color = getPixelColor(33, 12);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 206, 171, 58, 15,
    "Pixel (33, 12) should be painted with color [r=%d, g=%d, b=%d] not [r=%d, g=%d, b=%d] after third paint call", 206, 171,
    58, color.r, color.g, color.b);

  ts_send_success();
  return EXIT_SUCCESS;
}
