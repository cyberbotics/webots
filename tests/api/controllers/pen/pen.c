#include <webots/camera.h>
#include <webots/pen.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

WbDeviceTag *cam;
const int width = 64;
const int height = 64;
const int size = 4096;

typedef struct {
  int r;
  int g;
  int b;
} Color;

Color computeImageMeanColor(int index) {
  int x, y;
  Color color = {0, 0, 0};

  const unsigned char *image = wb_camera_get_image(cam[index]);
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      color.r += wb_camera_image_get_red(image, width, x, y);
      color.g += wb_camera_image_get_green(image, width, x, y);
      color.b += wb_camera_image_get_blue(image, width, x, y);
    }
  }
  color.r /= size;
  color.g /= size;
  color.b /= size;

  return color;
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag pen0, pen1;
  Color color;
  int i;

  pen0 = wb_robot_get_device("pen0");
  pen1 = wb_robot_get_device("pen1");
  wb_pen_write(pen0, false);
  wb_pen_write(pen1, false);

  cam = malloc(2 * sizeof(WbDeviceTag));
  cam[0] = wb_robot_get_device("camera0");
  cam[1] = wb_robot_get_device("camera1");
  wb_camera_enable(cam[0], TIME_STEP);
  wb_camera_enable(cam[1], TIME_STEP);

  // FIRST STEP - set ink color
  wb_robot_step(TIME_STEP);

  wb_pen_set_ink_color(pen0, 0xFF0000, 0.1);
  wb_pen_set_ink_color(pen1, 0x00FF00, 1.0);

  // check camera 0
  color = computeImageMeanColor(0);
  ts_assert_color_in_delta(color.r, color.g, color.b, 207, 207, 207, 2,
                           "The value measured by the camera \"cam0\" should be (r=207,g=207,b=207) and not (r=%d,g=%d,b=%d) "
                           "before the pen \"pen0\" prints",
                           color.r, color.g, color.b);
  // check camera 1
  color = computeImageMeanColor(1);
  ts_assert_color_in_delta(color.r, color.g, color.b, 207, 207, 207, 2,
                           "The value measured by the camera \"cam1\" should be (r=207,g=207,b=207) and not (r=%d,g=%d,b=%d) "
                           "before the pen \"pen1\" prints",
                           color.r, color.g, color.b);

  wb_pen_write(pen0, true);

  // SECOND STEP - write with pen 0
  wb_robot_step(TIME_STEP);

  // check camera 0
  color = computeImageMeanColor(0);
  ts_assert_color_in_delta(color.r, color.g, color.b, 207, 201, 201, 2,
                           "Pen \"pen0\" should print with color (r=207,g=201,b=201) and not (r=%d,g=%d,b=%d)", color.r,
                           color.g, color.b);
  // check camera 1
  color = computeImageMeanColor(1);
  ts_assert_color_in_delta(color.r, color.g, color.b, 207, 207, 207, 2,
                           "The value measured by the camera \"cam1\" should be (r=207,g=207,b=207) and not (r=%d,g=%d,b=%d) "
                           "before the pen \"pen1\" prints",
                           color.r, color.g, color.b);

  wb_pen_write(pen1, true);

  // THIRD STEP - write with both pens
  wb_robot_step(TIME_STEP);

  // check camera 0
  color = computeImageMeanColor(0);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 207, 195, 195, 2,
    "Pen \"pen0\" should print on the existing color obtaing color (r=207,g=195,b=195) and not (r=%d,g=%d,b=%d)d", color.r,
    color.g, color.b);

  // check camera 1
  color = computeImageMeanColor(1);
  ts_assert_color_in_delta(color.r, color.g, color.b, 0, 207, 0, 2,
                           "Pen \"pen1\" should print with color (r=0,g=207,b=0) and not (r=%d,g=%d,b=%d)", color.r, color.g,
                           color.b);

  wb_pen_set_ink_color(pen1, 0xCC33FF, 1.0);

  wb_pen_write(pen1, false);

  // FORTH STEP - change ink color of pen 1 but not write
  wb_robot_step(TIME_STEP);

  // check camera 0
  color = computeImageMeanColor(0);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 207, 189, 189, 2,
    "Pen \"pen0\" should print on the existing color obtaing color (r=207,g=189,b=189) and not (r=%d,g=%d,b=%d)", color.r,
    color.g, color.b);
  // check camera 1
  color = computeImageMeanColor(1);
  ts_assert_color_in_delta(color.r, color.g, color.b, 0, 207, 0, 2,
                           "Pen \"pen1\" should not overwrite color (r=0,g=207,b=0) with (r=%d,g=%d,b=%d) if WRITE=FALSE",
                           color.r, color.g, color.b);

  // wb_pen_write(pen0, true);
  wb_pen_write(pen1, true);

  // FIVE STEP - write with pen 1
  wb_robot_step(TIME_STEP);

  // check camera 0
  color = computeImageMeanColor(0);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 207, 178, 178, 2,
    "Pen \"pen0\" should print on the existing color obtaing color (r=207,g=178,b=178) and not (r=%d,g=%d,b=%d)", color.r,
    color.g, color.b);
  // check camera 1
  color = computeImageMeanColor(1);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 154, 178, 167, 2,
    "Pen \"pen1\" should overwrite previous color with color (r=154,g=178,b=167) and not (r=%d,g=%d,b=%d) if WRITE=FALSE",
    color.r, color.g, color.b);

  // FIFTH STEP - check density
  for (i = 1; i <= 33; i++) {
    wb_robot_step(TIME_STEP);
  }

  // check camera 0
  color = computeImageMeanColor(0);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 207, 0, 0, 5,
    "Pen \"pen0\" should print on the existing color obtaining color (r=207,g=0,b=0) and not (r=%d,g=%d,b=%d)", color.r,
    color.g, color.b);

  ts_send_success();
  return EXIT_SUCCESS;
}
