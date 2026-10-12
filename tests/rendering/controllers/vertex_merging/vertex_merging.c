/*
 * Description:  Test that the vertices WREN merges keep the hard edges and the texture seams of a mesh: vertices sharing
 *               a position but not a normal or a texture coordinate must stay apart.
 */

#include <webots/camera.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

static const unsigned char *image;
static int width;

// minimum and maximum of a channel (0: red, 1: green, 2: blue) over a rectangle of the image
static void channel_range(int channel, int x0, int y0, int x1, int y1, int *min, int *max) {
  *min = 255;
  *max = 0;
  int x, y;
  for (y = y0; y <= y1; ++y) {
    for (x = x0; x <= x1; ++x) {
      const int value = channel == 0 ? wb_camera_image_get_red(image, width, x, y) :
                        channel == 1 ? wb_camera_image_get_green(image, width, x, y) :
                                       wb_camera_image_get_blue(image, width, x, y);
      if (value < *min)
        *min = value;
      if (value > *max)
        *max = value;
    }
  }
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_robot_step(2 * TIME_STEP);
  image = wb_camera_get_image(camera);
  width = wb_camera_get_width(camera);

  // the cube with a crease angle of 0 has flat faces with different shades
  int left_min, left_max, front_min, front_max;
  channel_range(0, 55, 92, 68, 108, &left_min, &left_max);
  channel_range(0, 92, 88, 112, 120, &front_min, &front_max);
  ts_assert_int_in_delta(left_max, left_min, 1, "The left face of the hard edged cube is not flat (%d to %d).", left_min,
                         left_max);
  ts_assert_int_in_delta(front_max, front_min, 1, "The front face of the hard edged cube is not flat (%d to %d).", front_min,
                         front_max);
  ts_assert_boolean_equal(abs(left_min - front_min) > 40, "The faces of the hard edged cube have the same shade (%d and %d).",
                          left_min, front_min);

  // the cube with a crease angle of 1.6 is smoothly shaded: the test detects smoothed normals
  int smooth_min, smooth_max;
  channel_range(0, 185, 80, 235, 110, &smooth_min, &smooth_max);
  ts_assert_boolean_equal(smooth_max - smooth_min > 20, "The smooth cube is not smoothly shaded (%d to %d).", smooth_min,
                          smooth_max);

  // the two triangles of the quad share their positions and normals but map the red and the blue halves of the texture
  int red_min, red_max, blue_min, blue_max;
  channel_range(0, 122, 155, 136, 160, &red_min, &red_max);
  channel_range(2, 122, 155, 136, 160, &blue_min, &blue_max);
  ts_assert_boolean_equal(red_min > 150 && blue_max < 50, "The lower triangle is not red (red %d to %d, blue %d to %d).",
                          red_min, red_max, blue_min, blue_max);
  channel_range(0, 165, 137, 178, 142, &red_min, &red_max);
  channel_range(2, 165, 137, 178, 142, &blue_min, &blue_max);
  ts_assert_boolean_equal(blue_min > 150 && red_max < 50, "The upper triangle is not blue (red %d to %d, blue %d to %d).",
                          red_min, red_max, blue_min, blue_max);

  ts_send_success();
  return EXIT_SUCCESS;
}
