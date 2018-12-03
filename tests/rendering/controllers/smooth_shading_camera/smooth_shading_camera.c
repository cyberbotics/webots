#include <webots/camera.h>
#include <webots/robot.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const unsigned char *camera_image = wb_camera_get_image(camera);
  const int width = wb_camera_get_width(camera);

  int y = 100;
  int last_blue_value = 0;
  int x = 0;

  // width of cylinder on camera image (pixel 76 to 253)
  // The background has a blue component of 0 so we can test just blue pixels
  // in order to test for the ascending gradient pixel by pixel
  for (x = 76; x < 253; ++x) {
    int current_blue_value = (int)wb_camera_image_get_blue(camera_image, width, x, y);
    printf("current_blue_value: %d\n", current_blue_value);
    ts_assert_boolean_equal(wb_camera_image_get_blue(camera_image, width, x, y) >= last_blue_value,
                            "Each pixel should be brighter from right to left");
    last_blue_value = current_blue_value;
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
