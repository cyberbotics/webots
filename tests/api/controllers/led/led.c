#include <webots/camera.h>
#include <webots/led.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag led_phong = wb_robot_get_device("phong");
  WbDeviceTag led_pbr = wb_robot_get_device("pbr");
  WbDeviceTag led_light = wb_robot_get_device("light");

  WbDeviceTag camera = wb_robot_get_device("camera");

  wb_led_set(led_phong, 0xff0000);
  wb_led_set(led_pbr, 0xff0000);
  wb_led_set(led_light, 0xff0000);

  wb_camera_enable(camera, TIME_STEP);

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  ts_assert_int_equal(wb_led_get(led_phong), 0xff0000, "The phong LED has the wrong stored color.");
  ts_assert_int_equal(wb_led_get(led_pbr), 0xff0000, "The pbr LED has the wrong stored color.");
  ts_assert_int_equal(wb_led_get(led_light), 0xff0000, "The light LED has the wrong stored color.");

  const unsigned char *camera_image = wb_camera_get_image(camera);

  unsigned char phong_red = wb_camera_image_get_red(camera_image, 128, 19, 64);
  unsigned char pbr_red = wb_camera_image_get_red(camera_image, 128, 64, 64);
  unsigned char light_red = wb_camera_image_get_red(camera_image, 128, 103, 61);

  unsigned char phong_green = wb_camera_image_get_green(camera_image, 128, 19, 64);
  unsigned char phong_blue = wb_camera_image_get_blue(camera_image, 128, 19, 64);

  unsigned char pbr_green = wb_camera_image_get_green(camera_image, 128, 64, 64);
  unsigned char pbr_blue = wb_camera_image_get_blue(camera_image, 128, 64, 64);

  // fuzzy color check for HDR
  ts_assert_int_is_bigger(phong_red, 0xaa, "The phong material should be bright red");
  ts_assert_int_is_bigger(pbr_red, 0xaa, "The pbr material should be bright red");
  ts_assert_int_is_bigger(light_red, 0x80, "The spotlight's illuminated circle should be bright red");

  // check the spheres are only red
  ts_assert_int_equal(phong_green, 0x00, "The phong material should have no green");
  ts_assert_int_equal(phong_blue, 0x00, "The phong material should have no blue");
  ts_assert_int_equal(pbr_green, 0x00, "The pbr material should have no green");
  ts_assert_int_equal(pbr_blue, 0x00, "The pbr material should have no blue");

  ts_send_success();
  return EXIT_SUCCESS;
}
