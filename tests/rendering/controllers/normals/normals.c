#include <webots/camera.h>
#include <webots/robot.h>
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag cameras[7];
  char buffer[16];
  int i = 0, j = 0;

  for (i = 0; i < 7; ++i) {
    sprintf(buffer, "camera %d", i + 1);
    cameras[i] = wb_robot_get_device(buffer);
    wb_camera_enable(cameras[i], TIME_STEP);
  }

  wb_robot_step(TIME_STEP);

  // compare normal per vertex and creaseAngle
  const unsigned char *normal_per_vertex_image = wb_camera_get_image(cameras[0]);
  const unsigned char *crease_angle_image = wb_camera_get_image(cameras[1]);
  int diff = 0;
  for (i = 0; i < 256; ++i) {
    for (j = 0; j < 256; ++j) {
      diff += abs(wb_camera_image_get_red(normal_per_vertex_image, 256, i, j) -
                  wb_camera_image_get_red(crease_angle_image, 256, i, j));
      diff += abs(wb_camera_image_get_green(normal_per_vertex_image, 256, i, j) -
                  wb_camera_image_get_green(crease_angle_image, 256, i, j));
      diff += abs(wb_camera_image_get_blue(normal_per_vertex_image, 256, i, j) -
                  wb_camera_image_get_blue(crease_angle_image, 256, i, j));
    }
  }
  ts_assert_int_is_bigger(10000, diff, "Normal per vertex image does not match crease angle image.");

  // compare normal per vertex and normal per face for the flat case
  const unsigned char *normal_per_vertex_image_2 = wb_camera_get_image(cameras[2]);
  const unsigned char *normal_per_face_image = wb_camera_get_image(cameras[3]);
  diff = 0;
  for (i = 0; i < 256; ++i) {
    for (j = 0; j < 256; ++j) {
      diff += abs(wb_camera_image_get_red(normal_per_vertex_image_2, 256, i, j) -
                  wb_camera_image_get_red(normal_per_face_image, 256, i, j));
      diff += abs(wb_camera_image_get_green(normal_per_vertex_image_2, 256, i, j) -
                  wb_camera_image_get_green(normal_per_face_image, 256, i, j));
      diff += abs(wb_camera_image_get_blue(normal_per_vertex_image_2, 256, i, j) -
                  wb_camera_image_get_blue(normal_per_face_image, 256, i, j));
    }
  }
  ts_assert_int_is_bigger(10000, diff, "Normal per vertex image does not match normal per face image in flat case.");

  // compare normal per vertex and normal per face
  const unsigned char *normal_per_vertex_image_3 = wb_camera_get_image(cameras[4]);
  const unsigned char *normal_per_face_image_2 = wb_camera_get_image(cameras[5]);
  diff = 0;
  for (i = 0; i < 256; ++i) {
    for (j = 0; j < 256; ++j) {
      diff += abs(wb_camera_image_get_red(normal_per_vertex_image_3, 256, i, j) -
                  wb_camera_image_get_red(normal_per_face_image_2, 256, i, j));
      diff += abs(wb_camera_image_get_green(normal_per_vertex_image_3, 256, i, j) -
                  wb_camera_image_get_green(normal_per_face_image_2, 256, i, j));
      diff += abs(wb_camera_image_get_blue(normal_per_vertex_image_3, 256, i, j) -
                  wb_camera_image_get_blue(normal_per_face_image_2, 256, i, j));
    }
  }
  ts_assert_int_is_bigger(10000, diff, "Normal per vertex image does not match normal per face image.");

  // compare normal per vertex and no normal and no crease angle case
  const unsigned char *image = wb_camera_get_image(cameras[6]);
  diff = 0;
  for (i = 0; i < 256; ++i) {
    for (j = 0; j < 256; ++j) {
      diff += abs(wb_camera_image_get_red(normal_per_vertex_image, 256, i, j) - wb_camera_image_get_red(image, 256, i, j));
      diff += abs(wb_camera_image_get_green(normal_per_vertex_image, 256, i, j) - wb_camera_image_get_green(image, 256, i, j));
      diff += abs(wb_camera_image_get_blue(normal_per_vertex_image, 256, i, j) - wb_camera_image_get_blue(image, 256, i, j));
    }
  }
  ts_assert_int_is_bigger(diff, 10000, "Normal per vertex image should not match the no normal and no crease angle image.");

  ts_send_success();
  return EXIT_SUCCESS;
}
