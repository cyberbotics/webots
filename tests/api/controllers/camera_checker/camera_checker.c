#include <webots/camera.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <string.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_assert_boolean_equal(argc == 2, "camera_checker badly used. world name expected as argument.");

  ts_setup(argv[1]);

  WbDeviceTag camera = wb_robot_get_device("camera");
  ts_assert_boolean_equal(camera, "Camera device cannot be get.");

  WbDeviceTag camera2 = 0;
  if (strcmp(argv[1], "camera_color_compositor") == 0) {
    camera2 = wb_robot_get_device("camera2");
    ts_assert_boolean_equal(camera2, "Second camera device cannot be get.");
    wb_camera_enable(camera2, TIME_STEP);
  }

  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);

  ts_assert_boolean_equal(width >= 1 && height >= 1, "Unexpected dimension: %dx%d", width, height);

  if (strcmp(argv[1], "camera_color") == 0) {
    double fov = wb_camera_get_fov(camera);
    ts_assert_double_equal(fov, 0.785398, "Unexpected original FOV: %f", fov);

    wb_camera_set_fov(camera, 2 * fov);
  }

  wb_camera_enable(camera, TIME_STEP);

  wb_robot_step(TIME_STEP);

  if (strcmp(argv[1], "camera_color") == 0) {
    ts_assert_double_equal(wb_camera_get_min_fov(camera), 1.56, "Unexpected minimum FOV value: %f",
                           wb_camera_get_min_fov(camera));
    ts_assert_double_equal(wb_camera_get_max_fov(camera), 1.58, "Unexpected maximum FOV value: %f",
                           wb_camera_get_max_fov(camera));
    double fov = wb_camera_get_fov(camera);
    ts_assert_double_equal(fov, 1.570796, "Unexpected modified FOV value: %f", fov);
  } else {
    ts_assert_double_equal(wb_camera_get_min_fov(camera), wb_camera_get_fov(camera), "Unexpected minimum FOV value: %f",
                           wb_camera_get_min_fov(camera));
    ts_assert_double_equal(wb_camera_get_max_fov(camera), wb_camera_get_fov(camera), "Unexpected maximum FOV value: %f",
                           wb_camera_get_max_fov(camera));
  }

  if (strcmp(argv[1], "camera_color") == 0 || strcmp(argv[1], "camera_color_spherical") == 0) {
    int red, green, blue;
    int x, y;
    const unsigned char *image = wb_camera_get_image(camera);

    int samples_number = 4;
    int samples_positions[6][2];
    int samples_expected_colors[6][3];

    // first line, first square
    samples_positions[0][0] = width / 6 - height / 12;
    samples_positions[0][1] = height / 4 - height / 8;
    // first line, last square
    samples_positions[1][0] = width - height / 12;
    samples_positions[1][1] = height / 4 - height / 8;
    // last line, first square
    samples_positions[2][0] = width / 6 - height / 12;
    samples_positions[2][1] = height - height / 8;
    // last line, last square
    samples_positions[3][0] = width - height / 12;
    samples_positions[3][1] = height - height / 8;

    // brown
    samples_expected_colors[0][0] = 122;
    samples_expected_colors[0][1] = 75;
    samples_expected_colors[0][2] = 65;
    // cyan
    samples_expected_colors[1][0] = 153;
    samples_expected_colors[1][1] = 200;
    samples_expected_colors[1][2] = 191;
    // white
    samples_expected_colors[2][0] = 202;
    samples_expected_colors[2][1] = 202;
    samples_expected_colors[2][2] = 202;
    // dark gray
    samples_expected_colors[3][0] = 54;
    samples_expected_colors[3][1] = 54;
    samples_expected_colors[3][2] = 54;

    if (strcmp(argv[1], "camera_color_spherical") == 0) {
      samples_number = 6;

      samples_positions[0][0] = 22;
      samples_positions[0][1] = 12;
      samples_positions[1][0] = 230;
      samples_positions[1][1] = 9;
      samples_positions[2][0] = 19;
      samples_positions[2][1] = 115;
      samples_positions[3][0] = 234;
      samples_positions[3][1] = 115;
      samples_positions[4][0] = 146;
      samples_positions[4][1] = 11;
      samples_positions[5][0] = 81;
      samples_positions[5][1] = 104;

      // yellow
      samples_expected_colors[0][0] = 207;
      samples_expected_colors[0][1] = 207;
      samples_expected_colors[0][2] = 0;
      // red
      samples_expected_colors[1][0] = 207;
      samples_expected_colors[1][1] = 0;
      samples_expected_colors[1][2] = 0;
      // yellow
      samples_expected_colors[2][0] = 207;
      samples_expected_colors[2][1] = 207;
      samples_expected_colors[2][2] = 0;
      // red
      samples_expected_colors[3][0] = 207;
      samples_expected_colors[3][1] = 0;
      samples_expected_colors[3][2] = 0;
      // pink
      samples_expected_colors[4][0] = 203;
      samples_expected_colors[4][1] = 83;
      samples_expected_colors[4][2] = 124;
      // green-ish
      samples_expected_colors[5][0] = 173;
      samples_expected_colors[5][1] = 198;
      samples_expected_colors[5][2] = 63;
    }

    int i;
    for (i = 0; i < samples_number; i++) {
      x = samples_positions[i][0];
      y = samples_positions[i][1];
      red = wb_camera_image_get_red(image, width, x, y);
      green = wb_camera_image_get_green(image, width, x, y);
      blue = wb_camera_image_get_blue(image, width, x, y);
      ts_assert_color_in_delta(
        red, green, blue, samples_expected_colors[i][0], samples_expected_colors[i][1], samples_expected_colors[i][2], 2,
        "Wrong color (#%d) at (%d, %d), Received color = (%d, %d, %d), Expected color = (%d, %d, %d)", i, x, y, red, green,
        blue, samples_expected_colors[i][0], samples_expected_colors[i][1], samples_expected_colors[i][2]);
    }
  } else if (strcmp(argv[1], "camera_color_compositor") == 0) {
    // make sure compositor is applied
    const unsigned char *image = wb_camera_get_image(camera);
    int red = wb_camera_image_get_red(image, width, 31, 31);
    int blue = wb_camera_image_get_blue(image, width, 31, 31);
    ts_assert_int_equal(
      red, 255, "Compositor not working, the image should be fully red, received %d instead of 255 for the red component", red);
    ts_assert_int_equal(
      blue, 0, "Compositor not working, the image should be fully red, received %d instead of 0 for the blue component", blue);
    // make sure we have same result for a compositor defined relatively to the PROTO
    const unsigned char *image2 = wb_camera_get_image(camera2);
    int red2 = wb_camera_image_get_red(image2, wb_camera_get_width(camera2), 31, 31);
    int blue2 = wb_camera_image_get_blue(image2, wb_camera_get_width(camera2), 31, 31);
    ts_assert_int_equal(
      red2, red,
      "Compositor from PROTO camera not working, both images should be the same, red component are not equal (%d != %d)", red2,
      red);
    ts_assert_int_equal(
      blue2, blue,
      "Compositor from PROTO camera not working, both images should be the same, blue component are not equal (%d != %d)",
      blue2, blue);
  } else if (strcmp(argv[1], "camera_color_motion_blur") == 0) {
    const unsigned char *image = wb_camera_get_image(camera);
    int red = wb_camera_image_get_red(image, width, 31, 31);
    ts_assert_int_equal(red, 224, "Image should be red, received %d instead of 224 for the red component", red);
    int i;
    for (i = 0; i < 10; ++i)  // let the red box fall
      wb_robot_step(TIME_STEP);
    image = wb_camera_get_image(camera);
    unsigned char red_gradient[] = {87, 115, 150, 188, 224};
    for (i = 0; i < sizeof(red_gradient); ++i) {
      red = wb_camera_image_get_red(image, width, 31, 5 + i * 13);
      ts_assert_int_equal(red, red_gradient[i], "Pixel red level should be %d, but is %d", red_gradient[i], red);
    }
    ts_assert_int_not_equal(red, 0, "Image should still contain some red");
    for (i = 0; i < 10; ++i)  // let the red artefact disapear from image
      wb_robot_step(TIME_STEP);
    image = wb_camera_get_image(camera);
    red = wb_camera_image_get_red(image, width, 31, 31);
    ts_assert_boolean_equal(red >= 0 && red <= 8, "Image should not contain any red anymore, red level is %d", red);
  } else if (strcmp(argv[1], "camera_color_focus") == 0) {
    unsigned char *image = (unsigned char *)wb_camera_get_image(camera);
    // No blur due to focus => upper part should be fully blue and lower part fully red
    ts_assert_int_equal(wb_camera_image_get_red(image, width, 31, 30), 0,
                        "Upper part of the image should not contain any red without focus");
    ts_assert_int_equal(wb_camera_image_get_blue(image, width, 31, 30), 255,
                        "Upper part of the image should be entirely blue without focus");
    ts_assert_int_equal(wb_camera_image_get_red(image, width, 31, 32), 227,
                        "Lower part of the image should be entirely red without focus");
    ts_assert_int_equal(wb_camera_image_get_blue(image, width, 31, 32), 0,
                        "Lower part of the image should not contain any blue without focus");
    double focus = (wb_camera_get_max_focal_distance(camera) + wb_camera_get_min_focal_distance(camera)) / 2;
    wb_camera_set_focal_distance(camera, focus);
    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);
    wb_robot_step(TIME_STEP);
    image = (unsigned char *)wb_camera_get_image(camera);
    // Blur due to focus => close to the edge color should be mixed
    ts_assert_int_not_equal(wb_camera_image_get_red(image, width, 31, 30), 0,
                            "Upper part of the image should contain some red with focus");
    ts_assert_int_equal(wb_camera_image_get_blue(image, width, 31, 30), 255,
                        "Upper part of the image should be entirely blue with focus");
    ts_assert_int_not_equal(wb_camera_image_get_red(image, width, 31, 32), 227,
                            "Lower part of the image should not be entirely red with focus");
    ts_assert_int_not_equal(wb_camera_image_get_blue(image, width, 31, 32), 0,
                            "Lower part of the image should contain some blue with focus");
  } else if (strcmp(argv[1], "camera_color_exposure") == 0) {
    unsigned char *image = (unsigned char *)wb_camera_get_image(camera);
    ts_assert_double_equal(wb_camera_get_exposure(camera), 0.5, "Unexpected exposure value: %f",
                           wb_camera_get_exposure(camera));
    // half exposure => upper part should be entirely blue (background) and lower part half red
    ts_assert_int_equal(wb_camera_image_get_red(image, width, 31, 30), 0, "Upper part of the image should not contain any red");
    ts_assert_int_equal(wb_camera_image_get_blue(image, width, 31, 30), 255,
                        "Upper part of the image should be entirely blue without focus");
    ts_assert_int_equal(wb_camera_image_get_red(image, width, 31, 32), 190, "Lower part of the image should be dark red");
    ts_assert_int_equal(wb_camera_image_get_blue(image, width, 31, 32), 0,
                        "Lower part of the image should not contain any blue");

    wb_camera_set_exposure(camera, 5);
    wb_robot_step(TIME_STEP);
    ts_assert_double_equal(wb_camera_get_exposure(camera), 5, "Unexpected exposure value: %f", wb_camera_get_exposure(camera));

    image = (unsigned char *)wb_camera_get_image(camera);
    ts_assert_int_equal(wb_camera_image_get_red(image, width, 31, 30), 0, "Upper part of the image should not contain any red");
    ts_assert_int_equal(wb_camera_image_get_blue(image, width, 31, 30), 255, "Upper part of the image should be entirely blue");
    ts_assert_int_equal(wb_camera_image_get_red(image, width, 31, 32), 255, "Lower part of the image should be red saturated");
    ts_assert_int_equal(wb_camera_image_get_blue(image, width, 31, 32), 0,
                        "Lower part of the image should not contain any blue");
  } else
    ts_send_error_and_exit("camera_checker doesn't support this world: %s", argv[1]);

  ts_send_success();
  return EXIT_SUCCESS;
}
