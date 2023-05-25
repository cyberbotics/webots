#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <string.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define VISIBLE_SOLID_NUMBER 6

// This test is mainly testing the functionalities for a planar camera.
// Some basic tests for spherical and cylindrical cameras are also performed checking mainly
// the number of visible objects and the position on camera.

// objects visible in planar camera
static const char *visible_solid_models[VISIBLE_SOLID_NUMBER] = {
  "visible sphere", "visible box", "sub solid", "visible capsule", "composed solid", "visible sphere without BO"};

static const char *occcluded_solid_model = "occluded box";

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  int i, j;

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);

  WbDeviceTag camera_spherical = wb_robot_get_device("spherical");
  wb_camera_enable(camera_spherical, TIME_STEP);
  wb_camera_recognition_enable(camera_spherical, TIME_STEP);

  WbDeviceTag camera_cylindrical = wb_robot_get_device("cylindrical");
  wb_camera_enable(camera_cylindrical, TIME_STEP);
  wb_camera_recognition_enable(camera_cylindrical, TIME_STEP);

  // check number of object recognized
  wb_robot_step(TIME_STEP);
  int object_number = wb_camera_recognition_get_number_of_objects(camera);
  ts_assert_int_equal(object_number, VISIBLE_SOLID_NUMBER + 1,
                      "The planar camera should initially see %d objects and not %d (without occlusion).",
                      VISIBLE_SOLID_NUMBER + 1, object_number);

  object_number = wb_camera_recognition_get_number_of_objects(camera_spherical);
  ts_assert_int_equal(object_number, 10, "The spherical camera should initially see %d objects and not %d (with occlusion).", 9,
                      object_number);

  object_number = wb_camera_recognition_get_number_of_objects(camera_cylindrical);
  ts_assert_int_equal(object_number, 8, "The cylindrical camera should initially see %d objects and not %d (with occlusion).",
                      7, object_number);

  // enable occlusion
  WbNodeRef recognition_node = wb_supervisor_node_get_from_def("RECOGNITION");
  WbFieldRef occlusion_field = wb_supervisor_node_get_field(recognition_node, "occlusion");
  wb_supervisor_field_set_sf_int32(occlusion_field, 2);

  ts_assert_boolean_equal(wb_camera_recognition_has_segmentation(camera),
                          "The Recognition.segmentation field should be set to TRUE.");
  const unsigned char *image = wb_camera_recognition_get_segmentation_image(camera);
  ts_assert_boolean_equal(image == NULL, "No segmentation image should be returned if segmentaton is disabled.");

  wb_robot_step(TIME_STEP);

  // enable segmentation
  wb_camera_recognition_enable_segmentation(camera);

  wb_robot_step(TIME_STEP);
  // check segmentation image
  ts_assert_boolean_equal(wb_camera_recognition_is_segmentation_enabled(camera),
                          "The Recognition.segmentation functionality should be enabled.");
  image = wb_camera_recognition_get_segmentation_image(camera);
  ts_assert_boolean_not_equal(
    image == NULL, "The camera segmentation image should not be NULL after enabling the segmented image generation.");
  const int width = wb_camera_get_width(camera);
  int red, green, blue;
  red = wb_camera_image_get_red(image, width, 215, 124);
  ts_assert_int_in_delta(red, 193, 5, "Image should contain a red sphere at (215, 124): found red channel %d expected %d.", red,
                         193);
  red = wb_camera_image_get_red(image, width, 35, 71);
  ts_assert_int_in_delta(
    red, 255, 5, "Image should contain a red L shaped geometry at (35, 71): found red channel %d expected %d.", red, 255);
  red = wb_camera_image_get_red(image, width, 95, 77);
  green = wb_camera_image_get_red(image, width, 95, 77);
  blue = wb_camera_image_get_red(image, width, 95, 77);
  ts_assert_color_in_delta(red, green, blue, 0, 0, 0, 2,
                           "Image should not contain any object at (95, 77): found color (%d, %d, %d) expected (%d, %d, %d).",
                           red, green, blue, 0, 0, 0);
  red = wb_camera_image_get_red(image, width, 52, 124);
  green = wb_camera_image_get_green(image, width, 52, 124);
  blue = wb_camera_image_get_blue(image, width, 52, 124);
  ts_assert_color_in_delta(red, green, blue, 0, 0, 255, 5,
                           "Image should contain a blue sphere at (54, 124)): found color (%d, %d, %d) expected (%d, %d, %d).",
                           red, green, blue, 0, 0, 255);

  ts_assert_boolean_equal(
    wb_camera_recognition_save_segmentation_image(camera, "segmentation.jpg", 100) != -1,
    "Camera recognition segmentation image did not save correctly (wb_camera_recognition_save_segmentation_image returned -1)");

  wb_robot_step(TIME_STEP);
  // disable segmentation
  wb_camera_recognition_disable_segmentation(camera);
  image = wb_camera_recognition_get_segmentation_image(camera);
  ts_assert_boolean_equal(image == NULL, "No segmentation image should be returned if segmentation is disabled.");

  remove("segmentation.jpg");

  wb_robot_step(TIME_STEP);
  ts_assert_boolean_not_equal(wb_camera_recognition_is_segmentation_enabled(camera),
                              "The Recognition.segmentation functionality should be disabled.");
  WbFieldRef segmentation_field = wb_supervisor_node_get_field(recognition_node, "segmentation");
  wb_supervisor_field_set_sf_bool(segmentation_field, false);

  wb_robot_step(TIME_STEP);

  ts_assert_boolean_not_equal(wb_camera_recognition_has_segmentation(camera),
                              "Camera recognition segmentation should be FALSE after changing the field value.");

  wb_robot_step(TIME_STEP);
  object_number = wb_camera_recognition_get_number_of_objects(camera);
  const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);

  ts_assert_int_equal(object_number, VISIBLE_SOLID_NUMBER,
                      "The camera should initially see %d objects and not %d (with occlusion).", VISIBLE_SOLID_NUMBER,
                      object_number);
  int sub_solid_index = 0;
  int composed_solid_index = 0;
  for (i = 0; i < object_number; ++i) {
    // check position on the image of the 'visible sphere without BO' solid
    if (strcmp(objects[i].model, "visible sphere without BO") == 0) {
      int expected_position_on_image[2] = {189, 41};
      ts_assert_integers_in_delta(
        2, objects[i].position_on_image, expected_position_on_image, 1,
        "Image coordinate of the 'visible sphere without BO' solid is not correct: found (%d, %d), expected (%d, %d).",
        objects[i].position_on_image[0], objects[i].position_on_image[1], expected_position_on_image[0],
        expected_position_on_image[1]);
    }
    // check objct is one of the visible solid
    bool found = false;
    for (j = 0; j < VISIBLE_SOLID_NUMBER; ++j) {
      if (strcmp(objects[i].model, visible_solid_models[j]) == 0)
        found = true;
    }
    ts_assert_boolean_equal(found, "Object '%s' should not be visible.\n", objects[i].model);
    // remove solid from list
    WbNodeRef node_ref = wb_supervisor_node_get_from_id(objects[i].id);
    wb_supervisor_node_remove(node_ref);
    if (strcmp(objects[i].model, "sub solid") == 0)
      sub_solid_index = i;
    if (strcmp(objects[i].model, "composed solid") == 0)
      composed_solid_index = i;
  }

  double sub_solid_size = objects[sub_solid_index].size_on_image[0] * objects[sub_solid_index].size_on_image[1];
  double composed_solid_size = objects[composed_solid_index].size_on_image[0] * objects[composed_solid_index].size_on_image[1];
  ts_assert_double_is_bigger(composed_solid_size, sub_solid_size, "Object: '%s' should have a bigger pixel size than '%s'.",
                             objects[composed_solid_index].model, objects[sub_solid_index].model);

  wb_robot_step(TIME_STEP);

  object_number = wb_camera_recognition_get_number_of_objects(camera);
  objects = wb_camera_recognition_get_objects(camera);

  ts_assert_int_equal(object_number, 1, "The camera should see only 1 objects after removal of the initial objects and not %d.",
                      object_number);
  ts_assert_int_equal(strcmp(objects[0].model, occcluded_solid_model), 0,
                      "The last remaining object should be '%s' and not '%s'.", occcluded_solid_model, objects[0].model);

  int number_of_colors = objects[0].number_of_colors;
  ts_assert_int_equal(number_of_colors, 2, "The last remaining object should have 2 colors and not %d.", number_of_colors);
  int red1 = objects[0].colors[0] * 255;
  int green1 = objects[0].colors[1] * 255;
  int blue1 = objects[0].colors[2] * 255;
  int red2 = objects[0].colors[3] * 255;
  int green2 = objects[0].colors[4] * 255;
  int blue2 = objects[0].colors[5] * 255;

  ts_assert_color_in_delta(red1, green1, blue1, 0, 255, 0, 0, "Wrong first color for object '%s'.", objects[0].model);
  ts_assert_color_in_delta(red2, green2, blue2, 255, 0, 255, 0, "Wrong second color for object '%s'.", objects[0].model);

  // check segmentation image content after changing recognition color
  wb_supervisor_field_set_sf_bool(segmentation_field, true);
  wb_robot_step(TIME_STEP);
  wb_camera_recognition_enable_segmentation(camera);
  wb_camera_recognition_enable_segmentation(camera_spherical);
  wb_camera_recognition_enable_segmentation(camera_cylindrical);
  wb_robot_step(TIME_STEP);
  image = wb_camera_recognition_get_segmentation_image(camera);
  const int solid_x = 124;
  const int solid_y = 200;
  red = wb_camera_image_get_red(image, width, solid_x, solid_y);
  green = wb_camera_image_get_green(image, width, solid_x, solid_y);
  blue = wb_camera_image_get_blue(image, width, solid_x, solid_y);
  ts_assert_color_in_delta(red, green, blue, 0, 255, 0, 2,
                           "Wrong green box segmentation color before changing the recognition color.");

  WbNodeRef green_box = wb_supervisor_node_get_from_def("GREEN_BOX");
  WbFieldRef color_field = wb_supervisor_node_get_field(green_box, "recognitionColors");
  const double new_color[3] = {0.5, 0.5, 0.5};
  wb_supervisor_field_set_mf_color(color_field, 0, new_color);
  wb_robot_step(TIME_STEP);
  image = wb_camera_recognition_get_segmentation_image(camera);
  red = wb_camera_image_get_red(image, width, solid_x, solid_y);
  green = wb_camera_image_get_green(image, width, solid_x, solid_y);
  blue = wb_camera_image_get_blue(image, width, solid_x, solid_y);
  ts_assert_color_in_delta(red, green, blue, 127, 127, 127, 2,
                           "Wrong green box segmentation color after changing the recognition color.");

  // test spherical camera
  const double invisible_capsule_position[3] = {0.369, 1.650, 0.899};
  const double invisible_capsule_orientation[4] = {0.577350, -0.577350, -0.577350, 2.094390};
  const double invisible_capsule_size[2] = {0.1, 0.1};
  object_number = wb_camera_recognition_get_number_of_objects(camera_spherical);
  ts_assert_int_equal(object_number, 4,
                      "The spherical camera should see only 4 objects after removal of the initial objects and not %d.",
                      object_number);
  objects = wb_camera_recognition_get_objects(camera_spherical);
  image = wb_camera_recognition_get_segmentation_image(camera_spherical);
  for (i = 0; i < object_number; ++i) {
    if (strcmp(objects[i].model, occcluded_solid_model) == 0) {
      // position
      double expected_position[3] = {-0.550, 0.0, 0.429};
      ts_assert_doubles_in_delta(
        3, objects[i].position, expected_position, 0.001,
        "Position of 'occluded box' is not correct for spherical camera: found=(%f, %f, %f), expected=(%f, %f, %f).",
        objects[i].position[0], objects[i].position[1], objects[i].position[2], expected_position[0], expected_position[1],
        expected_position[2]);
      // orientation
      double expected_orientation[4] = {0.577350, -0.577350, -0.577350, 2.094390};
      ts_assert_doubles_in_delta(
        4, objects[i].orientation, expected_orientation, 0.001,
        "Orientation of 'occluded box' is not correct for spherical camera: found=(%f, %f, %f, %f), expected=(%f, %f, %f, %f).",
        objects[i].orientation[0], objects[i].orientation[1], objects[i].orientation[2], objects[i].orientation[3],
        expected_orientation[0], expected_orientation[1], expected_orientation[2], expected_orientation[3]);
      // size
      double expected_size[2] = {0.1, 0.9};
      ts_assert_doubles_in_delta(
        2, objects[i].size, expected_size, 0.001,
        "Size of 'occluded box' is not correct for spherical camera: found=(%f, %f), expected=(%f, %f).", objects[i].size[0],
        objects[i].size[1], expected_size[0], expected_size[1]);
      // size on image
      int expected_size_on_image[2] = {146, 42};
      ts_assert_integers_in_delta(
        2, objects[i].size_on_image, expected_size_on_image, 1,
        "Size on image of 'occluded box' is not correct for spherical camera: found=(%d, %d), expected=(%d, %d).",
        objects[i].size_on_image[0], objects[i].size_on_image[1], expected_size_on_image[0], expected_size_on_image[1]);
      // position on image
      int expected_position_on_image[2] = {128, 47};
      ts_assert_integers_in_delta(
        2, objects[i].position_on_image, expected_position_on_image, 1,
        "Position on image of 'occluded box' is not correct for spherical camera: found=(%d, %d), expected=(%d, %d).",
        objects[i].position_on_image[0], objects[i].position_on_image[1], expected_position_on_image[0],
        expected_position_on_image[1]);

      // check segmentation image content
      int u = 69, v = 47;
      red = wb_camera_image_get_red(image, width, u, v);
      green = wb_camera_image_get_green(image, width, u, v);
      blue = wb_camera_image_get_blue(image, width, u, v);
      ts_assert_color_in_delta(red, green, blue, 127, 127, 127, 2,
                               "Wrong green box segmentation color in spherical camera image.");

    } else if (strcmp(objects[i].model, "invisible capsule") == 0) {
      // position
      ts_assert_doubles_in_delta(
        3, objects[i].position, invisible_capsule_position, 0.001,
        "Position of 'invisble capsule' is not correct for spherical camera: found=(%f, %f, %f), expected=(%f, %f, %f).",
        objects[i].position[0], objects[i].position[1], objects[i].position[2], invisible_capsule_position[0],
        invisible_capsule_position[1], invisible_capsule_position[2]);
      // orientation
      ts_assert_doubles_in_delta(4, objects[i].orientation, invisible_capsule_orientation, 0.001,
                                 "Orientation of 'invisble capsule' is not correct for spherical camera: found=(%f, %f, %f, "
                                 "%f), expected=(%f, %f, %f, %f).",
                                 objects[i].orientation[0], objects[i].orientation[1], objects[i].orientation[2],
                                 objects[i].orientation[3], invisible_capsule_orientation[0], invisible_capsule_orientation[1],
                                 invisible_capsule_orientation[2], invisible_capsule_orientation[3]);
      // size
      ts_assert_doubles_in_delta(
        2, objects[i].size, invisible_capsule_size, 0.001,
        "Size of 'invisble capsule' is not correct for spherical camera: found=(%f, %f), expected=(%f, %f).",
        objects[i].size[0], objects[i].size[1], invisible_capsule_size[0], invisible_capsule_size[1]);
      // size on image
      int expected_size_on_image[2] = {14, 28};
      ts_assert_integers_in_delta(
        2, objects[i].size_on_image, expected_size_on_image, 1,
        "Size on image of 'invisble capsule' is not correct for spherical camera: found=(%d, %d), expected=(%d, %d).",
        objects[i].size_on_image[0], objects[i].size_on_image[1], expected_size_on_image[0], expected_size_on_image[1]);
      // position on image
      int expected_position_on_image[2] = {79, 102};
      ts_assert_integers_in_delta(
        2, objects[i].position_on_image, expected_position_on_image, 1,
        "Position on image of 'invisble capsule' is not correct for spherical camera: found=(%d, %d), expected=(%d, %d).",
        objects[i].position_on_image[0], objects[i].position_on_image[1], expected_position_on_image[0],
        expected_position_on_image[1]);
    }
  }

  // test cylindrical camera
  // position, orientation and size of objects should be the same as for spherical camera (same translation and rotation)
  object_number = wb_camera_recognition_get_number_of_objects(camera_cylindrical);
  ts_assert_int_equal(object_number, 3,
                      "The cylidrical camera should see only 3 objects after removal of the initial objects and not %d.",
                      object_number);
  objects = wb_camera_recognition_get_objects(camera_cylindrical);
  image = wb_camera_recognition_get_segmentation_image(camera_cylindrical);
  for (i = 0; i < object_number; ++i) {
    if (strcmp(objects[i].model, "invisible capsule") == 0) {
      // position
      ts_assert_doubles_in_delta(
        3, objects[i].position, invisible_capsule_position, 0.001,
        "Position of 'invisble capsule' is not correct for cylindrical camera: found=(%f, %f, %f), expected=(%f, %f, %f).",
        objects[i].position[0], objects[i].position[1], objects[i].position[2], invisible_capsule_position[0],
        invisible_capsule_position[1], invisible_capsule_position[2]);
      // orientation
      ts_assert_doubles_in_delta(4, objects[i].orientation, invisible_capsule_orientation, 0.001,
                                 "Orientation of 'invisble capsule' is not correct for cylindrical camera: found=(%f, %f, %f, "
                                 "%f), expected=(%f, %f, %f, %f).",
                                 objects[i].orientation[0], objects[i].orientation[1], objects[i].orientation[2],
                                 objects[i].orientation[3], invisible_capsule_orientation[0], invisible_capsule_orientation[1],
                                 invisible_capsule_orientation[2], invisible_capsule_orientation[3]);
      // size
      ts_assert_doubles_in_delta(
        2, objects[i].size, invisible_capsule_size, 0.001,
        "Size of 'invisble capsule' is not correct for cylindrical camera: found=(%f, %f), expected=(%f, %f).",
        objects[i].size[0], objects[i].size[1], invisible_capsule_size[0], invisible_capsule_size[1]);
      // size on image
      int expected_size_on_image[2] = {6, 45};
      ts_assert_integers_in_delta(
        2, objects[i].size_on_image, expected_size_on_image, 1,
        "Size on image of 'invisble capsule' is not correct for cylindrical camera: found=(%d, %d), expected=(%d, %d).",
        objects[i].size_on_image[0], objects[i].size_on_image[1], expected_size_on_image[0], expected_size_on_image[1]);
      // position on image
      int expected_position_on_image[2] = {12, 88};
      ts_assert_integers_in_delta(
        2, objects[i].position_on_image, expected_position_on_image, 1,
        "Position on image of 'invisble capsule' is not correct for cylindrical camera: found=(%d, %d), expected=(%d, %d).",
        objects[i].position_on_image[0], objects[i].position_on_image[1], expected_position_on_image[0],
        expected_position_on_image[1]);

      // check segmentation image content
      int u = 13, v = 92;
      red = wb_camera_image_get_red(image, width, u, v);
      green = wb_camera_image_get_green(image, width, u, v);
      blue = wb_camera_image_get_blue(image, width, u, v);
      ts_assert_color_in_delta(red, green, blue, 0, 0, 255, 2,
                               "Wrong blue capsule segmentation color in cylindrical camera image.");
    }
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
