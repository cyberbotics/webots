#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <string.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define VISIBLE_SOLID_NUMBER 6

static const char *visible_solid_models[VISIBLE_SOLID_NUMBER] = {
  "visible sphere", "visible box", "sub solid", "visible capsule", "composed solid", "visible sphere without BO"};

static const char *occcluded_solid_model = "occluded box";

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  int i, j;

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);

  // check number of object recognized
  wb_robot_step(TIME_STEP);
  int object_number = wb_camera_recognition_get_number_of_objects(camera);
  ts_assert_int_equal(object_number, VISIBLE_SOLID_NUMBER + 1,
                      "The camera should initially see %d objects and not %d (without occlusion).", VISIBLE_SOLID_NUMBER + 1,
                      object_number);

  // enable occlusion
  WbNodeRef recognition_node = wb_supervisor_node_get_from_def("RECOGNITION");
  WbFieldRef occlusion_field = wb_supervisor_node_get_field(recognition_node, "occlusion");
  wb_supervisor_field_set_sf_bool(occlusion_field, true);

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
  red = wb_camera_image_get_red(image, width, 85, 200);
  green = wb_camera_image_get_green(image, width, 85, 200);
  blue = wb_camera_image_get_blue(image, width, 85, 200);
  ts_assert_color_in_delta(red, green, blue, 0, 255, 0, 5,
                           "Image should contain a green box at (85, 200)): found color (%d, %d, %d) expected (%d, %d, %d).",
                           red, green, blue, 0, 255, 0);
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
      ts_assert_int_equal(objects[i].position_on_image[0], 191,
                          "Image coordinate of the 'visible sphere without BO' solid is not correct.");
      ts_assert_int_equal(objects[i].position_on_image[1], 41,
                          "Image coordinate of the 'visible sphere without BO' solid is not correct.");
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

  ts_send_success();
  return EXIT_SUCCESS;
}
