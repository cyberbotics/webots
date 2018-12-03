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
                      "The camera should initialy see %d objects and not %d (without occlusion).", VISIBLE_SOLID_NUMBER + 1,
                      object_number);

  // enable occlusion
  WbNodeRef recognition_node = wb_supervisor_node_get_from_def("RECOGNITION");
  WbFieldRef occlusion_field = wb_supervisor_node_get_field(recognition_node, "occlusion");
  wb_supervisor_field_set_sf_bool(occlusion_field, true);

  wb_robot_step(TIME_STEP);
  object_number = wb_camera_recognition_get_number_of_objects(camera);
  const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);

  ts_assert_int_equal(object_number, VISIBLE_SOLID_NUMBER,
                      "The camera should initialy see %d objects and not %d (with occlusion).", VISIBLE_SOLID_NUMBER,
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
