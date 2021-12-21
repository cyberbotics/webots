#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/skin.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

static void check_image(WbDeviceTag camera, const int *color, const char *message) {
  const unsigned char *image = wb_camera_get_image(camera);
  const int r = wb_camera_image_get_red(image, 1, 0, 0);
  const int g = wb_camera_image_get_green(image, 1, 0, 0);
  const int b = wb_camera_image_get_blue(image, 1, 0, 0);
  ts_assert_color_in_delta(r, g, b, color[0], color[1], color[2], 5, message, r, g, b);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  /* Test graphical skeleton */
  WbDeviceTag graphical_skin = wb_robot_get_device("graphical skin");
  const int bones_count = wb_skin_get_bone_count(graphical_skin);
  ts_assert_int_equal(bones_count, 31, "Wrong number of bones in Anthony FBX skeleton.");

  int spine_bone_index = 7;
  ts_assert_string_equal(wb_skin_get_bone_name(graphical_skin, 0), "Hips", "Wrong name of root bone.");
  ts_assert_string_equal(wb_skin_get_bone_name(graphical_skin, spine_bone_index), "Spine", "Wrong name of 7th bone.");

  const double initial_root_position[3] = {0.0, 0.692374, -0.039902};
  const double *root_p = wb_skin_get_bone_position(graphical_skin, 0, false);
  ts_assert_doubles_in_delta(3, root_p, initial_root_position, 0.000001, "Wrong initial position of root bone.");

  const double initial_spine_position[3] = {0.0, 0.861315, -0.057307};
  const double *spine_p = wb_skin_get_bone_position(graphical_skin, spine_bone_index, true);
  ts_assert_doubles_in_delta(3, spine_p, initial_spine_position, 0.000001, "Wrong initial position of spine bone.");

  const double initial_spine_orientation[4] = {0.0, 0.0, -1.000001, 0.346287};
  const double *spine_o = wb_skin_get_bone_orientation(graphical_skin, spine_bone_index, false);
  ts_assert_doubles_in_delta(4, spine_o, initial_spine_orientation, 0.000001, "Wrong initial orientation of spine bone.");

  const double new_root_position[3] = {-0.5, initial_root_position[1], initial_root_position[2]};
  wb_skin_set_bone_position(graphical_skin, 0, new_root_position, false);

  const double expected_spine_position[3] = {initial_spine_position[0] - 0.5, initial_spine_position[1],
                                             initial_spine_position[2]};
  const double *spine_p_after = wb_skin_get_bone_position(graphical_skin, spine_bone_index, true);
  ts_assert_doubles_in_delta(3, spine_p_after, expected_spine_position, 0.000001, "Wrong position of spine bone after move.");

  wb_robot_step(TIME_STEP);

  double new_spine_orientation[4] = {initial_spine_orientation[0], initial_spine_orientation[1], initial_spine_orientation[2],
                                     -0.5};
  wb_skin_set_bone_orientation(graphical_skin, spine_bone_index, new_spine_orientation, false);

  const double expected_spine_orientation[4] = {0.275161, 0.921180, 0.275161, 1.652804};
  const double *spine_o_after = wb_skin_get_bone_orientation(graphical_skin, spine_bone_index, true);
  ts_assert_doubles_in_delta(4, spine_o_after, expected_spine_orientation, 0.000001,
                             "Wrong orientation of spine bone after rotation.");

  wb_robot_step(TIME_STEP);

  /* Test physical skeleton */
  const int skin_color[3] = {153, 0, 0};
  const int sky_color[3] = {102, 178, 255};

  check_image(camera, skin_color, "Skin mesh is not detected at the beginning of the test: detected color [%d %d %d].");

  WbDeviceTag physical_skin = wb_robot_get_device("physical skin");
  const int physical_bones_count = wb_skin_get_bone_count(physical_skin);
  ts_assert_int_equal(physical_bones_count, 2, "Wrong number of bones in simple_skin FBX skeleton.");

  ts_assert_string_equal(wb_skin_get_bone_name(physical_skin, 0), "Head", "Wrong name of physical first bone.");
  ts_assert_string_equal(wb_skin_get_bone_name(physical_skin, 1), "Tail", "Wrong name of physical second bone.");

  const double expected_head_bone_orientation[4] = {0.0, 0.0, -0.25};
  const double *physical_root_position = wb_skin_get_bone_position(physical_skin, 0, false);
  ts_assert_doubles_in_delta(3, physical_root_position, expected_head_bone_orientation, 0.000001,
                             "Wrong initial position of head bone.");
  const double new_physical_root_position[3] = {physical_root_position[0], 1.0, physical_root_position[2]};
  wb_skin_set_bone_position(graphical_skin, 0, new_physical_root_position, false);

  wb_robot_step(TIME_STEP);

  check_image(camera, skin_color, "Skin mesh is not detected after invalid change of position: detected color [%d %d %d].");

  const double new_physical_root_orientation[4] = {0.0, 1.0, 0.0, 1.5708};
  wb_skin_set_bone_orientation(physical_skin, 0, new_physical_root_orientation, false);

  wb_robot_step(TIME_STEP);

  check_image(camera, skin_color, "Skin mesh is not detected after invalid change of orientation: detected color [%d %d %d].");

  WbNodeRef parameters_node = wb_supervisor_node_get_from_def("JOINT_PARAMETERS");
  WbFieldRef joint_position_field = wb_supervisor_node_get_field(parameters_node, "position");
  wb_supervisor_field_set_sf_float(joint_position_field, 1.3);

  wb_robot_step(TIME_STEP);

  check_image(camera, sky_color, "Skin mesh is still detected after moving physical skeleton: detected color [%d %d %d].");

  ts_send_success();
  return EXIT_SUCCESS;
}
