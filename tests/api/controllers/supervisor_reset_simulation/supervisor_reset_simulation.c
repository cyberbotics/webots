/*
 * Description:  Test Supervisor device API
 *               This file contains tests of reset simulation method
 */

#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define SOLID_BOX_NUMBER 5

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  WbDeviceTag camera[2];
  camera[0] = wb_robot_get_device("camera0");
  camera[1] = wb_robot_get_device("camera1");

  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef root_children = wb_supervisor_node_get_field(root_node, "children");

  WbNodeRef self_node = wb_supervisor_node_get_self();
  WbFieldRef controllerArgs_field = wb_supervisor_node_get_field(self_node, "controllerArgs");
  WbFieldRef description_field = wb_supervisor_node_get_field(self_node, "description");

  WbNodeRef box_node = wb_supervisor_node_get_from_def("BOX");
  WbNodeRef camera_node = wb_supervisor_node_get_from_def("CAMERA0");

  WbNodeRef solid_box[SOLID_BOX_NUMBER];
  solid_box[0] = wb_supervisor_node_get_from_def("SOLID0");
  solid_box[1] = wb_supervisor_node_get_from_def("SOLID1");
  solid_box[2] = wb_supervisor_node_get_from_def("SOLID2");
  solid_box[3] = wb_supervisor_node_get_from_def("SOLID3");
  solid_box[4] = wb_supervisor_node_get_from_def("SOLID4");

  ts_assert_pointer_not_null(box_node, "The supervisor cannot get the WbNodeRef of BOX.");
  ts_assert_pointer_not_null(camera_node, "The supervisor cannot get the WbNodeRef of CAMERA0.");

  wb_camera_enable(camera[0], TIME_STEP);
  wb_camera_enable(camera[1], TIME_STEP);

  wb_robot_step(TIME_STEP);

  const unsigned char *image = wb_camera_get_image(camera[0]);
  ts_assert_color_in_delta(image[2], image[1], image[0], 207, 0, 0, 0,
                           "CAMERA0 should see the red box before changing it's visibility.");

  image = wb_camera_get_image(camera[1]);
  ts_assert_color_in_delta(image[2], image[1], image[0], 0, 0, 255, 0,
                           "CAMERA1 should see the background before importing the second box.");

  wb_supervisor_field_import_mf_node_from_string(root_children, -1,
                                                 "Solid { translation 1.3 0 0 children [ USE BOX_SHAPE ] name \"solid(1)\" }");
  wb_supervisor_node_set_visibility(box_node, camera_node, false);

  wb_robot_step(TIME_STEP);

  image = wb_camera_get_image(camera[0]);
  ts_assert_color_in_delta(image[2], image[1], image[0], 0, 0, 255, 0,
                           "CAMERA0 should see the background after changing the visibility of the first box.");

  image = wb_camera_get_image(camera[1]);
  ts_assert_color_in_delta(image[2], image[1], image[0], 207, 0, 0, 0,
                           "CAMERA1 should see the second red box before importing it.");

  while (wb_robot_step(TIME_STEP) != -1 && wb_robot_get_time() < 2.0)
    ;

  char file_path[128];
  int iteration;
  sscanf(wb_robot_get_custom_data(), "%d %s", &iteration, file_path);

  if (strcmp(argv[1], "supervisor_reset_simulation_iteration_0") == 0) {
    wb_supervisor_field_set_mf_string(controllerArgs_field, 0, "supervisor_reset_simulation_iteration_1");
    ts_assert_int_equal(iteration, 0,
                        "Robot custom data should start with '0' at first iteration, not '%d'. The PROTO has been regenerated "
                        "since Webots started.",
                        iteration);
    // save position of the solid box in the 'description' field
    int i;
    char buffer[512];
    buffer[0] = '\0';
    for (i = 0; i < SOLID_BOX_NUMBER; ++i) {
      const double *position = wb_supervisor_node_get_position(solid_box[i]);
      char buffer2[64];
      sprintf(buffer2, "%.8lf %.8lf %.8lf ", position[0], position[1], position[2]);
      strcat(buffer, buffer2);
    }
    wb_supervisor_field_set_sf_string(description_field, buffer);
    wb_robot_step(TIME_STEP);
    // reset simulation
    wb_supervisor_simulation_reset();
    wb_supervisor_node_restart_controller(wb_supervisor_node_get_self());
    wb_supervisor_node_restart_controller(wb_supervisor_node_get_from_def("TEST_SUITE_SUPERVISOR"));
  } else if (strcmp(argv[1], "supervisor_reset_simulation_iteration_1") == 0) {
    wb_supervisor_field_set_mf_string(controllerArgs_field, 0, "supervisor_reset_simulation_iteration_2");
    // check that the non-deterministic PROTO node was regenerated
    ts_assert_int_equal(iteration, 1, "Robot custom data should start with '1' at second iteration.");
    // read expected position from the 'description' field and compare with current position
    char *buffer = (char *)wb_supervisor_field_get_sf_string(description_field);
    int i;
    for (i = 0; i < SOLID_BOX_NUMBER; ++i) {
      double expected_x, expected_y, expected_z;
      sscanf(buffer, "%lf %lf %lf", &expected_x, &expected_y, &expected_z);
      const double *position = wb_supervisor_node_get_position(solid_box[i]);
      ts_assert_vec3_in_delta(position[0], position[1], position[2], expected_x, expected_y, expected_z, 0.000001,
                              "Position of solid box %d is not equal to the one of the first run.", i);
      if (i != SOLID_BOX_NUMBER - 1) {
        int j;
        for (j = 0; j < 3; ++j)
          buffer = strchr(buffer, ' ') + 1;
      }
    }
    // reset the counter
    remove(file_path);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
