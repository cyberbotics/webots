#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

static const char *temp_world_file_path = "../../worlds/check_saved_hidden_parameters_temp.wbt";

static void check_robot_position(int time_step) {
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);
  wb_robot_step(time_step);
  int image_width = wb_camera_get_width(camera);
  int image_height = wb_camera_get_height(camera);
  const unsigned char *image = wb_camera_get_image(camera);
  int x, y;
  for (x = 0; x < image_width; x++) {
    for (y = 0; y < image_height; y++) {
      if (y < 12 || (y < 28 && (x < 10 || x > 23))) {
        int value = wb_camera_image_get_gray(image, image_width, x, y);
        ts_assert_int_equal(value, 255, "Wrong arm position: camera image pixel at [x=%d, y=%d] should be white.", x, y);
      }
    }
  }
}

int main(int argc, char **argv) {
  wb_robot_init();
  sprintf(ts_test_name, "check_saved_hidden_parameter");

  int time_step = wb_robot_get_basic_time_step();
  const char *custom_data = wb_robot_get_custom_data();

  if (strcmp(custom_data, "init") == 0) {
    // move robot arm and save to temporary world file
    wb_robot_step(32 * time_step);  // wait until arm moved

    wb_robot_set_custom_data("save1");

    WbNodeRef robot_node = wb_supervisor_node_get_from_def("ROBOT");
    WbFieldRef controller_field = wb_supervisor_node_get_field(robot_node, "controller");
    wb_supervisor_field_set_sf_string(controller_field, "<generic>");

    wb_supervisor_world_save(temp_world_file_path);
    wb_supervisor_world_load(temp_world_file_path);

  } else if (strcmp(custom_data, "save1") == 0) {
    // save again
    wb_robot_set_custom_data("save2");
    wb_supervisor_world_save(temp_world_file_path);
    wb_supervisor_world_load(temp_world_file_path);

  } else if (strcmp(custom_data, "save2") == 0) {
    // save again
    wb_robot_set_custom_data("test");
    wb_supervisor_world_save(temp_world_file_path);
    wb_supervisor_world_load(temp_world_file_path);

  } else if (strcmp(custom_data, "test") == 0) {
    ts_setup_done = true;
    ts_notify_controller_status(true);

    // check initial position of the robot
    check_robot_position(time_step);

    ts_send_success();
    return EXIT_SUCCESS;
  }

  wb_robot_cleanup();

  return 0;
}
