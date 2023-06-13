/*
 * Description:  Test supervisor import function in case of a robot object.
 *               The controller of the imported robot has to be started
 *               automatically and after some step the position of the robot
 *               has to be different from the inial one.
 */

#include <webots/camera.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 64

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  const double initialPosition[3] = {0.0, 0.0, -0.104};

  WbNodeRef root = wb_supervisor_node_get_root();
  WbFieldRef rootChildren = wb_supervisor_node_get_field(root, "children");

  wb_robot_step(2 * TIME_STEP);

  // import robot object
  FILE *fd = fopen("MyBot.txt", "rb");
  char contents[4096];
  const int n = fread(contents, 1, sizeof(contents), fd);
  fclose(fd);
  contents[n] = '\0';  // end of string
  ts_assert_int_equal(n, 3262, "The MyBot.txt file could be fully read: %d/3262 bytes read", n);
  wb_supervisor_field_import_mf_node_from_string(rootChildren, -1, contents);

  // check imported robot position
  WbNodeRef robot = wb_supervisor_node_get_from_def("MY_BOT");
  ts_assert_boolean_equal(robot != NULL, "The robot node has not been imported correctly");

  const double *position = wb_supervisor_node_get_position(robot);
  ts_assert_doubles_in_delta(3, position, initialPosition, 0.0001, "The robot has not been imported at the correct position");

  wb_robot_step(4 * TIME_STEP);

  // check that robot moves forwards
  double previousZ = position[2];
  position = wb_supervisor_node_get_position(robot);
  ts_assert_boolean_equal(position[2] < (previousZ - 0.02), "The controller of the imported robot has not started correctly");

  // import device after controller start
  WbNodeRef self_node = wb_supervisor_node_get_self();
  WbFieldRef self_children = wb_supervisor_node_get_field(self_node, "children");
  wb_supervisor_field_import_mf_node_from_string(self_children, 0, "Camera { name \"imported camera\"}");

  WbDeviceTag camera = wb_robot_get_device("imported camera");
  ts_assert_int_not_equal(camera, 0, "Camera imported during controller execution not found.");
  wb_camera_enable(camera, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const unsigned char *image = wb_camera_get_image(camera);
  ts_assert_boolean_not_equal(image == NULL, "Camera image is NULL.");

  ts_send_success();
  return EXIT_SUCCESS;
}
