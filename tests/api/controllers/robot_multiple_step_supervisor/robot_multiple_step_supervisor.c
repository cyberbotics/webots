#include <webots/robot.h>
#include <webots/supervisor.h>

int main(int argc, char **argv) {
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();

  wb_robot_step(3 * time_step);

  // update world
  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
  WbFieldRef rotationField = wb_supervisor_node_get_field(robot, "rotation");
  WbFieldRef translationField = wb_supervisor_node_get_field(robot, "translation");
  double newRotation[4] = {0.0, 1.0, 0.0, 1.5708};
  double newTranslation[3] = {-0.05, 0.0295, 0.1};
  wb_supervisor_field_set_sf_rotation(rotationField, newRotation);
  wb_supervisor_field_set_sf_vec3f(translationField, newTranslation);

  wb_robot_cleanup();

  return 0;
}
