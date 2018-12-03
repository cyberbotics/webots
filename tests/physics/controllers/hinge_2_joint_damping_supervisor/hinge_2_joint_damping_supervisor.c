#include <webots/robot.h>
#include <webots/supervisor.h>

#include <stdio.h>

#define TIME_STEP 64

const double damping[3] = {0, 20, 50};

int main(int argc, char **argv) {
  int i;
  wb_robot_init();
  WbNodeRef wheel_parameters_nodes[2];
  wheel_parameters_nodes[0] = wb_supervisor_node_get_from_def("WHEEL1PARAMETERS");
  wheel_parameters_nodes[1] = wb_supervisor_node_get_from_def("WHEEL2PARAMETERS");

  WbFieldRef damping_fields[2];
  damping_fields[0] = wb_supervisor_node_get_field(wheel_parameters_nodes[0], "dampingConstant");
  damping_fields[1] = wb_supervisor_node_get_field(wheel_parameters_nodes[1], "dampingConstant");

  wb_supervisor_field_set_sf_float(damping_fields[0], damping[0]);
  wb_supervisor_field_set_sf_float(damping_fields[1], damping[0]);

  for (i = 0; i < 10; ++i)
    wb_robot_step(TIME_STEP);

  wb_supervisor_field_set_sf_float(damping_fields[0], damping[1]);
  wb_supervisor_field_set_sf_float(damping_fields[1], damping[1]);

  for (i = 0; i < 10; ++i)
    wb_robot_step(TIME_STEP);

  wb_supervisor_field_set_sf_float(damping_fields[0], damping[2]);
  wb_supervisor_field_set_sf_float(damping_fields[1], damping[2]);

  wb_robot_cleanup();
  return 0;
}
