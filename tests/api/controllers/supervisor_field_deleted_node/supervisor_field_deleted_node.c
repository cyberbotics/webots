#include <stdio.h>

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define N_TARGETS 5

int main(int argc, char **argv) {
  const bool is_remover = argv[1][0] == 'R';
  if (is_remover)
    wb_robot_init();
  else
    ts_setup(argv[0]);

  WbFieldRef sky = wb_supervisor_node_get_field(wb_supervisor_node_get_from_def("BACKGROUND"), "skyColor");
  WbNodeRef targets[N_TARGETS];
  WbFieldRef translations[N_TARGETS];
  for (int i = 0; i < N_TARGETS; ++i) {
    char def[8];
    sprintf(def, "T%d", i);
    targets[i] = wb_supervisor_node_get_from_def(def);
    translations[i] = wb_supervisor_node_get_field(targets[i], "translation");
  }

  if (is_remover) {
    for (int i = 0; i < N_TARGETS; ++i) {
      wb_robot_step(TIME_STEP);
      wb_supervisor_node_remove(targets[i]);
      wb_supervisor_field_remove_mf(sky, -1);
    }
    wb_robot_step(TIME_STEP);
    return EXIT_SUCCESS;
  }

  WbNodeRef self_target = wb_supervisor_node_get_from_def("SELF_TARGET");
  const double position[3] = {0.1, 0.2, 0.3};
  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(self_target, "translation"), position);
  wb_supervisor_node_remove(self_target);

  for (int step = 0; step < 2 * N_TARGETS; ++step) {
    for (int i = 0; i < N_TARGETS; ++i)
      wb_supervisor_field_set_sf_vec3f(translations[i], position);
    const int last = wb_supervisor_field_get_count(sky) - 1;
    if (last >= 0) {
      wb_supervisor_field_set_mf_color(sky, last, position);
      wb_supervisor_field_get_mf_color(sky, last);
    }
    wb_robot_step(TIME_STEP);
  }

  ts_assert_int_equal(wb_supervisor_field_get_count(sky), 1, "skyColor should only keep its first item");
  ts_send_success();
  return EXIT_SUCCESS;
}
