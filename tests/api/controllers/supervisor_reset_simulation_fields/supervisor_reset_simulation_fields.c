/*
 * Description:  Test Supervisor device API
 *               This file contains tests of reset simulation method
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define SOLID_BOX_NUMBER 5

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  const double new_translation[3] = {1, 0, 0};
  WbFieldRef translation_field = wb_supervisor_node_get_field(wb_supervisor_node_get_from_def("SOLID"), "translation");
  if (strcmp(argv[1], "supervisor_reset_simulation_fields_resetter") == 0) {
    wb_robot_step(TIME_STEP);

    // <Other controller verifies SOLID's inital position>

    wb_robot_step(TIME_STEP);
    wb_supervisor_simulation_reset();
    wb_supervisor_field_set_sf_vec3f(translation_field, new_translation);
  } else {
    wb_robot_step(TIME_STEP);
    const double *first_translation = wb_supervisor_field_get_sf_vec3f(translation_field);
    ts_assert_vec3_equal(first_translation[0], first_translation[1], first_translation[2], 0, 0, 1,
                         "SOLID's initial position should be [0, 0, 1] not [%f, %f, %f]");
    wb_robot_step(TIME_STEP);

    // <Other controller resets the simulation and updates SOLID's position>

    wb_robot_step(TIME_STEP);
    const double *second_translation = wb_supervisor_field_get_sf_vec3f(translation_field);
    ts_assert_vec3_equal(second_translation[0], second_translation[1], second_translation[2],
                         new_translation[0], new_translation[1], new_translation[2],
                         "SOLID's position should be [1, 0, 0] not [%f, %f, %f] after reset");
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
