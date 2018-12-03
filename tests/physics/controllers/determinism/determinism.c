#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/file_utils.h"
#include "../../../lib/string_utils.h"
#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <time.h>

#define TIME_STEP 16

static const char *ode_dif = "../../plugins/physics/ode_dif_exporter/ode.dif";
static const char *ode_tmp_dif = "ode_tmp.dif";

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  if (file_exists(ode_dif))
    ts_assert_boolean_equal(remove_file(ode_dif), "Cannot remove ODE dif file");

  time_t now;
  time(&now);

  if (file_exists(ode_tmp_dif)) {
    double delta = difftime(now, file_get_creation_time(ode_tmp_dif));
    // printf("delta time = %f\n", delta);
    if (delta > 2.0) {
      ts_assert_boolean_equal(remove_file(ode_tmp_dif), "Cannot remove ODE tmp dif file");
      ts_assert_boolean_not_equal(file_exists(ode_tmp_dif), "Cannot remove ODE tmp dif file");
    }
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    while (wb_receiver_get_queue_length(receiver) > 0) {
      const char *message = wb_receiver_get_data(receiver);
      if (string_starts_with("Failure", message))
        ts_assert_boolean_equal(0, "%s", message);

      ts_assert_boolean_equal(file_exists(ode_dif), "Expected ODE dif file not present");

      if (file_exists(ode_tmp_dif)) {
        // a previous simulation has run
        if (compare_file_content(ode_tmp_dif, ode_dif)) {
          if (file_contains_string(ode_dif, "body[5]") && file_contains_string(ode_dif, "dynamics.hinge_joint")) {
            ts_send_success();
            return EXIT_SUCCESS;
          } else {
            ts_assert_boolean_equal(0, "The ODE dif file doesn't contain the expected patterns");
            wb_robot_cleanup();
            exit(EXIT_FAILURE);
          }
        } else {
          ts_assert_boolean_equal(0, "The ODE dif files contents differ");
          wb_robot_cleanup();
          exit(EXIT_FAILURE);
        }
      } else {
        move_file(ode_dif, ode_tmp_dif);
        wb_supervisor_world_reload();
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
      }

      wb_receiver_next_packet(receiver);
    }
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
