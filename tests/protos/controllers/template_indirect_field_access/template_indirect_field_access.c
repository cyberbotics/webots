#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);  // give the controller args
  WbNodeRef proto = wb_supervisor_node_get_from_def("PROTO_template_indirect_field_access");
  WbFieldRef radarCrossSection = wb_supervisor_node_get_base_node_field(proto, "radarCrossSection");
  ts_assert_double_in_delta(wb_supervisor_field_get_sf_float(radarCrossSection), 6.0, 0.0001,
                            "radarCrossSection should be 6.0");
  ts_send_success();

  return EXIT_SUCCESS;
}
