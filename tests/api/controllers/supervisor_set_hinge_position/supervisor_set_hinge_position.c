#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/touch_sensor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  WbNodeRef s1_node = wb_supervisor_node_get_from_def("S1");
  WbNodeRef s2_node = wb_supervisor_node_get_from_def("S2");
  WbNodeRef ts1_node = wb_supervisor_node_get_from_def("TS1");
  WbNodeRef ts2_node = wb_supervisor_node_get_from_def("TS2");

  WbFieldRef s1_tr_field = wb_supervisor_node_get_field(s1_node, "translation");
  WbFieldRef s2_tr_field = wb_supervisor_node_get_field(s2_node, "translation");
  WbFieldRef ts1_tr_field = wb_supervisor_node_get_field(ts1_node, "translation");
  WbFieldRef ts2_tr_field = wb_supervisor_node_get_field(ts2_node, "translation");

  ts_assert_pointer_not_null(s1_tr_field, "Cannot retrieve S1");
  ts_assert_pointer_not_null(s2_tr_field, "Cannot retrieve S2");
  ts_assert_pointer_not_null(ts1_tr_field, "Cannot retrieve TS1");
  ts_assert_pointer_not_null(ts2_tr_field, "Cannot retrieve TS2");

  const double s1_tr[3] = {0.0, 0.15, 0.0};
  wb_supervisor_field_set_sf_vec3f(s1_tr_field, s1_tr);

  const double s2_tr[3] = {0.0, 0.15, 0.0};
  wb_supervisor_field_set_sf_vec3f(s2_tr_field, s2_tr);

  const double ts1_tr[3] = {0.0, 0.15, -0.25};
  wb_supervisor_field_set_sf_vec3f(ts1_tr_field, ts1_tr);

  const double ts2_tr[3] = {0.0, 0.3, -0.25};
  wb_supervisor_field_set_sf_vec3f(ts2_tr_field, ts2_tr);

  WbDeviceTag m1 = wb_robot_get_device("m1");
  WbDeviceTag m2 = wb_robot_get_device("m2");
  WbDeviceTag ts1 = wb_robot_get_device("ts1");
  WbDeviceTag ts2 = wb_robot_get_device("ts2");

  wb_touch_sensor_enable(ts1, TIME_STEP);
  wb_touch_sensor_enable(ts2, TIME_STEP);

  wb_robot_step(TIME_STEP);

  wb_motor_set_position(m1, -0.1);
  wb_motor_set_position(m2, -0.1);

  while (wb_robot_get_time() < 1.0)
    wb_robot_step(TIME_STEP);

  ts_assert_boolean_equal(wb_touch_sensor_get_value(ts1), "Touch sensor ts1 not fired");

  ts_assert_boolean_equal(wb_touch_sensor_get_value(ts2), "Touch sensor ts2 not fired");

  ts_send_success();
  return EXIT_SUCCESS;
}
