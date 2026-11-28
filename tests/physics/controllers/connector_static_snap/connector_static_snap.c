#include <math.h>
#include <stdio.h>
#include <string.h>
#include <webots/connector.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 8
#define POSITION_TOLERANCE 1e-3
#define ANGLE_TOLERANCE 1e-3

static double clamp_unit(double v) {
  if (v > 1.0)
    return 1.0;
  if (v < -1.0)
    return -1.0;
  return v;
}

static double rad_to_deg(double rad) {
  return rad * 180.0 / M_PI;
}

static void assert_exact_collinear_x_axes(WbNodeRef c1, WbNodeRef c2, const char *label) {
  const double *o1 = wb_supervisor_node_get_orientation(c1);
  const double *o2 = wb_supervisor_node_get_orientation(c2);
  const double x1[3] = {o1[0], o1[3], o1[6]};
  const double x2[3] = {-o2[0], -o2[3], -o2[6]};
  const double wx = x1[1] * x2[2] - x1[2] * x2[1];
  const double wy = x1[2] * x2[0] - x1[0] * x2[2];
  const double wz = x1[0] * x2[1] - x1[1] * x2[0];
  ts_assert_vec3_equal(wx, wy, wz, 0.0, 0.0, 0.0,
                       "%s: pre-lock x-axes cross product must be exact zero to hit w.isNull(), got [%g, %g, %g].", label, wx,
                       wy, wz);
}

static void assert_snapped_alignment(WbNodeRef stat_conn, WbNodeRef dyn_conn, double expected_z_angle, const char *label) {
  const double *p_stat = wb_supervisor_node_get_position(stat_conn);
  const double *p_dyn = wb_supervisor_node_get_position(dyn_conn);
  ts_assert_vec3_in_delta(p_dyn[0], p_dyn[1], p_dyn[2], p_stat[0], p_stat[1], p_stat[2], POSITION_TOLERANCE,
                          "%s: post-lock connector origins do not match ([%lf, %lf, %lf] vs [%lf, %lf, %lf]).", label, p_dyn[0],
                          p_dyn[1], p_dyn[2], p_stat[0], p_stat[1], p_stat[2]);

  const double *o_stat = wb_supervisor_node_get_orientation(stat_conn);
  const double *o_dyn = wb_supervisor_node_get_orientation(dyn_conn);

  const double x_dot = o_stat[0] * (-o_dyn[0]) + o_stat[3] * (-o_dyn[3]) + o_stat[6] * (-o_dyn[6]);
  const double x_err = acos(clamp_unit(x_dot));
  ts_assert_double_in_delta(x_err, 0.0, ANGLE_TOLERANCE,
                            "%s: post-lock x-axes misaligned by %.6f rad (%.2f deg), expected 0.0 rad.", label, x_err,
                            rad_to_deg(x_err));

  const double z_dot = o_stat[2] * o_dyn[2] + o_stat[5] * o_dyn[5] + o_stat[8] * o_dyn[8];
  const double z_ang = acos(clamp_unit(z_dot));
  ts_assert_double_in_delta(z_ang, expected_z_angle, ANGLE_TOLERANCE,
                            "%s: post-lock z-axes angle is %.6f rad (%.2f deg), expected %.6f rad (%.2f deg).", label, z_ang,
                            rad_to_deg(z_ang), expected_z_angle, rad_to_deg(expected_z_angle));
}

int main(int argc, char **argv) {
  if (argc > 1 && strcmp(argv[1], "dynamic") == 0) {
    wb_robot_init();
    WbDeviceTag connector = wb_robot_get_device("connector");
    wb_connector_enable_presence(connector, TIME_STEP);
    wb_robot_step(TIME_STEP);
    wb_connector_lock(connector);
    while (wb_robot_step(TIME_STEP) != -1) {
    }
    wb_robot_cleanup();
    return EXIT_SUCCESS;
  }

  ts_setup(argv[0]);

  WbNodeRef stat_c1 = wb_supervisor_node_get_from_def("STATIC_CONNECTOR_1");
  WbNodeRef dyn_c1 = wb_supervisor_node_get_from_def("DYNAMIC_CONNECTOR_1");
  WbNodeRef stat_c2 = wb_supervisor_node_get_from_def("STATIC_CONNECTOR_2");
  WbNodeRef dyn_c2 = wb_supervisor_node_get_from_def("DYNAMIC_CONNECTOR_2");
  WbNodeRef stat_c3 = wb_supervisor_node_get_from_def("STATIC_CONNECTOR_3");
  WbNodeRef dyn_c3 = wb_supervisor_node_get_from_def("DYNAMIC_CONNECTOR_3");
  WbNodeRef stat_c4 = wb_supervisor_node_get_from_def("STATIC_CONNECTOR_4");
  WbNodeRef dyn_c4 = wb_supervisor_node_get_from_def("DYNAMIC_CONNECTOR_4");

  WbDeviceTag c1 = wb_robot_get_device("static_connector_1");
  WbDeviceTag c3 = wb_robot_get_device("static_connector_3");
  wb_connector_enable_presence(c1, TIME_STEP);
  wb_connector_enable_presence(c3, TIME_STEP);

  wb_robot_step(TIME_STEP);

  ts_assert_int_equal(wb_connector_get_presence(c1), 1, "Pair 1 connectors should be in presence before locking.");
  ts_assert_int_equal(wb_connector_get_presence(c3), 1, "Pair 3 connectors should be in presence before locking.");

  assert_exact_collinear_x_axes(stat_c3, dyn_c3, "Pair 3 (collinear static-initiates-lock)");
  assert_exact_collinear_x_axes(dyn_c4, stat_c4, "Pair 4 (collinear dynamic-initiates-lock)");

  wb_connector_lock(c1);
  wb_connector_lock(c3);

  for (int i = 0; i < 120; ++i)
    wb_robot_step(TIME_STEP);

  assert_snapped_alignment(stat_c1, dyn_c1, 0.0, "Pair 1 (tilted static-initiates-lock)");
  assert_snapped_alignment(stat_c2, dyn_c2, 0.0, "Pair 2 (tilted dynamic-initiates-lock)");
  assert_snapped_alignment(stat_c3, dyn_c3, 0.0, "Pair 3 (collinear static-initiates-lock)");
  assert_snapped_alignment(stat_c4, dyn_c4, M_PI / 2.0, "Pair 4 (collinear dynamic-initiates-lock)");

  ts_send_success();
  return EXIT_SUCCESS;
}
