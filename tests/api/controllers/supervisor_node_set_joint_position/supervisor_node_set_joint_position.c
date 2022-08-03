/*
 * Description:  Test Supervisor API: wb_supervisor_node_set_joint_position
 *
 */

#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag slider_sensor = wb_robot_get_device("slider sensor");
  WbDeviceTag hinge_sensor = wb_robot_get_device("hinge sensor");
  WbDeviceTag hinge2_sensor = wb_robot_get_device("hinge2 sensor");
  WbDeviceTag ball_joint_sensor = wb_robot_get_device("ball joint sensor");
  wb_distance_sensor_enable(slider_sensor, TIME_STEP);
  wb_distance_sensor_enable(hinge_sensor, TIME_STEP);
  wb_distance_sensor_enable(hinge2_sensor, TIME_STEP);
  wb_distance_sensor_enable(ball_joint_sensor, TIME_STEP);

  // Run some simulation steps to stabilize the simulation
  wb_robot_step(15 * TIME_STEP);

  WbNodeRef ball_joint_node = wb_supervisor_node_get_from_def("BALL_JOINT");
  wb_supervisor_node_set_joint_position(ball_joint_node, 0.6, 3);

  WbNodeRef slider_joint_node = wb_supervisor_node_get_from_def("SLIDER_JOINT");
  wb_supervisor_node_set_joint_position(slider_joint_node, 0.6, 1);

  WbNodeRef hinge_joint_node = wb_supervisor_node_get_from_def("HINGE_JOINT");
  wb_supervisor_node_set_joint_position(hinge_joint_node, 0.6, 1);
  // even if requested hinge position is 0.6, only 0.4 can be set due to joint minStop/maxStop

  WbNodeRef hinge2_joint_node = wb_supervisor_node_get_from_def("HINGE2_JOINT");
  wb_supervisor_node_set_joint_position(hinge2_joint_node, 0.6, 2);

  wb_robot_step(TIME_STEP);

  // Check joints position is applied correctly
  double slider_value = wb_distance_sensor_get_value(slider_sensor);
  ts_assert_double_in_delta(slider_value, 305.0, 2.0, "Slider joint position is not set correctly.");
  const double hinge_value = wb_distance_sensor_get_value(hinge_sensor);
  printf("value %f\n", hinge_value);
  ts_assert_double_in_delta(hinge_value, 432.0, 2.0, "Hinge joint position is not set correctly.");
  const double hinge2_value = wb_distance_sensor_get_value(hinge2_sensor);
  ts_assert_double_in_delta(hinge2_value, 150.0, 2.0, "Hinge2 joint position is not set correctly.");
  const double ball_value = wb_distance_sensor_get_value(ball_joint_sensor);
  ts_assert_double_in_delta(ball_value, 205.0, 2.0, "Ball joint position is not set correctly.");

  // Test invalid indices
  wb_supervisor_node_set_joint_position(slider_joint_node, 0, 2);
  wb_robot_step(TIME_STEP);
  slider_value = wb_distance_sensor_get_value(slider_sensor);
  ts_assert_double_in_delta(slider_value, 305.0, 50.0, "Slider joint position should not change using invalid index 2.");

  wb_supervisor_node_set_joint_position(slider_joint_node, 0, 0);
  wb_robot_step(TIME_STEP);
  slider_value = wb_distance_sensor_get_value(slider_sensor);
  ts_assert_double_in_delta(slider_value, 305.0, 50.0, "Slider joint position should not change using invalid index 0.");

  wb_supervisor_node_set_joint_position(slider_joint_node, 0, -1);
  wb_robot_step(TIME_STEP);
  slider_value = wb_distance_sensor_get_value(slider_sensor);
  ts_assert_double_in_delta(slider_value, 305.0, 50.0, "Slider joint position should not change using invalid index -1.");

  // Reset position of slider joint
  wb_supervisor_node_set_joint_position(slider_joint_node, 0, 1);
  wb_robot_step(TIME_STEP);
  slider_value = wb_distance_sensor_get_value(slider_sensor);
  ts_assert_double_in_delta(slider_value, 1000.0, 1.0, "Slider joint position is correctly reset.");

  ts_send_success();
  return EXIT_SUCCESS;
}
