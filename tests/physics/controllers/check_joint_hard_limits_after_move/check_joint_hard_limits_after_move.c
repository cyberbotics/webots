#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define MAX_STOP 0.4
int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = wb_robot_get_basic_time_step();

  WbDeviceTag slider_motor = wb_robot_get_device("slider motor");
  WbDeviceTag slider_sensor = wb_robot_get_device("slider sensor");
  wb_position_sensor_enable(slider_sensor, time_step);
  wb_motor_set_position(slider_motor, INFINITY);
  wb_motor_set_velocity(slider_motor, 2.0);

  WbDeviceTag hinge_motor = wb_robot_get_device("hinge motor");
  WbDeviceTag hinge_sensor = wb_robot_get_device("hinge sensor");
  wb_position_sensor_enable(hinge_sensor, time_step);
  wb_motor_set_position(hinge_motor, INFINITY);
  wb_motor_set_velocity(hinge_motor, 1.0);

  WbDeviceTag hinge2_motor = wb_robot_get_device("hinge2 motor1");
  WbDeviceTag hinge2_sensor = wb_robot_get_device("hinge2 sensor1");
  wb_position_sensor_enable(hinge2_sensor, time_step);
  wb_motor_set_position(hinge2_motor, INFINITY);
  wb_motor_set_velocity(hinge2_motor, 1.0);

  wb_robot_step(60 * time_step);

  double joint_position = wb_position_sensor_get_value(slider_sensor);
  ts_assert_double_in_delta(
    joint_position, MAX_STOP, 0.01,
    "Before moving the robot the SliderJoint hard limit is not respected: position %.3f > maxStop %.3f.", joint_position,
    MAX_STOP);

  joint_position = wb_position_sensor_get_value(hinge_sensor);
  ts_assert_double_in_delta(joint_position, MAX_STOP, 0.01,
                            "Before moving the robot the HingeJoint hard limit is not respected: position %.3f > maxStop %.3f.",
                            joint_position, MAX_STOP);

  joint_position = wb_position_sensor_get_value(hinge2_sensor);
  ts_assert_double_in_delta(
    joint_position, MAX_STOP, 0.01,
    "Before moving the robot the Hinge2Joint hard limit is not respected: position %.3f > maxStop %.3f.", joint_position,
    MAX_STOP);

  // Move robot
  WbNodeRef robot = wb_supervisor_node_get_self();
  WbFieldRef translation_field = wb_supervisor_node_get_field(robot, "translation");
  const double *translation = wb_supervisor_field_get_sf_vec3f(translation_field);
  const double new_translation[3] = {translation[0], translation[1], -0.5};
  wb_supervisor_field_set_sf_vec3f(translation_field, new_translation);

  wb_robot_step(60 * time_step);

  joint_position = wb_position_sensor_get_value(slider_sensor);
  ts_assert_double_in_delta(joint_position, MAX_STOP, 0.01,
                            "After moving the robot the SliderJoint hard limit is not respected: position %.3f > maxStop %.3f.",
                            joint_position, MAX_STOP);

  joint_position = wb_position_sensor_get_value(hinge_sensor);
  ts_assert_double_in_delta(joint_position, MAX_STOP, 0.01,
                            "After moving the robot the HingeJoint hard limit is not respected: position %.3f > maxStop %.3f.",
                            joint_position, MAX_STOP);

  joint_position = wb_position_sensor_get_value(hinge2_sensor);
  ts_assert_double_in_delta(joint_position, MAX_STOP, 0.01,
                            "After moving the robot the Hinge2Joint hard limit is not respected: position %.3f > maxStop %.3f.",
                            joint_position, MAX_STOP);

  ts_send_success();
  return EXIT_SUCCESS;
}
