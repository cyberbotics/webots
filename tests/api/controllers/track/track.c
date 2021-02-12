#include <stdio.h>
#include <webots/brake.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

void test_track_brake() {
  int i = 0;
  const double *position;

  WbDeviceTag motor = wb_robot_get_device("motor");
  WbDeviceTag brake = wb_robot_get_device("brake");

  WbNodeRef robotNode = wb_supervisor_node_get_from_def("ROBOT2");
  position = wb_supervisor_node_get_position(robotNode);
  ts_assert_vec3_equal(position[0], position[1], position[2], 0.0, 0.07, 0.5, "Initial position of robot 2 is wrong.");

  for (i = 0; i < 4; i++)
    wb_robot_step(TIME_STEP);

  wb_motor_set_force(motor, 20);

  for (i = 0; i < 40; i++)
    wb_robot_step(TIME_STEP);

  position = wb_supervisor_node_get_position(robotNode);
  ts_assert_double_in_delta(position[0], 0.031310, 0.00001, "Robot 2 position X after moving straight is wrong.");

  for (i = 0; i < 40; i++)
    wb_robot_step(TIME_STEP);

  position = wb_supervisor_node_get_position(robotNode);
  ts_assert_double_in_delta(position[0], 0.062818, 0.00001, "Robot 2 position X after moving straight is wrong.");

  wb_brake_set_damping_constant(brake, 500);
  for (i = 0; i < 40; i++)
    wb_robot_step(TIME_STEP);

  position = wb_supervisor_node_get_position(robotNode);
  ts_assert_double_in_delta(position[0], 0.082369, 0.00001, "Robot 2 position X after activating the brake is wrong.");
}

void test_tracked_robot() {
  const double *position, *orientation;
  double offset, previousValue, leftSensorValue, rightSensorValue;

  WbDeviceTag leftMotor = wb_robot_get_device("left motor");
  WbDeviceTag rightMotor = wb_robot_get_device("right motor");
  WbDeviceTag leftSensor = wb_robot_get_device("left position sensor");
  WbDeviceTag rightSensor = wb_robot_get_device("right position sensor");

  wb_position_sensor_enable(leftSensor, TIME_STEP);
  wb_position_sensor_enable(rightSensor, TIME_STEP);

  WbNodeRef robotNode = wb_supervisor_node_get_from_def("ROBOT1");
  WbFieldRef rotationField = wb_supervisor_node_get_field(robotNode, "rotation");
  position = wb_supervisor_node_get_position(robotNode);
  ts_assert_vec3_equal(position[0], position[1], position[2], 0.0, 0.06, 0.0, "Initial position of tracked robot is wrong.");

  int i = 0;
  double p = 0.0;
  wb_robot_step(TIME_STEP);
  for (; i < 20; i++) {
    p += 0.01;
    wb_motor_set_position(rightMotor, p);
    wb_motor_set_position(leftMotor, p);

    wb_robot_step(TIME_STEP);
  }

  leftSensorValue = wb_position_sensor_get_value(leftSensor);
  rightSensorValue = wb_position_sensor_get_value(rightSensor);
  ts_assert_double_in_delta(leftSensorValue, 0.174772, 0.00001,
                            "Left motor position of tracked robotis wrong after moving straight.");
  ts_assert_double_in_delta(rightSensorValue, leftSensorValue, 0.00001,
                            "Left and right motor position of tracked robot doesn't match after moving straight.");

  position = wb_supervisor_node_get_position(robotNode);
  ts_assert_double_in_delta(position[0], 0.172570, 0.0001, "Tracked robot position X after moving straight is wrong.");
  ts_assert_double_in_delta(position[2], 0.0000, 0.0001, "Tracked robot position Z after moving straight is wrong.");

  orientation = wb_supervisor_field_get_sf_rotation(rotationField);
  ts_assert_double_in_delta(orientation[3], 0.0, 0.001, "Tracked robot position Z after moving straight is wrong.");

  wb_motor_set_position(rightMotor, INFINITY);
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_velocity(rightMotor, -0.2);
  wb_motor_set_velocity(leftMotor, 0.2);

  for (i = 0; i < 100; i++) {
    wb_robot_step(TIME_STEP);
  }

  previousValue = leftSensorValue;
  leftSensorValue = wb_position_sensor_get_value(leftSensor);
  ts_assert_double_in_delta(leftSensorValue, previousValue + 0.2 * TIME_STEP / 1000 * 100, 0.00001,
                            "Left motor position of tracked robot is wrong after rotating.");

  offset = leftSensorValue - previousValue;
  previousValue = rightSensorValue;
  rightSensorValue = wb_position_sensor_get_value(rightSensor);
  ts_assert_double_in_delta(rightSensorValue, previousValue - offset, 0.00001,
                            "Left and right motor position offset of tracked robot doesn't match after rotating.");

  orientation = wb_supervisor_field_get_sf_rotation(rotationField);
  ts_assert_vec3_in_delta(orientation[0], orientation[1], orientation[2], 0.0, -1.0, 0.0, 0.002,
                          "Tracked robot rotation axis is wrong after rotating.");
  ts_assert_double_in_delta(orientation[3], 1.097467, 0.0001, "Tracked robot rotation angle is wrong after rotating.");

  wb_motor_set_velocity(rightMotor, 0.0);
  wb_motor_set_velocity(leftMotor, 0.0);
}

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  if (strcmp(wb_robot_get_name(), "tracked robot") == 0)
    test_tracked_robot();
  else
    test_track_brake();

  ts_send_success();
  return EXIT_SUCCESS;
}
