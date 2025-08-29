#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

static void move_vehicle(const char *def_name) {
  WbNodeRef robot_node = wb_supervisor_node_get_from_def(def_name);
  WbFieldRef translation_field = wb_supervisor_node_get_field(robot_node, "translation");
  const double *current_position = wb_supervisor_field_get_sf_vec3f(translation_field);
  double position[3] = {current_position[0] - 2, current_position[1], current_position[2]};
  wb_supervisor_field_set_sf_vec3f(translation_field, position);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag motors[3];
  motors[0] = wb_robot_get_device("front motor");
  motors[1] = wb_robot_get_device("rear right motor");
  motors[2] = wb_robot_get_device("rear left motor");
  int i;
  for (i = 0; i < 3; ++i) {
    wb_motor_set_velocity(motors[i], 4.0);
    wb_motor_set_position(motors[i], 1000000);
  }

  WbDeviceTag steer_motor = wb_robot_get_device("steer motor");
  wb_motor_set_position(steer_motor, 0.234);

  wb_robot_step(5 * time_step);

  move_vehicle("VEHICLE");

  WbDeviceTag sensor = wb_robot_get_device("front wheel sensor");
  wb_distance_sensor_enable(sensor, time_step);

  wb_robot_step(20 * time_step);

  move_vehicle("CAR");  // the behavior after CAR is moved is tested in move_car.c

  int steps = 100;
  while (steps > 0 && wb_robot_step(time_step) != -1) {
    ts_assert_double_is_bigger(wb_distance_sensor_get_value(sensor), 999.0, "Front wheel joint axis or anchor is wrong.");
    --steps;
  }

  ts_assert_int_equal(steps, 0, "Simulation terminated before completing all the tests.");

  ts_send_success();
  return EXIT_SUCCESS;
}
