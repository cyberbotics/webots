#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

#define X_SPEED 0.15
#define Z_SPEED 0.1

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag motor1 = wb_robot_get_device("linear motor");
  WbDeviceTag motor2 = wb_robot_get_device("linear motor 2");

  wb_motor_set_position(motor1, INFINITY);
  wb_motor_set_position(motor2, INFINITY);

  wb_motor_set_velocity(motor1, Z_SPEED);
  wb_motor_set_velocity(motor2, X_SPEED);

  wb_robot_step(20 * TIME_STEP);

  WbNodeRef robot = wb_supervisor_node_get_from_def("DYNAMIC_ROBOT");
  WbNodeRef solid0 = wb_supervisor_node_get_from_def("DYNAMIC_SOLID_0");
  WbNodeRef solid1 = wb_supervisor_node_get_from_def("DYNAMIC_SOLID_1");
  WbNodeRef solid2 = wb_supervisor_node_get_from_def("DYNAMIC_SOLID_2");

  const double *velocity = wb_supervisor_node_get_velocity(robot);
  ts_assert_vec3_in_delta(velocity[0], velocity[1], velocity[2], 0.0, 0.0, 0.0, 0.005, "Velocity of the robot should be 0.");

  velocity = wb_supervisor_node_get_velocity(solid0);
  ts_assert_vec3_in_delta(velocity[0], velocity[1], velocity[2], 0.0, 0.0, Z_SPEED, 0.005,
                          "Velocity of the solid 0 should be equal to the speed in Z.");

  velocity = wb_supervisor_node_get_velocity(solid1);
  ts_assert_vec3_in_delta(velocity[0], velocity[1], velocity[2], X_SPEED, 0.0, Z_SPEED, 0.005,
                          "Velocity of the solid 1 should be equal to the speed in X and speed in Z.");

  const double *velocity2 = wb_supervisor_node_get_velocity(solid2);
  ts_assert_vec3_in_delta(velocity[0], velocity[1], velocity[2], velocity2[0], velocity2[1], velocity2[2], 0.001,
                          "Velocity of the solid 1 should be equivalent to the velocity of the solid 2.");

  ts_send_success();
  return EXIT_SUCCESS;
}
