#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16
#define TEST_DURATION 2.5f
#define SPEED 0.2f

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // wb_robot_init();

  WbDeviceTag motorInput = wb_robot_get_device("hingeInputMotor");
  WbDeviceTag positionInput = wb_robot_get_device("hingeInputSensor");
  wb_position_sensor_enable(positionInput, TIME_STEP);

  wb_motor_set_position(motorInput, INFINITY);
  wb_motor_set_velocity(motorInput, SPEED);

  WbNodeRef pendulum_bob = wb_supervisor_node_get_from_def("PENDULUM_BOB");

  // TEST #1: by switching direction BEFORE the backlash limit is reached, the endPoint shouldn't move
  printf("Test #1: Start.\n");
  const double expected[] = {0.0, -0.4, 0.0};
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    // command
    double pos = wb_position_sensor_get_value(positionInput);

    if (pos >= 0.10)
      wb_motor_set_velocity(motorInput, -SPEED);
    if (pos <= -0.10)
      wb_motor_set_velocity(motorInput, SPEED);

    // control
    const double *values = wb_supervisor_node_get_position(pendulum_bob);
    ts_assert_vec3_in_delta(values[0], values[1], values[2], expected[0], expected[1], expected[2], 0.0000001,
                            "The pendulum moves despite being inside the backlash range."
                            " (expected = {%f, %f, %f}, received = {%f, %f, %f})",
                            expected[0], expected[1], expected[2], values[0], values[1], values[2]);
  }
  printf("Test #1: Done.\n");

  wb_motor_set_velocity(motorInput, 0.0);
  wb_supervisor_simulation_reset_physics();
  wb_motor_set_velocity(motorInput, SPEED);

  // TEST #2: by switching direction AFTER the backlash limit is reached, the endPoint shouldn't move
  printf("Test #2: Start.\n");

  double xmax = -INFINITY;
  double xmin = INFINITY;
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < 2 * TEST_DURATION) {
    // command
    double pos = wb_position_sensor_get_value(positionInput);

    if (pos >= 0.105)
      wb_motor_set_velocity(motorInput, -SPEED);
    if (pos <= -0.105)
      wb_motor_set_velocity(motorInput, SPEED);

    // control
    const double *values = wb_supervisor_node_get_position(pendulum_bob);
    if (values[0] < xmin)
      xmin = values[0];
    if (values[0] > xmax)
      xmax = values[0];
  }

  printf("Test #2: Done.\n");
  printf("%.5f %.5f\n", xmin, xmax);
  // Test that it moved ...
  double exp = 0.0421;
  double delta = fabs(xmax) - exp;
  ts_assert_double_in_delta(delta, 0.0, 0.0001, "Positive swing overshoot or undershoot. (expected = %.10f, value = %.10f", exp,
                            fabs(xmax));

  delta = fabs(xmin) - exp;
  ts_assert_double_in_delta(delta, 0.0, 0.0001, "Negative swing overshoot or undershoot. (expected = %.10f, value = %.10f", exp,
                            fabs(xmin));

  // Test that the swings are sufficiently symmetric
  delta = fabs(xmax) - fabs(xmin);
  ts_assert_double_in_delta(delta, 0.0, 0.00005, "The swings differ by too much. (values: %.10f // %.10f)", xmin, xmax);

  wb_robot_cleanup();

  ts_send_success();
  return EXIT_SUCCESS;
}
