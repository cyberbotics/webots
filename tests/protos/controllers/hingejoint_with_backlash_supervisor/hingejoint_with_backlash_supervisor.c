#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16
#define TEST_DURATION 5.0f

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag motorInput = wb_robot_get_device("hingeInputMotor");
  WbDeviceTag positionInput = wb_robot_get_device("hingeInputSensor");
  wb_position_sensor_enable(positionInput, TIME_STEP);

  wb_motor_set_position(motorInput, INFINITY);
  wb_motor_set_velocity(motorInput, 0.2);

  //WbNodeRef pendulum_robot = wb_supervisor_node_get_from_def("PENDULUM_ROBOT");
  WbNodeRef pendulum_bob = wb_supervisor_node_get_from_def("PENDULUM_BOB");
  
  // TEST #1: by switching direction BEFORE the backlash limit is reached, the endPoint shouldn't move
  printf("Test #1: Start.\n");
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    // command
    double pos = wb_position_sensor_get_value(positionInput);

    if (pos >= 0.10)
      wb_motor_set_velocity(motorInput, -0.2);
    if (pos <= -0.10)
      wb_motor_set_velocity(motorInput, 0.2);
   
    // control
    const double *bpos = wb_supervisor_node_get_position(pendulum_bob);
    printf("%.10f %.10f %.10f\n", bpos[0], bpos[1], bpos[2]);
  }
  
  printf("Test #1: Done.\n");

  wb_supervisor_simulation_reset();

  wb_position_sensor_enable(positionInput, TIME_STEP);
  wb_motor_set_position(motorInput, INFINITY);
  wb_motor_set_velocity(motorInput, 0.2);

  printf("%f\n", wb_robot_get_time());
  // TEST #2: by switching direction AFTER the backlash limit is reached, the endPoint shouldn't move
  printf("Test #2: Start.\n");
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    // command
    double pos = wb_position_sensor_get_value(positionInput);
  printf("%f\n", wb_robot_get_time());

    if (pos >= 0.3)
      wb_motor_set_velocity(motorInput, -0.2);
    if (pos <= -0.3)
      wb_motor_set_velocity(motorInput, 0.2);
   
    // control
    const double *bpos = wb_supervisor_node_get_position(pendulum_bob);
    printf("%.10f %.10f %.10f\n", bpos[0], bpos[1], bpos[2]);
  }
  
  printf("Test #2: Done.\n");

  wb_robot_cleanup();

  return 0;
}