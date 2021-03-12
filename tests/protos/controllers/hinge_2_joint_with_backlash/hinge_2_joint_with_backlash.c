#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 16
#define TEST_DURATION 10.0f
#define SPEED 0.2f
#define ANGLE_LIMIT 0.523599f

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag rotational_motor = wb_robot_get_device("rotational_motor");
  WbDeviceTag position_sensor = wb_robot_get_device("position_sensor");

  WbDeviceTag rotational_motor2 = wb_robot_get_device("rotational_motor2");
  WbDeviceTag position_sensor2 = wb_robot_get_device("position_sensor2");

  // Test #1: actuate only axis2
  wb_motor_set_position(rotational_motor, INFINITY);
  wb_motor_set_velocity(rotational_motor, 0);
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  wb_motor_set_position(rotational_motor2, INFINITY);
  wb_motor_set_velocity(rotational_motor2, SPEED);
  wb_position_sensor_enable(position_sensor2, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < TEST_DURATION) {
    double position = wb_position_sensor_get_value(position_sensor);
    double position2 = wb_position_sensor_get_value(position_sensor2);

    if (position > ANGLE_LIMIT)
      wb_motor_set_velocity(rotational_motor, -SPEED);
    if (position < -ANGLE_LIMIT)
      wb_motor_set_velocity(rotational_motor, SPEED);

    if (position2 > ANGLE_LIMIT)
      wb_motor_set_velocity(rotational_motor2, -SPEED);
    if (position2 < -ANGLE_LIMIT)
      wb_motor_set_velocity(rotational_motor2, SPEED);
  }

  wb_motor_set_velocity(rotational_motor, 0);
  wb_motor_set_velocity(rotational_motor2, 0);

  wb_robot_step(TIME_STEP);  // reset
  wb_robot_step(TIME_STEP);  // reset

  wb_motor_set_velocity(rotational_motor, SPEED);
  wb_motor_set_velocity(rotational_motor2, 0);

  // Test #2: actuate only axis2
  while (wb_robot_step(TIME_STEP) != -1.0 && wb_robot_get_time() < 2 * TEST_DURATION) {
    double position = wb_position_sensor_get_value(position_sensor);
    double position2 = wb_position_sensor_get_value(position_sensor2);

    if (position > ANGLE_LIMIT)
      wb_motor_set_velocity(rotational_motor, -SPEED);
    if (position < -ANGLE_LIMIT)
      wb_motor_set_velocity(rotational_motor, SPEED);

    if (position2 > ANGLE_LIMIT)
      wb_motor_set_velocity(rotational_motor2, -SPEED);
    if (position2 < -ANGLE_LIMIT)
      wb_motor_set_velocity(rotational_motor2, SPEED);
  }

  wb_robot_cleanup();

  return 0;
}
