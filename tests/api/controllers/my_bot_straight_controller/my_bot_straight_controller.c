/*
 * Description:  Simple controller for a MyBot robot with two distance sensors
 *               that moves straight till an object is detected and then starts
 *               moving backwards.
 */

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 8

int main() {
  wb_robot_init();  // necessary to initialize webots stuff

  // get devices
  WbDeviceTag us0 = wb_robot_get_device("us0");
  WbDeviceTag us1 = wb_robot_get_device("us1");

  // enable the devices
  wb_distance_sensor_enable(us0, TIME_STEP);
  wb_distance_sensor_enable(us1, TIME_STEP);
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 5);
  wb_motor_set_velocity(right_motor, 5);

  // run simulation
  while (wb_robot_step(TIME_STEP) != -1) {
    // read distance sensors
    double d0 = wb_distance_sensor_get_value(us0);
    double d1 = wb_distance_sensor_get_value(us1);
    if (d0 < 100 || d1 < 100) {
      // in case of collision move backwards
      wb_motor_set_velocity(left_motor, -5);
      wb_motor_set_velocity(right_motor, -5);
    }
  }

  wb_robot_cleanup();

  return 0;  // never reached
}
