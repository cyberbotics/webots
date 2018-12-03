/*
 * Description:  Simple controller for a MyBot robot with two distance sensors.
 */

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
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

  // run simulation
  while (wb_robot_step(TIME_STEP) != -1) {
    // read distance sensors
    double d0 = wb_distance_sensor_get_value(us0);
    double d1 = wb_distance_sensor_get_value(us1);
    if (d0 < 100 || d1 < 100)
      // in case of collision turn left
      wb_differential_wheels_set_speed(-5, 5);
    else
      // otherwise go straight
      wb_differential_wheels_set_speed(5, 5);
  }

  wb_robot_cleanup();

  return 0;  // never reached
}
