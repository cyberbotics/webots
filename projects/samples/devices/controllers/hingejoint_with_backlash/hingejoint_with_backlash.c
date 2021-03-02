#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>

#define TIME_STEP 16
#define SPEED 0.2f

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag rotor = wb_robot_get_device("rotor motor");
  
  WbDeviceTag rotorSensor = wb_robot_get_device("rotor sensor");
  wb_position_sensor_enable(rotorSensor, TIME_STEP);
  
  wb_motor_set_position(rotor, INFINITY);
  wb_motor_set_velocity(rotor, SPEED);
  
 
  while (wb_robot_step(TIME_STEP) != -1){
    double pos = wb_position_sensor_get_value(rotorSensor); 
    if(pos > 1.0471)
      wb_motor_set_velocity(rotor, -SPEED);
    if(pos < -1.0471)
      wb_motor_set_velocity(rotor, SPEED);


  }
  
  wb_robot_cleanup();
  
  return 0;
}
