#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>

#include <stdio.h>


#define TIME_STEP 32

enum State { WAITING, GRASPING, ROTATING, RELEASING, ROTATING_BACK }; 


int main(int argc, char **argv) {
  wb_robot_init();
  int counter = 0, i = 0;
  int state = WAITING;
  
  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");
  WbDeviceTag ur_motor_0 = wb_robot_get_device("shoulder_lift_joint");
  WbDeviceTag ur_motor_1 = wb_robot_get_device("wrist_1_joint");
  
  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);
  
  WbDeviceTag position_sensor = wb_robot_get_device("shoulder_lift_joint_sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {

    if (counter <= 0) {
      switch(state) {
        case WAITING:
          if (wb_distance_sensor_get_value(distance_sensor) < 500) {
            state = GRASPING;
            counter = 8;
            printf("Grasping can\n");
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], 0.85);
          }
          break;
        case GRASPING:
          wb_motor_set_position(ur_motor_0, -3.14159);
          wb_motor_set_position(ur_motor_1, 3.14159);
          printf("Rotating arm\n");
          state = ROTATING;
          break;
        case ROTATING:
          if (wb_position_sensor_get_value(position_sensor) < -3.1) {
            counter = 8;
            printf("Releasing can\n");
            state = RELEASING;
            for (i = 0; i < 3; ++i)
               wb_motor_set_position(hand_motors[i], 0);
          }
          break;
        case RELEASING:
          wb_motor_set_position(ur_motor_0, 0);
          wb_motor_set_position(ur_motor_1, 0);
          printf("Rotating arm back\n");
          state = ROTATING_BACK;
          break;
        case ROTATING_BACK:
          if (wb_position_sensor_get_value(position_sensor) > -0.1) {
            state = WAITING;
            printf("Waiting can\n");
          }
          break;
      }
    }
    
    counter--;
  };

  wb_robot_cleanup();

  return 0;
}
