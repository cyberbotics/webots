/*
 Ned_Controller controller in C.

 Webots controller for the Niryo Ned robot. 
 With this controller, you can see the 6 different axis of the robot moving
 You can also control the robots with your keyboard
*/


// Import the Webots simulator API and the Keyboard libray.
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <stdlib.h> 

#define TIME_STEP 32


static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

// function to create a timer
static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}


//function to show the differents commands
static void command_manager() {
  printf("------------COMMANDS--------------\n");
  printf("Launch demo --> D\n");
  printf("Move_Joint1 --> A and Z\n");
  printf("Move_Joint2 --> Q and S\n");
  printf("Move_Joint3 --> W and X\n");
  printf("Move_Joint4 --> Y and U\n");
  printf("Move_Joint5 --> H and J\n");
  printf("Move_Joint6 --> B and N\n");
  printf("Open/Close Gripper --> L or M\n");
  printf("Launch Pick and Place --> P\n");
  printf("----------------------------------\n");
}


int main(int argc, char **argv) {
   
  wb_robot_init(); //init the robot
  command_manager(); //show commands
  
  // Init of the Keyboard control
  int time_step = (int)wb_robot_get_basic_time_step();
  wb_keyboard_enable(time_step);
  
  
  //----------------------Init all the motors of the Ned---------------------
  WbDeviceTag motors[8];
  motors[1] = wb_robot_get_device("joint_1");
  motors[2] = wb_robot_get_device("joint_2");
  motors[3] = wb_robot_get_device("joint_3");
  motors[4] = wb_robot_get_device("joint_4");
  motors[5] = wb_robot_get_device("joint_5");
  motors[6] = wb_robot_get_device("joint_6");
  motors[7] = wb_robot_get_device("joint_base_to_mors_1");
  motors[8] = wb_robot_get_device("joint_base_to_mors_2");
  
  
  // Set the motor velocity
  // First we make sure that every joints are at their initial positions
  wb_motor_set_position(motors[1], 0.0);
  wb_motor_set_position(motors[2], 0.0);
  wb_motor_set_position(motors[3], 0.0);
  wb_motor_set_position(motors[4], 0.0);
  wb_motor_set_position(motors[5], 0.0);
  wb_motor_set_position(motors[6], 0.0);
  wb_motor_set_position(motors[7], 0.0);
  wb_motor_set_position(motors[8], 0.0);
  
  
  // Set the motors speed. Here we put 1 or 2 radian/second
  wb_motor_set_velocity(motors[1], 1.0);
  wb_motor_set_velocity(motors[2], 1.0);
  wb_motor_set_velocity(motors[3], 1.0);
  wb_motor_set_velocity(motors[4], 1.0);
  wb_motor_set_velocity(motors[5], 1.0);
  wb_motor_set_velocity(motors[6], 1.0);
  wb_motor_set_velocity(motors[7], 1.0);
  wb_motor_set_velocity(motors[8], 1.0);
  //--------------------------------------------------------------------------------
  
  //control with keyboard
  while (wb_robot_step(time_step) != -1) {
  
    int c = wb_keyboard_get_key(); //get the keyboard key
    switch (c) {
        case 65:
          wb_motor_set_position(motors[1], -1.5);
          printf("Move --> Joint1");
          break;
          
        case 90:
          wb_motor_set_position(motors[1], 1.5);
          printf("Move --> Joint1");
          break;
          
        case 81:
          printf("Move --> Joint2");
          wb_motor_set_position(motors[2], -0.5);
          break;
          
        case 83:
          printf("Move --> Joint2");
          wb_motor_set_position(motors[2], 0.5);
          break;
          
        case 87:
          printf("Move --> Joint3");
          wb_motor_set_position(motors[3], -0.5);
          break;
          
        case 88:
          printf("Move --> Joint3");
          wb_motor_set_position(motors[3], 0.5);
          break;
          
        case 89:
          printf("Move --> Joint4");
          wb_motor_set_position(motors[4], -1);
          break;
          
        case 85:
          printf("Move --> Joint4");
          wb_motor_set_position(motors[4], 1);
          break;
          
        case 72:
          printf("Move --> Joint5");
          wb_motor_set_position(motors[5], -1.5);
          break;
          
        case 74:
          printf("Move --> Joint5");
          wb_motor_set_position(motors[5], 1.5);
          break;
          
        case 66:
          printf("Move --> Joint6");
          wb_motor_set_position(motors[6], 1.5);
          break;
          
        case 78:
          printf("Move --> Joint6");
          wb_motor_set_position(motors[6], -1.5);
          break;
          
        case 76:
          printf("Open Gripper");
          wb_motor_set_position(motors[7], 0.01);
          wb_motor_set_position(motors[8], 0.01);
          break;
          
        case 77:
          printf("Close Gripper");
          wb_motor_set_position(motors[7], 0.0);
          wb_motor_set_position(motors[8], 0.0);
          break;
          
        case 68: //demo
        
          wb_motor_set_velocity(motors[1], 1.0);
          wb_motor_set_velocity(motors[2], 1.0);
          wb_motor_set_velocity(motors[3], 1.0);
  
          wb_motor_set_position(motors[1], 1.5);
          wb_motor_set_position(motors[7], 0.01);
          wb_motor_set_position(motors[8], 0.01);
          passive_wait(1.5);
          wb_motor_set_position(motors[1], 0.0);
          passive_wait(1.5);
          wb_motor_set_position(motors[2], 0.5);
          passive_wait(0.7);
          wb_motor_set_position(motors[2], 0.0);
          passive_wait(0.7);
          wb_motor_set_position(motors[1], -0.5);
          wb_motor_set_position(motors[4], 1.45);
          passive_wait(1.5);
          wb_motor_set_position(motors[4], 0.0);
          passive_wait(1.5);
          wb_motor_set_position(motors[5], -1.0);
          passive_wait(0.7);
          wb_motor_set_position(motors[5], 0.0);
          passive_wait(1.0);
          wb_motor_set_position(motors[3], 0.0);
          wb_motor_set_position(motors[1], 0.0);
          passive_wait(0.5);
          wb_motor_set_position(motors[6], 1.5);
          passive_wait(1.0);
          wb_motor_set_position(motors[6], 0);
          passive_wait(1.0);
          wb_motor_set_position(motors[7], 0);
          wb_motor_set_position(motors[8], 0);
          passive_wait(0.5);
          wb_motor_set_position(motors[7], 0.01);
          wb_motor_set_position(motors[8], 0.01);
          break;
          
        case 80: //pick and place
        
        
          wb_motor_set_velocity(motors[1], 0.5);
          wb_motor_set_velocity(motors[2], 0.5);
          wb_motor_set_velocity(motors[3], 0.5);
          
          
          wb_motor_set_position(motors[1], 1.5);
          wb_motor_set_position(motors[2], 0.70);
          wb_motor_set_position(motors[7], 0.01);
          wb_motor_set_position(motors[8], 0.01);
          passive_wait(4.2);
          wb_motor_set_position(motors[3], 0.5);
          passive_wait(1.2);
          wb_motor_set_position(motors[7], 0.0);
          wb_motor_set_position(motors[8], 0.0);
          passive_wait(1.5);
          wb_motor_set_position(motors[3], 0.3);
          passive_wait(1.2);
          wb_motor_set_position(motors[1], 0.0);
          passive_wait(5.0);
          wb_motor_set_position(motors[3], 0.5);
          passive_wait(0.5);
          wb_motor_set_position(motors[7], 0.01);
          wb_motor_set_position(motors[8], 0.01);
          passive_wait(0.5);
          wb_motor_set_position(motors[2], 0.0);
          wb_motor_set_position(motors[3], 0.0);
          break;
      }
  }


  wb_robot_cleanup();
  return 0;
}
    
  









