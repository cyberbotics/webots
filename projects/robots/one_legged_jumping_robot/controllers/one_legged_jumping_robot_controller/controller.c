// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/***************************************************************************

  Description : the controller file for the one-legged jumping robot
  Authors     : Yu Xianyuan
  Email       : xy_yu_beijing@163.com
  
***************************************************************************/
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <webots/keyboard.h>

#include "easyMat.h"
#include "controller.h"

robotTypeDef robot;
//----------------------------------------------------------declaration
void updateRobotStateMachine();
void forwardKinematics(matTypeDef* workPoint, jointSpaceTypeDef* jointPoint);
void update_xz_dot();
void update_last_Ts();
void update_xz_dot_desire();
/*
机器人参数初始化，在while（1）之前调用
*/
void robot_init()
{
  //------------------------------------------------------------------------参数区
  robot.spring_normal_length    = 1.2   ;     //弹簧原长
  robot.v                       = 0.20  ;     //机器人水平运动速度
  robot.r_threshold             = 0.95  ;     //用于状态机切换的腿长阈值系数
  robot.k_spring                = 2000  ;     //弹簧刚度
  robot.F_thrust                = 100.0 ;     //THRUST推力，用于调节跳跃高度
  robot.k_leg_p                 = 6     ;     //腿部控制时的kp
  robot.k_leg_v                 = 0.8   ;     //腿部控制时的kv 
  robot.k_xz_dot                = 0.08 ;     //净加速度系数
  robot.k_pose_p                = 0.8   ;     //姿态控制时的kp
  robot.k_pose_v                = 0.025 ;     //姿态控制时的kv

  //------------------------------------------------------------------------状态区
  //欧拉角，Roll,Pitch,Yaw
  robot.eulerAngle.roll         = 0;
  robot.eulerAngle.pitch        = 0;
  robot.eulerAngle.yaw          = 0;
  //欧拉角的导数，Roll,Pitch,Yaw
  robot.eulerAngle_dot.roll     = 0;
  robot.eulerAngle_dot.pitch    = 0;
  robot.eulerAngle_dot.yaw      = 0;
  //从{B}坐标系到{H}坐标系的转换
  easyMat_create(&robot.R_H_B, 3, 3);
  easyMat_eye(&robot.R_H_B);
  //从{H}坐标系到{B}坐标系的转换
  easyMat_create(&robot.R_B_H, 3, 3);
  easyMat_eye(&robot.R_B_H);             
  //关节空间
  robot.jointPoint.r                 = 0.8;
  robot.jointPoint.X_motor_angle     = 0  ;
  robot.jointPoint.Z_motor_angle     = 0  ;
  //关节空间的导数
  robot.jointPoint_dot.r             = 0  ;
  robot.jointPoint_dot.X_motor_angle = 0  ;
  robot.jointPoint_dot.Z_motor_angle = 0  ;
  //{B}坐标系工作空间
  easyMat_create(&robot.workPoint_B, 3, 1);
  easyMat_clear(&robot.workPoint_B);
  //{H}坐标系工作空间
  easyMat_create(&robot.workPoint_H, 3, 1);
  easyMat_clear(&robot.workPoint_H);
 //{B}坐标系工作空间期望值    
  easyMat_create(&robot.workPoint_B_desire, 3, 1); 
  easyMat_clear(&robot.workPoint_B_desire); 
 //{H}坐标系工作空间期望值    
  easyMat_create(&robot.workPoint_H_desire, 3, 1);  
  easyMat_clear(&robot.workPoint_H_desire);
  
  robot.is_foot_touching    = true;       //足底是否触地
  robot.Ts                  = 0;          //上一个支撑相持续时间
  robot.x_dot               = 0;          //机身x方向水平速度
  robot.z_dot               = 0;          //机身z方向水平速度
  robot.x_dot_desire        = 0;          //机身x方向期望水平速度
  robot.z_dot_desire        = 0;          //机身z方向期望水平速度
  robot.system_ms           = 0;          //从仿真启动开始的计时器
  robot.stateMachine        = THRUST;     //状态机
}
/*
机器人内存空间释放
*/
void robot_free()
{
  easyMat_free(&robot.R_H_B);
  easyMat_free(&robot.R_B_H);
  easyMat_free(&robot.workPoint_B);
  easyMat_free(&robot.workPoint_H);
  easyMat_free(&robot.workPoint_B_desire);  
  easyMat_free(&robot.workPoint_H_desire);  
}
/*
机器人状态更新，包括读取传感器的数据以及一些状态估计
*/
void updateRobotState()
{
  /*时钟更新*/
  robot.system_ms += TIME_STEP;
  /*通过键盘修改期望速度*/
  update_xz_dot_desire();
  /*足底传感器更新*/
  robot.is_foot_touching = is_foot_touching();
  /*IMU，IMU导数，以及旋转矩阵更新*/
  eulerAngleTypeDef now_IMU  = get_IMU_Angle();
  eulerAngleTypeDef now_IMU_dot;
  
  now_IMU_dot.roll           = (now_IMU.roll  - robot.eulerAngle.roll )/(0.001*TIME_STEP);
  now_IMU_dot.pitch          = (now_IMU.pitch - robot.eulerAngle.pitch)/(0.001*TIME_STEP);
  now_IMU_dot.yaw            = (now_IMU.yaw   - robot.eulerAngle.yaw  )/(0.001*TIME_STEP);
  robot.eulerAngle = now_IMU;
  
  robot.eulerAngle_dot.roll  = robot.eulerAngle_dot.roll *0.5 + now_IMU_dot.roll *0.5;
  robot.eulerAngle_dot.pitch = robot.eulerAngle_dot.pitch*0.5 + now_IMU_dot.pitch*0.5;
  robot.eulerAngle_dot.yaw   = robot.eulerAngle_dot.yaw  *0.5 + now_IMU_dot.yaw  *0.5;
  
  easyMat_RPY(&robot.R_H_B, robot.eulerAngle.roll, robot.eulerAngle.pitch, robot.eulerAngle.yaw);
  easyMat_trans(&robot.R_B_H, &robot.R_H_B);
  /*弹簧长度及其导数更新*/
  double now_r           = get_spring_length();
  double now_r_dot       = (now_r - robot.jointPoint.r)/(0.001*TIME_STEP);
  robot.jointPoint.r     = now_r;
  robot.jointPoint_dot.r = robot.jointPoint_dot.r*0.5 + now_r_dot*0.5;    //一阶低通滤波器
  /*X、Z关节角度更新*/
  double now_X_motor_angle           = get_X_motor_angle();
  double now_X_motor_angle_dot       = (now_X_motor_angle - robot.jointPoint.X_motor_angle)/(0.001*TIME_STEP);
  robot.jointPoint.X_motor_angle     = now_X_motor_angle;
  robot.jointPoint_dot.X_motor_angle = robot.jointPoint_dot.X_motor_angle *0.5 + now_X_motor_angle_dot*0.5;    //一阶低通滤波器
  
  double now_Z_motor_angle           = get_Z_motor_angle();
  double now_Z_motor_angle_dot       = (now_Z_motor_angle - robot.jointPoint.Z_motor_angle)/(0.001*TIME_STEP);
  robot.jointPoint.Z_motor_angle     = now_Z_motor_angle;
  robot.jointPoint_dot.Z_motor_angle = robot.jointPoint_dot.Z_motor_angle *0.5 + now_Z_motor_angle_dot*0.5;    //一阶低通滤波器
  /*机器人在世界坐标系下水平速度估计更新*/
  update_xz_dot();
  /*上次支撑相时间Ts更新*/
  update_last_Ts();
  /*更新状态机*/
  updateRobotStateMachine();
}
/*
机器人控制
*/
void robot_control()
{
  /*控制弹簧弹性力*/
  double dx = robot.spring_normal_length - robot.jointPoint.r;  //求压缩量
  double F_spring = dx*robot.k_spring;
  if(robot.stateMachine == THRUST)
  {
    F_spring += robot.F_thrust;
  }
  set_spring_force(F_spring);
  /*控制臀部扭矩力*/
  //LOADING和UNLOADING时候，扭矩为0
  if((robot.stateMachine == LOADING)||(robot.stateMachine == UNLOADING))
  {
    set_X_torque(0);
    set_Z_torque(0);
  }
  //COMPRESSION和THRUST时候，臀部电机控制身体姿态
  if((robot.stateMachine == COMPRESSION)||(robot.stateMachine == THRUST))
  {
    double Tx, Tz;
    Tx = -(-robot.k_pose_p*robot.eulerAngle.roll  - robot.k_pose_v*robot.eulerAngle_dot.roll) ;
    Tz = -(-robot.k_pose_p*robot.eulerAngle.pitch - robot.k_pose_v*robot.eulerAngle_dot.pitch);
    
    set_X_torque(Tx);
    set_Z_torque(Tz);
  }
  //FLIGHT的时候，控制足底移动到落足点
  if(robot.stateMachine == FLIGHT)
  {
    double Tx, Tz;
    //计算落足点
    double r   = robot.jointPoint.r;
    
    double x_f = robot.x_dot*robot.Ts/2.0 + robot.k_xz_dot*(robot.x_dot - robot.x_dot_desire);
    double z_f = robot.z_dot*robot.Ts/2.0 + robot.k_xz_dot*(robot.z_dot - robot.z_dot_desire);
    double y_f = -sqrt(r*r -x_f*x_f - z_f*z_f);

    robot.workPoint_H_desire.data[0][0] = x_f;
    robot.workPoint_H_desire.data[1][0] = y_f;
    robot.workPoint_H_desire.data[2][0] = z_f;
    //转到{B}坐标系下
    easyMat_mult(&robot.workPoint_B_desire, &robot.R_B_H, &robot.workPoint_H_desire);
    //计算期望关节角
    double x_f_B = robot.workPoint_H_desire.data[0][0];
    double y_f_B = robot.workPoint_H_desire.data[1][0];
    double z_f_B = robot.workPoint_H_desire.data[2][0];
    double x_angle_desire = atan(z_f_B/y_f_B)*180.0/PI;
    double z_angle_desire = asin(x_f_B/r)    *180.0/PI;

    //控制关节角
    double x_angle     = robot.jointPoint.X_motor_angle;
    double z_angle     = robot.jointPoint.Z_motor_angle;
    double x_angle_dot = robot.jointPoint_dot.X_motor_angle;
    double z_angle_dot = robot.jointPoint_dot.Z_motor_angle;
    
    Tx = -robot.k_leg_p*(x_angle - x_angle_desire)  - robot.k_leg_v*x_angle_dot;
    Tz = -robot.k_leg_p*(z_angle - z_angle_desire)  - robot.k_leg_v*z_angle_dot;
    
    set_X_torque(Tx);
    set_Z_torque(Tz);
  }
}
/*
通过键盘修改期望速度
*/
void update_xz_dot_desire()
{
   /*读取键盘，获取速度*/
  switch(get_keyboard())
  {
    case WB_KEYBOARD_UP:
    {
      robot.x_dot_desire = robot.v;
      break;
    }
    case WB_KEYBOARD_DOWN:
    {
      robot.x_dot_desire = -robot.v;
      break;
    }
    case WB_KEYBOARD_RIGHT:
    {
      robot.z_dot_desire = robot.v;
      break;
    }
    case WB_KEYBOARD_LEFT:
    {
      robot.z_dot_desire = -robot.v;
      break;
    }
    case '+':
    {
      static int pre_pressed_time_ms = 0;
      if(robot.system_ms > pre_pressed_time_ms + 500)
      {
        robot.v += 0.05;
        
        if(robot.v > 0.5) robot.v = 0.5;
          printf("farther: %f\n", robot.v);
          
        pre_pressed_time_ms = robot.system_ms;
      }
      break;
    }
     case '-':
    {
      static int pre_pressed_time_ms = 0;
      if(robot.system_ms > pre_pressed_time_ms + 500)
      {
        robot.v -= 0.05;
        
        if(robot.v < 0.1) robot.v = 0.1;
          printf("nearer: %f\n", robot.v);
          
        pre_pressed_time_ms = robot.system_ms;
      }
      break;
    }
    default:
    {
      robot.z_dot_desire = 0;
      robot.x_dot_desire = 0;
      break;
    }
  }
}
/*
上次支撑相时间Ts更新
*/
void update_last_Ts()
{
   static bool pre_is_foot_touching = false;
   static int stance_start_ms = 0;
   if((pre_is_foot_touching == false)&&(robot.is_foot_touching == true)) //此时进入支撑相
   {
     stance_start_ms = robot.system_ms;
   }
   if((pre_is_foot_touching == true)&&(robot.is_foot_touching == false)) //此时进入摆动相
   {
     int stance_end_ms = robot.system_ms;
     robot.Ts = 0.001*(double)(stance_end_ms - stance_start_ms);
   }
   pre_is_foot_touching = robot.is_foot_touching;
}
/*
机器人在世界坐标系下水平速度估计更新
*/
void update_xz_dot()
{
    //正运动学
    forwardKinematics(&robot.workPoint_B, &robot.jointPoint);
      
    //转换到{H}坐标系下
    double pre_x = robot.workPoint_H.data[0][0];
    double pre_z = robot.workPoint_H.data[2][0];
    easyMat_mult(&robot.workPoint_H, &robot.R_H_B, &robot.workPoint_B);
    double now_x = robot.workPoint_H.data[0][0];
    double now_z = robot.workPoint_H.data[2][0];
    //求导
    double now_x_dot = -(now_x - pre_x)/(0.001*TIME_STEP);
    double now_z_dot = -(now_z - pre_z)/(0.001*TIME_STEP);

    //滤波
    static double pre_x_dot = 0;
    static double pre_z_dot = 0;
    now_x_dot = pre_x_dot*0.5 + now_x_dot*0.5;
    now_z_dot = pre_z_dot*0.5 + now_z_dot*0.5;
    pre_x_dot = now_x_dot;
    pre_z_dot = now_z_dot;
    
    if((robot.stateMachine == COMPRESSION)||(robot.stateMachine == THRUST))
    {
      robot.x_dot = now_x_dot;
      robot.z_dot = now_z_dot;
    }
}

/*
机器人状态机切换
---------------------------------------------------
|  名称     |     代码       |       触发条件      |
---------------------------------------------------
|  落地     |   LOADING      |   足底传感器触地    |
|  压缩腿   |   COMPRESSION  |   腿长小于阈值      |
|  伸长腿   |   THRUST       |   腿长导数为正      |
|  离地     |   UNLOADING    |   腿长大于阈值      |
| 飞行      |   FLIGHT       |   足底传感器离地    |
---------------------------------------------------
*/
void updateRobotStateMachine()
{
  switch(robot.stateMachine)
  {
    case LOADING:
    {
      if(robot.jointPoint.r < robot.spring_normal_length * robot.r_threshold)
        robot.stateMachine = COMPRESSION;
      break;
    }
    case COMPRESSION:
    {
      if(robot.jointPoint_dot.r > 0)
        robot.stateMachine = THRUST;
      break;
    }
    case THRUST:
    {
      if(robot.jointPoint.r > robot.spring_normal_length * robot.r_threshold)
        robot.stateMachine = UNLOADING;
      break;
    }
    case UNLOADING:
    {
      if(robot.is_foot_touching == false)
        robot.stateMachine = FLIGHT;
      break;
    }
    case FLIGHT:
    {
      if(robot.is_foot_touching == true)
      robot.stateMachine = LOADING;
        break;
    }
    default: break;
  }
}
/*
运动学正解
{B}坐标系下足底坐标 = forwardKinematics(关节角度)
*/
void forwardKinematics(matTypeDef* workPoint, jointSpaceTypeDef* jointPoint)
{
  //转换成弧度制
  double Tx = jointPoint->X_motor_angle*PI/180.0;
  double Tz = jointPoint->Z_motor_angle*PI/180.0;
  double r  = jointPoint->r;
  
  workPoint->data[0][0] =  r*sin(Tz);
  workPoint->data[1][0] = -r*cos(Tz)*cos(Tx);
  workPoint->data[2][0] = -r*cos(Tz)*sin(Tx);
}

