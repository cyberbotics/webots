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

  Description : webots function interface
  Authors     : Yu Xianyuan
  Email       : xy_yu_beijing@163.com
  
***************************************************************************/
#ifndef _WEBOTSINTERFACE_H_
#define _WEBOTSINTERFACE_H_

//-----------------------------------------------------------macro
#define PI           (3.141892654)
#define TIME_STEP    (2)
//-----------------------------------------------------------typedef
/*
1，陀螺仪数据定义,为了方便调试采用角度制，注意，采用角度制
2，webots IMU模块采用RPY角度制，定系旋转，矩阵左乘，即：
       Rot=RotY(yaw)*RotZ(pitch)*RotX(roll);
*/
typedef struct
{
  double roll;       //横滚，x轴
  double pitch;      //俯仰，z轴
  double yaw;        //偏航，y轴
}eulerAngleTypeDef;

//-----------------------------------------------------------extern
extern void              webots_device_init               ();
extern void              set_spring_force     (double force);
extern void              set_X_torque        (double torque);
extern void              set_Z_torque        (double torque);
extern double            get_spring_length                ();
extern double            get_X_motor_angle                ();
extern double            get_Z_motor_angle                ();
extern bool              is_foot_touching                 ();
extern int               get_keyboard                     ();
extern eulerAngleTypeDef get_IMU_Angle                    ();
#endif

