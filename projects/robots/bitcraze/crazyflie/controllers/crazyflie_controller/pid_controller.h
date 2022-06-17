#pragma once

/* 
 *  ...........       ____  _ __
 *  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 *  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 *  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *  
 * MIT License
 * 
 * Copyright (c) 2022 Bitcraze
 * 
 * @file pid_controller.h
 * PID controller header file
 * 
 */

typedef struct MotorPower_s{

  double m1;
  double m2;
  double m3;
  double m4;
} MotorPower_t;


typedef struct ControlCommands_s{
  double roll;
  double pitch;
  double yaw;
  double altitude;
} ControlCommands_t;

typedef struct DesiredState_s{
  double roll;
  double pitch;
  double yaw_rate;
  double altitude;
  double vx;
  double vy;
} DesiredState_t;

typedef struct ActualState_s{
  double roll;
  double pitch;
  double yaw_rate;
  double altitude;
  double vx;
  double vy;
} ActualState_t;

typedef struct GainsPID_s{
  double kp_att_rp;
  double kd_att_rp;
  double kp_att_y;
  double kd_att_y;
  double kp_vel_xy;
  double kd_vel_xy;
  double kp_z;
  double kd_z;
  double ki_z;
} GainsPID_t;

float constrain(float value, const float minVal, const float maxVal);
void init_pid_attitude_fixed_height_controller();

void pid_attitude_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, MotorPower_t* motorCommands);

void pid_velocity_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, MotorPower_t* motorCommands);

void pid_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, ControlCommands_t* controlCommands);

void motor_mixing(ControlCommands_t controlCommands, MotorPower_t* motorCommands);

void pid_attitude_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, ControlCommands_t* controlCommands);

void pid_horizontal_velocity_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt);