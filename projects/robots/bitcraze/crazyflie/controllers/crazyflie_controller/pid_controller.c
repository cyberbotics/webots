/** 
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
 * @file pid_controller.c
 * A simple PID controller for attitude control of an 
 * quadcopter 
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pid_controller.h"

float constrain(float value, const float minVal, const float maxVal)
{
  return fminf(maxVal, fmaxf(minVal,value));
}


double pastAltitudeError, pastPitchError, pastRollError, pastYawRateError;
double pastVxError, pastVyError;

void init_pid_attitude_fixed_height_controller()
{
    pastAltitudeError = 0;
    pastYawRateError =0;
    pastPitchError = 0;
    pastRollError = 0;
    pastVxError = 0;
    pastVyError = 0;
}

void pid_attitude_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, MotorPower_t* motorCommands)
{
    ControlCommands_t controlCommands = {0};
    pid_fixed_height_controller(actualState, 
    desiredState, gainsPID, dt, &controlCommands);
    pid_attitude_controller(actualState, 
    desiredState, gainsPID, dt, &controlCommands);
    motor_mixing(controlCommands, motorCommands);
}


void pid_velocity_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, MotorPower_t* motorCommands)
{
    ControlCommands_t controlCommands = {0};
    pid_horizontal_velocity_controller(actualState, 
    desiredState, gainsPID, dt);
    pid_fixed_height_controller(actualState, 
    desiredState, gainsPID, dt, &controlCommands);
    pid_attitude_controller(actualState, 
    desiredState, gainsPID, dt, &controlCommands);
    motor_mixing(controlCommands, motorCommands);
}

void pid_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, ControlCommands_t* controlCommands)
{
    double altitudeError = desiredState->altitude - actualState.altitude;
    double altitudeDerivativeError = (altitudeError - pastAltitudeError)/dt;
    controlCommands->altitude = gainsPID.kp_z * constrain(altitudeError, -1, 1) + gainsPID.kd_z*altitudeDerivativeError + gainsPID.ki_z;
    pastAltitudeError = altitudeError;

}

void motor_mixing(ControlCommands_t controlCommands, MotorPower_t* motorCommands)
{
    // Motor mixing
    motorCommands->m1 =  controlCommands.altitude - controlCommands.roll + controlCommands.pitch + controlCommands.yaw;
    motorCommands->m2 =  controlCommands.altitude - controlCommands.roll - controlCommands.pitch - controlCommands.yaw;
    motorCommands->m3 =  controlCommands.altitude + controlCommands.roll - controlCommands.pitch + controlCommands.yaw;
    motorCommands->m4 =  controlCommands.altitude + controlCommands.roll + controlCommands.pitch - controlCommands.yaw;
}


void pid_attitude_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, ControlCommands_t* controlCommands)
{

    // Calculate errors
    double pitchError = desiredState->pitch - actualState.pitch;
    double pitchDerivativeError = (pitchError - pastPitchError)/dt;
    double rollError = desiredState->roll - actualState.roll;
    double rollDerivativeError = (rollError - pastRollError)/dt;
    double yawRateError = desiredState->yaw_rate -actualState.yaw_rate;

    //PID control
    controlCommands->roll = gainsPID.kp_att_rp * constrain(rollError,-1, 1) + gainsPID.kd_att_rp*rollDerivativeError;
    controlCommands->pitch = -gainsPID.kp_att_rp * constrain(pitchError,-1, 1) - gainsPID.kd_att_rp*pitchDerivativeError;
    controlCommands->yaw = gainsPID.kp_att_y * constrain(yawRateError, -1, 1);
    
    // Save error for the next round
    pastPitchError= pitchError;
    pastRollError= rollError;
    pastYawRateError = yawRateError;

}


void pid_horizontal_velocity_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt)
{

    double vxError = desiredState->vx - actualState.vx;
    double vxDerivative = (vxError - pastVxError)/dt;
    double vyError = desiredState->vy - actualState.vy;
    double vyDerivative = (vyError - pastVyError)/dt;

    //PID control
    double pitchCommand = gainsPID.kp_vel_xy * constrain(vxError,-1, 1) + gainsPID.kd_vel_xy*vxDerivative;
    double rollCommand = - gainsPID.kp_vel_xy * constrain(vyError,-1, 1) - gainsPID.kd_vel_xy*vyDerivative;
    
    desiredState->pitch = pitchCommand;
    desiredState->roll = rollCommand;

    // Save error for the next round
    pastVxError = vxError;
    pastVyError = vyError;

}