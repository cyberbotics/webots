/*
 *   MotionStatus.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "MotionStatus.h"

using namespace Robot;

JointData MotionStatus::m_CurrentJoints;
int MotionStatus::FB_GYRO(0);
int MotionStatus::RL_GYRO(0);
int MotionStatus::FB_ACCEL(0);
int MotionStatus::RL_ACCEL(0);

int MotionStatus::BUTTON(0);
int MotionStatus::FALLEN(0);
