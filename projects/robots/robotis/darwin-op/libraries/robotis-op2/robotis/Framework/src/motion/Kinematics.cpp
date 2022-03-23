/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "Kinematics.h"


using namespace Robot;

const double Kinematics::CAMERA_DISTANCE       = 33.2; //mm
const double Kinematics::EYE_TILT_OFFSET_ANGLE = 40.0; //degree
const double Kinematics::LEG_SIDE_OFFSET       = 37.0; //mm
const double Kinematics::THIGH_LENGTH          = 93.0; //mm
const double Kinematics::CALF_LENGTH           = 93.0; //mm
const double Kinematics::ANKLE_LENGTH          = 33.5; //mm
const double Kinematics::LEG_LENGTH            = 219.5; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

const double Kinematics::CW_LIMIT_R_SHOULDER_ROLL    = -75.0; // degree
const double Kinematics::CCW_LIMIT_R_SHOULDER_ROLL   = 135.0; // degree
const double Kinematics::CW_LIMIT_L_SHOULDER_ROLL    = -135.0; // degree
const double Kinematics::CCW_LIMIT_L_SHOULDER_ROLL   = 75.0; // degree
const double Kinematics::CW_LIMIT_R_ELBOW            = -95.0; // degree
const double Kinematics::CCW_LIMIT_R_ELBOW           = 70.0; // degree
const double Kinematics::CW_LIMIT_L_ELBOW            = -70.0; // degree
const double Kinematics::CCW_LIMIT_L_ELBOW           = 95.0; // degree
const double Kinematics::CW_LIMIT_R_HIP_YAW          = -123.0; // degree
const double Kinematics::CCW_LIMIT_R_HIP_YAW         = 53.0; // degree
const double Kinematics::CW_LIMIT_L_HIP_YAW          = -53.0; // degree
const double Kinematics::CCW_LIMIT_L_HIP_YAW         = 123.0; // degree
const double Kinematics::CW_LIMIT_R_HIP_ROLL         = -45.0; // degree
const double Kinematics::CCW_LIMIT_R_HIP_ROLL        = 59.0; // degree
const double Kinematics::CW_LIMIT_L_HIP_ROLL         = -59.0; // degree
const double Kinematics::CCW_LIMIT_L_HIP_ROLL        = 45.0; // degree
const double Kinematics::CW_LIMIT_R_HIP_PITCH        = -100.0; // degree
const double Kinematics::CCW_LIMIT_R_HIP_PITCH       = 29.0; // degree
const double Kinematics::CW_LIMIT_L_HIP_PITCH        = -29.0; // degree
const double Kinematics::CCW_LIMIT_L_HIP_PITCH       = 100.0; // degree
const double Kinematics::CW_LIMIT_R_KNEE             = -6.0; // degree
const double Kinematics::CCW_LIMIT_R_KNEE            = 130.0; // degree
const double Kinematics::CW_LIMIT_L_KNEE             = -130.0; // degree
const double Kinematics::CCW_LIMIT_L_KNEE            = 6.0; // degree
const double Kinematics::CW_LIMIT_R_ANKLE_PITCH      = -72.0; // degree
const double Kinematics::CCW_LIMIT_R_ANKLE_PITCH     = 80.0; // degree
const double Kinematics::CW_LIMIT_L_ANKLE_PITCH      = -80.0; // degree
const double Kinematics::CCW_LIMIT_L_ANKLE_PITCH     = 72.0; // degree
const double Kinematics::CW_LIMIT_R_ANKLE_ROLL       = -44.0; // degree
const double Kinematics::CCW_LIMIT_R_ANKLE_ROLL      = 63.0; // degree
const double Kinematics::CW_LIMIT_L_ANKLE_ROLL       = -63.0; // degree
const double Kinematics::CCW_LIMIT_L_ANKLE_ROLL      = 44.0; // degree
const double Kinematics::CW_LIMIT_HEAD_PAN           = -90.0; // degree
const double Kinematics::CCW_LIMIT_HEAD_PAN          = 90.0; // degree
const double Kinematics::CW_LIMIT_HEAD_TILT          = -25.0; // degree
const double Kinematics::CCW_LIMIT_HEAD_TILT         = 55.0; // degree

Kinematics* Kinematics::m_UniqueInstance = new Kinematics();

Kinematics::Kinematics()
{
}

Kinematics::~Kinematics()
{
}
