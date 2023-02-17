// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "RosBrake.hpp"

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

RosBrake::RosBrake(Brake *brake, Ros *ros) : RosDevice(brake, ros) {
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mSetDampingConstantServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/set_damping_constant", &RosBrake::setDampingConstantCallback);
  mGetTypeServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_type", &RosBrake::getTypeCallback);
  mGetMotorNameServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_motor_name", &RosBrake::getMotorNameCallback);
  mGetPositionSensorNameServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/get_position_sensor_name", &RosBrake::getPositionSensorNameCallback);
  mBrake = brake;
}

RosBrake::~RosBrake() {
  mSetDampingConstantServer.shutdown();
  mGetTypeServer.shutdown();
}

bool RosBrake::setDampingConstantCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  assert(mBrake);
  mBrake->setDampingConstant(req.value);
  res.success = true;
  return true;
}

bool RosBrake::getTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mBrake);
  res.value = mBrake->getType();
  return true;
}

bool RosBrake::getMotorNameCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res) {
  Motor *motor = mBrake->getMotor();
  if (motor)
    res.value = motor->getName();
  else
    res.value = "";
  return true;
}

bool RosBrake::getPositionSensorNameCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res) {
  PositionSensor *positionSensor = mBrake->getPositionSensor();
  if (positionSensor)
    res.value = positionSensor->getName();
  else
    res.value = "";
  return true;
}
