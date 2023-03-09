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

#include "RosAutomobile.hpp"
#include "webots_ros/Float64Stamped.h"

extern "C" {
int wb_robot_init();
}

using namespace std;

RosAutomobile::RosAutomobile() : Ros() {
}

RosAutomobile::~RosAutomobile() {
  mEnableIndicatorAutoDisabling.shutdown();
  mEnableLimitedSlipDifferential.shutdown();
  mGetAntiFogLightServer.shutdown();
  mGetBackwardsLightServer.shutdown();
  mGetBrakeLightsServer.shutdown();
  mGetControlModeServer.shutdown();
  mGetCruisingSpeedServer.shutdown();
  mGetDimensionsServer.shutdown();
  mGetDippedBeamServer.shutdown();
  mGetEngineTypeServer.shutdown();
  mGetGearServer.shutdown();
  mGetGearNumberServer.shutdown();
  mGetHazardFlashersServer.shutdown();
  mGetIndicatorServer.shutdown();
  mGetIndicatorPeriodServer.shutdown();
  mGetAutomobileTypeServer.shutdown();
  mGetWiperModeServer.shutdown();
  mGetWipersModeServer.shutdown();
  mSetAntiFogLightServer.shutdown();
  mSetBrakeServer.shutdown();
  mSetCruisingSpeedServer.shutdown();
  mSetDippedBeamServer.shutdown();
  mSetGearServer.shutdown();
  mSetIndicatorServer.shutdown();
  mSetIndicatorPeriodServer.shutdown();
  mSetHazardFlashersServer.shutdown();
  mSetSteeringAngleServer.shutdown();
  mSetThrottleServer.shutdown();
  mSetWiperModeServer.shutdown();
  mSetWipersModeServer.shutdown();

  mBrakePublisher.shutdown();
  mCurrentSpeedPublisher.shutdown();
  mRpmPublisher.shutdown();
  mSteeringAnglePublisher.shutdown();
  mRightSteeringAnglePublisher.shutdown();
  mLeftSteeringAnglePublisher.shutdown();
  mThrottlePublisher.shutdown();
  mWheelEncoderPublisher[0].shutdown();
  mWheelEncoderPublisher[1].shutdown();
  mWheelEncoderPublisher[2].shutdown();
  mWheelEncoderPublisher[3].shutdown();
  mWheelSpeedPublisher[0].shutdown();
  mWheelSpeedPublisher[1].shutdown();
  mWheelSpeedPublisher[2].shutdown();
  mWheelSpeedPublisher[3].shutdown();
}

void RosAutomobile::setupRobot() {
  wb_robot_init();
  mRobot = new Car();
}

void RosAutomobile::launchRos(int argc, char **argv) {
  Ros::launchRos(argc, argv);

  // add services
  mEnableIndicatorAutoDisabling = nodeHandle()->advertiseService("automobile/enable_indicator_auto_disabling",
                                                                 &RosAutomobile::enableIndicatorAutoDisablingCallback, this);
  mEnableLimitedSlipDifferential = nodeHandle()->advertiseService("automobile/enable_limited_slip_differential",
                                                                  &RosAutomobile::enableLimitedSlipDifferentialCallback, this);
  mGetAntiFogLightServer =
    nodeHandle()->advertiseService("automobile/get_antifog_light", &RosAutomobile::getAntiFogLightCallback, this);
  mGetBackwardsLightServer =
    nodeHandle()->advertiseService("automobile/get_backwards_light", &RosAutomobile::getBackwardsLightCallback, this);
  mGetBrakeLightsServer =
    nodeHandle()->advertiseService("automobile/get_brake_light", &RosAutomobile::getBrakeLightsCallback, this);
  mGetControlModeServer =
    nodeHandle()->advertiseService("automobile/get_control_mode", &RosAutomobile::getControlModeCallback, this);
  mGetCruisingSpeedServer =
    nodeHandle()->advertiseService("automobile/get_cruising_speed", &RosAutomobile::getCruisingSpeedCallback, this);
  mGetDimensionsServer =
    nodeHandle()->advertiseService("automobile/get_dimensions", &RosAutomobile::getDimensionsCallback, this);
  mGetDippedBeamServer =
    nodeHandle()->advertiseService("automobile/get_dipped_beam", &RosAutomobile::getDippedBeamCallback, this);
  mGetEngineTypeServer =
    nodeHandle()->advertiseService("automobile/get_engine_type", &RosAutomobile::getEngineTypeCallback, this);
  mGetGearServer = nodeHandle()->advertiseService("automobile/get_gear", &RosAutomobile::getGearCallback, this);
  mGetGearNumberServer =
    nodeHandle()->advertiseService("automobile/get_gear_number", &RosAutomobile::getGearNumberCallback, this);
  mGetHazardFlashersServer =
    nodeHandle()->advertiseService("automobile/get_hazard_flashers", &RosAutomobile::getHazardFlashersCallback, this);
  mGetIndicatorServer = nodeHandle()->advertiseService("automobile/get_indicator", &RosAutomobile::getIndicatorCallback, this);
  mGetIndicatorPeriodServer =
    nodeHandle()->advertiseService("automobile/get_indicator_period", &RosAutomobile::getIndicatorPeriodCallback, this);
  mGetAutomobileTypeServer =
    nodeHandle()->advertiseService("automobile/get_type", &RosAutomobile::getAutomobileTypeCallback, this);
  mGetWiperModeServer = nodeHandle()->advertiseService("automobile/get_wiper_mode", &RosAutomobile::getWiperModeCallback, this);
  mGetWipersModeServer =
    nodeHandle()->advertiseService("automobile/get_wipers_mode", &RosAutomobile::getWiperModeCallback, this);
  mSetAntiFogLightServer =
    nodeHandle()->advertiseService("automobile/set_antifog_light", &RosAutomobile::setAntiFogLightCallback, this);
  mSetBrakeServer = nodeHandle()->advertiseService("automobile/set_brake_intensity", &RosAutomobile::setBrakeCallback, this);
  mSetCruisingSpeedServer =
    nodeHandle()->advertiseService("automobile/set_cruising_speed", &RosAutomobile::setCruisingSpeedCallback, this);
  mSetDippedBeamServer =
    nodeHandle()->advertiseService("automobile/set_dipped_beam", &RosAutomobile::setDippedBeamCallback, this);
  mSetGearServer = nodeHandle()->advertiseService("automobile/set_gear", &RosAutomobile::setGearCallback, this);
  mSetIndicatorServer = nodeHandle()->advertiseService("automobile/set_indicator", &RosAutomobile::setIndicatorCallback, this);
  mSetIndicatorPeriodServer =
    nodeHandle()->advertiseService("automobile/set_indicator_period", &RosAutomobile::setIndicatorPeriodCallback, this);
  mSetHazardFlashersServer =
    nodeHandle()->advertiseService("automobile/set_hazard_flashers", &RosAutomobile::setHazardFlashersCallback, this);
  mSetSteeringAngleServer =
    nodeHandle()->advertiseService("automobile/set_steering_angle", &RosAutomobile::setSteeringAngleCallback, this);
  mSetThrottleServer = nodeHandle()->advertiseService("automobile/set_throttle", &RosAutomobile::setThrottleCallback, this);
  mSetWiperModeServer = nodeHandle()->advertiseService("automobile/set_wiper_mode", &RosAutomobile::setWiperModeCallback, this);
  mSetWipersModeServer =
    nodeHandle()->advertiseService("automobile/set_wipers_mode", &RosAutomobile::setWiperModeCallback, this);

  // add topics
  mBrakePublisher = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/brake_intensity", 1);
  mCurrentSpeedPublisher = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/current_speed", 1);
  mRpmPublisher = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/rpm", 1);
  mSteeringAnglePublisher = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/steering_angle", 1);
  mRightSteeringAnglePublisher = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/right_steering_angle", 1);
  mLeftSteeringAnglePublisher = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/left_steering_angle", 1);
  mThrottlePublisher = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/throttle", 1);
  mWheelEncoderPublisher[0] = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/front_right_wheel_encoder", 1);
  mWheelEncoderPublisher[1] = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/front_left_wheel_encoder", 1);
  mWheelEncoderPublisher[2] = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/rear_right_wheel_encoder", 1);
  mWheelEncoderPublisher[3] = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/rear_left_wheel_encoder", 1);
  mWheelSpeedPublisher[0] = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/front_right_wheel_speed", 1);
  mWheelSpeedPublisher[1] = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/front_left_wheel_speed", 1);
  mWheelSpeedPublisher[2] = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/rear_right_wheel_speed", 1);
  mWheelSpeedPublisher[3] = nodeHandle()->advertise<webots_ros::Float64Stamped>("automobile/rear_left_wheel_speed", 1);
}

void RosAutomobile::setRosDevices(const char **hiddenDevices, int numberHiddenDevices) {
  const char *automobileDevices[21] = {
    "left_steer",        "right_steer",        "left_front_wheel", "right_front_wheel", "left_rear_wheel", "right_rear_wheel",
    "left_front_sensor", "right_front_sensor", "left_rear_sensor", "right_rear_sensor", "antifog_lights",  "backwards_lights",
    "brake_lights",      "front_lights",       "left_front_brake", "left_indicators",   "left_rear_brake", "rear_lights",
    "right_front_brake", "right_indicators",   "right_rear_brake"};
  Ros::setRosDevices(automobileDevices, 21);
}

bool RosAutomobile::enableIndicatorAutoDisablingCallback(webots_ros::set_bool::Request &req,
                                                         webots_ros::set_bool::Response &res) {
  car()->enableIndicatorAutoDisabling(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::enableLimitedSlipDifferentialCallback(webots_ros::set_bool::Request &req,
                                                          webots_ros::set_bool::Response &res) {
  car()->enableLimitedSlipDifferential(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::getAntiFogLightCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  res.value = car()->getAntifogLights();
  return true;
}

bool RosAutomobile::getBackwardsLightCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  res.value = car()->getBackwardsLights();
  return true;
}

bool RosAutomobile::getBrakeLightsCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  res.value = car()->getBrakeLights();
  return true;
}

bool RosAutomobile::getControlModeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  res.value = car()->getControlMode();
  return true;
}

bool RosAutomobile::getCruisingSpeedCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  res.value = car()->getTargetCruisingSpeed();
  return true;
}

bool RosAutomobile::getDimensionsCallback(webots_ros::automobile_get_dimensions::Request &req,
                                          webots_ros::automobile_get_dimensions::Response &res) {
  res.trackFront = car()->getTrackFront();
  res.trackRear = car()->getTrackRear();
  res.wheelBase = car()->getWheelbase();
  res.frontWheelRadius = car()->getFrontWheelRadius();
  res.rearWheelRadius = car()->getRearWheelRadius();
  return true;
}

bool RosAutomobile::getDippedBeamCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  res.value = car()->getDippedBeams();
  return true;
}

bool RosAutomobile::getEngineTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  res.value = car()->getEngineType();
  return true;
}

bool RosAutomobile::getGearCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  res.value = car()->getGear();
  return true;
}

bool RosAutomobile::getGearNumberCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  res.value = car()->getGearNumber();
  return true;
}

bool RosAutomobile::getHazardFlashersCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  res.value = car()->getHazardFlashers();
  return true;
}

bool RosAutomobile::getIndicatorCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  res.value = car()->getIndicator();
  return true;
}

bool RosAutomobile::getIndicatorPeriodCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  res.value = car()->getIndicatorPeriod();
  return true;
}

bool RosAutomobile::getAutomobileTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  res.value = car()->getType();
  return true;
}

bool RosAutomobile::getWiperModeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  res.value = car()->getWiperMode();
  return true;
}

bool RosAutomobile::setAntiFogLightCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res) {
  car()->setAntifogLights(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::setBrakeCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  car()->setBrakeIntensity(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::setCruisingSpeedCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  car()->setCruisingSpeed(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::setDippedBeamCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res) {
  car()->setDippedBeams(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::setGearCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  car()->setGear(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::setIndicatorCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  car()->setIndicator(Driver::IndicatorState(req.value));
  res.success = true;
  return true;
}

bool RosAutomobile::setIndicatorPeriodCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  car()->setIndicatorPeriod(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::setHazardFlashersCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res) {
  car()->setHazardFlashers(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::setSteeringAngleCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  car()->setSteeringAngle(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::setThrottleCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  car()->setThrottle(req.value);
  res.success = true;
  return true;
}

bool RosAutomobile::setWiperModeCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  car()->setWiperMode(Driver::WiperMode(req.value));
  res.success = true;
  return true;
}

int RosAutomobile::step(int duration) {
  // publish topics
  webots_ros::Float64Stamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = name() + "/automobile";

  value.data = car()->getBrakeIntensity();
  mBrakePublisher.publish(value);

  value.data = car()->getCurrentSpeed();
  mCurrentSpeedPublisher.publish(value);

  if (car()->getControlMode() == Driver::TORQUE) {
    value.data = car()->getRpm();
    mRpmPublisher.publish(value);
  }

  value.data = car()->getSteeringAngle();
  mSteeringAnglePublisher.publish(value);

  value.data = car()->getRightSteeringAngle();
  mRightSteeringAnglePublisher.publish(value);

  value.data = car()->getLeftSteeringAngle();
  mLeftSteeringAnglePublisher.publish(value);

  value.data = car()->getThrottle();
  mThrottlePublisher.publish(value);

  for (int i = 0; i < Car::WHEEL_NB; ++i) {
    value.data = car()->getWheelEncoder(Car::WheelIndex(i));
    mWheelEncoderPublisher[i].publish(value);
    value.data = car()->getWheelSpeed(Car::WheelIndex(i));
    mWheelSpeedPublisher[i].publish(value);
  }

  return car()->step();
}
