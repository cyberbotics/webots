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

#ifndef ROS_AUTOMOBILE_HPP
#define ROS_AUTOMOBILE_HPP

#include <webots/vehicle/Car.hpp>

#include <webots_ros/get_float.h>
#include <webots_ros/get_int.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/automobile_get_dimensions.h>

#include "Ros.hpp"

using namespace webots;

class RosAutomobile : public Ros {
public:
  RosAutomobile();
  virtual ~RosAutomobile();

  bool enableIndicatorAutoDisablingCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool enableLimitedSlipDifferentialCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool getAntiFogLightCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool getBackwardsLightCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool getBrakeLightsCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool getControlModeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getCruisingSpeedCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getDimensionsCallback(webots_ros::automobile_get_dimensions::Request &req,
                             webots_ros::automobile_get_dimensions::Response &res);
  bool getDippedBeamCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool getEngineTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getGearCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getGearNumberCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getHazardFlashersCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool getIndicatorCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getIndicatorPeriodCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool getAutomobileTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool getWiperModeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool setAntiFogLightCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool setBrakeCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setCruisingSpeedCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setDippedBeamCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool setGearCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);
  bool setIndicatorCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);
  bool setIndicatorPeriodCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setHazardFlashersCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool setSteeringAngleCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setThrottleCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setWiperModeCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);

protected:
  virtual void setupRobot();
  virtual void setRosDevices(const char **hiddenDevices, int numberHiddenDevices);
  virtual void launchRos(int argc, char **argv);
  virtual int step(int duration);  // the argument of this method is ignored

private:
  Car *car() { return static_cast<Car *>(mRobot); }

  ros::ServiceServer mEnableIndicatorAutoDisabling;
  ros::ServiceServer mEnableLimitedSlipDifferential;
  ros::ServiceServer mGetAntiFogLightServer;
  ros::ServiceServer mGetBackwardsLightServer;
  ros::ServiceServer mGetBrakeLightsServer;
  ros::ServiceServer mGetControlModeServer;
  ros::ServiceServer mGetCruisingSpeedServer;
  ros::ServiceServer mGetDimensionsServer;
  ros::ServiceServer mGetDippedBeamServer;
  ros::ServiceServer mGetEngineTypeServer;
  ros::ServiceServer mGetGearServer;
  ros::ServiceServer mGetGearNumberServer;
  ros::ServiceServer mGetHazardFlashersServer;
  ros::ServiceServer mGetIndicatorServer;
  ros::ServiceServer mGetIndicatorPeriodServer;
  ros::ServiceServer mGetAutomobileTypeServer;
  ros::ServiceServer mGetWiperModeServer;
  ros::ServiceServer mGetWipersModeServer;
  ros::ServiceServer mSetAntiFogLightServer;
  ros::ServiceServer mSetBrakeServer;
  ros::ServiceServer mSetCruisingSpeedServer;
  ros::ServiceServer mSetDippedBeamServer;
  ros::ServiceServer mSetGearServer;
  ros::ServiceServer mSetIndicatorServer;
  ros::ServiceServer mSetIndicatorPeriodServer;
  ros::ServiceServer mSetHazardFlashersServer;
  ros::ServiceServer mSetSteeringAngleServer;
  ros::ServiceServer mSetThrottleServer;
  ros::ServiceServer mSetWiperModeServer;
  ros::ServiceServer mSetWipersModeServer;

  ros::Publisher mBrakePublisher;
  ros::Publisher mCurrentSpeedPublisher;
  ros::Publisher mRpmPublisher;
  ros::Publisher mSteeringAnglePublisher;
  ros::Publisher mRightSteeringAnglePublisher;
  ros::Publisher mLeftSteeringAnglePublisher;
  ros::Publisher mThrottlePublisher;
  ros::Publisher mWheelEncoderPublisher[4];
  ros::Publisher mWheelSpeedPublisher[4];
};

#endif  // ROS_AUTOMOBILE_HPP
