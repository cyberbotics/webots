// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "RosJoystick.hpp"
#include "webots_ros/Int8Stamped.h"

RosJoystick::RosJoystick(Joystick *joystick, Ros *ros) : RosSensor("joystick", NULL, ros) {
  mJoystick = joystick;
  mAxesValuePublisher = NULL;
  mPovsValuePublisher = NULL;
  mGetModelServer = RosDevice::rosAdvertiseService((ros->name()) + "/joystick/get_model", &RosJoystick::getModelCallback);
  mGetNumberOfAxesServer =
    RosDevice::rosAdvertiseService((ros->name()) + "/joystick/get_number_of_axes", &RosJoystick::getNumberOfAxesCallback);
  mGetNumberOfPovsServer =
    RosDevice::rosAdvertiseService((ros->name()) + "/joystick/get_number_of_povs", &RosJoystick::getNumberOfPovsCallback);
  mIsConnectedServer =
    RosDevice::rosAdvertiseService((ros->name()) + "/joystick/is_connected", &RosJoystick::isConnectedCallback);
  mSetConstantForceServer =
    RosDevice::rosAdvertiseService((ros->name()) + "/joystick/set_constant_force", &RosJoystick::setConstantForceCallback);
  mSetConstantForceDurationServer = RosDevice::rosAdvertiseService((ros->name()) + "/joystick/set_constant_force_duration",
                                                                   &RosJoystick::setConstantForceDurationCallback);
  mSetAutoCenteringGainServer =
    RosDevice::rosAdvertiseService((ros->name()) + "/joystick/set_auto_centering_gain", &RosJoystick::setAutoCenteringCallback);
  mSetResistanceGainServer =
    RosDevice::rosAdvertiseService((ros->name()) + "/joystick/set_resistance_gain", &RosJoystick::setResistanceGainCallback);
  mSetForceAxisServer =
    RosDevice::rosAdvertiseService((ros->name()) + "/joystick/set_force_axis", &RosJoystick::setForceAxisCallback);
}

RosJoystick::~RosJoystick() {
  mGetModelServer.shutdown();
  mGetNumberOfAxesServer.shutdown();
  mGetNumberOfPovsServer.shutdown();
  mIsConnectedServer.shutdown();
  mSetConstantForceServer.shutdown();
  mSetConstantForceDurationServer.shutdown();
  mSetAutoCenteringGainServer.shutdown();
  mSetResistanceGainServer.shutdown();
  mSetForceAxisServer.shutdown();
  for (int i = 0; i < mJoystick->getNumberOfAxes(); ++i) {
    if (mAxesValuePublisher[i])
      mAxesValuePublisher[i].shutdown();
  }
  delete mAxesValuePublisher;
  for (int i = 0; i < mJoystick->getNumberOfPovs(); ++i) {
    if (mPovsValuePublisher[i])
      mPovsValuePublisher[i].shutdown();
  }
  delete mPovsValuePublisher;
  cleanup();
}

// creates a publisher for joystick values with a webots_ros/Int8Stamped as message type
ros::Publisher RosJoystick::createPublisher() {
  webots_ros::Int8Stamped type;
  std::string topicName = mRos->name() + "/joystick/pressed_button";
  return RosDevice::rosAdvertiseTopic(topicName, type);
}

// get button from the joystick and publish it
void RosJoystick::publishValue(ros::Publisher publisher) {
  webots_ros::Int8Stamped value;
  value.header.stamp = ros::Time::now();
  value.header.frame_id = mRos->name() + "/joystick";
  int button = mJoystick->getPressedButton();
  while (button >= 0) {
    value.data = button;
    publisher.publish(value);
    button = mJoystick->getPressedButton();
  }
}

// get axes and point of views value from the joystick and publish them
void RosJoystick::publishAuxiliaryValue() {
  webots_ros::Int8Stamped value;
  value.header.stamp = ros::Time::now();
  int axesNumber = mJoystick->getNumberOfAxes();
  int povsNumber = mJoystick->getNumberOfPovs();

  // create the axes publishers if not already done
  if (!mAxesValuePublisher) {
    if (axesNumber > 0) {
      mAxesValuePublisher = new ros::Publisher[axesNumber];
      for (int i = 0; i < axesNumber; ++i) {
        std::ostringstream s;
        s << i;
        mAxesValuePublisher[i] = RosDevice::rosAdvertiseTopic(mRos->name() + "/joystick/axis" + s.str(), value);
      }
    }
  }

  // publish the axes value
  if (mAxesValuePublisher && axesNumber > 0) {
    for (int i = 0; i < axesNumber; ++i) {
      value.header.frame_id = mRos->name() + "/joystick";
      value.data = mJoystick->getAxisValue(i);
      mAxesValuePublisher[i].publish(value);
    }
  }

  // create the point of views publishers if not already done
  if (!mPovsValuePublisher) {
    if (povsNumber > 0) {
      mPovsValuePublisher = new ros::Publisher[povsNumber];
      for (int i = 0; i < povsNumber; ++i) {
        std::ostringstream s;
        s << i;
        mPovsValuePublisher[i] = RosDevice::rosAdvertiseTopic(mRos->name() + "/joystick/pov" + s.str(), value);
      }
    }
  }

  // publish the point of views value
  if (mPovsValuePublisher && povsNumber > 0) {
    for (int i = 0; i < povsNumber; ++i) {
      value.header.frame_id = mRos->name() + "/joystick";
      value.data = mJoystick->getPovValue(i);
      mPovsValuePublisher[i].publish(value);
    }
  }
}

bool RosJoystick::getModelCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res) {
  assert(mJoystick);
  res.value = mJoystick->getModel();
  return true;
}

bool RosJoystick::getNumberOfAxesCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mJoystick);
  res.value = mJoystick->getNumberOfAxes();
  return true;
}

bool RosJoystick::getNumberOfPovsCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mJoystick);
  res.value = mJoystick->getNumberOfPovs();
  return true;
}

bool RosJoystick::isConnectedCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  assert(mJoystick);
  res.value = mJoystick->isConnected();
  return true;
}

bool RosJoystick::setConstantForceCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  mJoystick->setConstantForce(req.value);
  res.success = true;
  return true;
}

bool RosJoystick::setConstantForceDurationCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mJoystick->setConstantForceDuration(req.value);
  res.success = true;
  return true;
}

bool RosJoystick::setAutoCenteringCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mJoystick->setAutoCenteringGain(req.value);
  res.success = true;
  return true;
}

bool RosJoystick::setResistanceGainCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mJoystick->setResistanceGain(req.value);
  res.success = true;
  return true;
}

bool RosJoystick::setForceAxisCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  mJoystick->setForceAxis(req.value);
  res.success = true;
  return true;
}
