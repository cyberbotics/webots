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

#ifndef ROS_DEVICE_HPP
#define ROS_DEVICE_HPP

#include <webots/Device.hpp>

#include <webots_ros/get_int.h>
#include <webots_ros/get_string.h>

#include "Ros.hpp"

using namespace webots;

class RosDevice {
public:
  virtual ~RosDevice();
  void init();

  bool getModelCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool getNameCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool getNodeTypeCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);

  virtual std::string deviceName();
  virtual Device *device() { return mDevice; }

protected:
  RosDevice(Device *device, Ros *ros, bool enableDefaultServices = true);

  std::string fixedDeviceName();

  template<class T, class MReq, class MRes>
  ros::ServiceServer rosAdvertiseService(std::string ServiceName, bool (T::*callback)(MReq, MRes));

  template<class M> ros::Publisher rosAdvertiseTopic(std::string topicName, M mType);

  template<class T, class M> ros::Subscriber rosSubscribeTopic(std::string topicName, void (T::*callback)(M));

  Ros *mRos;

private:
  ros::ServiceServer mGetModel;
  ros::ServiceServer mGetName;
  ros::ServiceServer mGetNodeType;

  Device *mDevice;
  bool mEnableDefaultServices;
};

// template definition depending on the callback method's class
template<class T, class MReq, class MRes>
ros::ServiceServer RosDevice::rosAdvertiseService(std::string ServiceName, bool (T::*callback)(MReq, MRes)) {
  return (mRos->nodeHandle())->advertiseService(ServiceName, callback, static_cast<T *>(this));
}

// template definition depending on the message type
template<class M> ros::Publisher RosDevice::rosAdvertiseTopic(std::string topicName, M mType) {
  return (mRos->nodeHandle())->advertise<M>(topicName, 1);
}

// template definition depending on the callback method's class
template<class T, class M> ros::Subscriber RosDevice::rosSubscribeTopic(std::string topicName, void (T::*callback)(M)) {
  return (mRos->nodeHandle())->subscribe(topicName, 10, callback, static_cast<T *>(this));
}

#endif  // ROS_DEVICE_HPP
