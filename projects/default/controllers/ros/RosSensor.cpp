// Copyright 1996-2020 Cyberbotics Ltd.
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

#include "RosSensor.hpp"

RosSensor::RosSensor(std::string deviceName, Device *device, Ros *ros, bool enableDefaultServices) :
  RosDevice(device, ros, enableDefaultServices) {
  std::string fixedDeviceName = Ros::fixedNameString(deviceName);
  mSensorEnableServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/enable", &RosSensor::sensorEnableCallback);
  mSamplingPeriodServer = RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/get_sampling_period",
                                                         &RosSensor::samplingPeriodCallback);
}

RosSensor::~RosSensor() {
  for (unsigned int i = 0; i < mPublishList.size(); ++i)
    mPublishList[i].mPublisher.shutdown();
  mPublishList.clear();
  mSensorEnableServer.shutdown();
  mSamplingPeriodServer.shutdown();
}

// create a topic for the requested sampling period if it doesn't exist yet,
// enable the sensor with the new period if needed and
// store publisher and it's details into the mPublishList vector
bool RosSensor::sensorEnableCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  if (req.value == 0) {
    res.success = true;
    for (unsigned int i = 0; i < mPublishList.size(); ++i)
      mPublishList[i].mPublisher.shutdown();
    mPublishList.clear();
    rosDisable();
  } else if (req.value % (mRos->stepSize()) == 0) {
    int copy = 0;
    int minPeriod = req.value;
    for (unsigned int i = 0; i < mPublishList.size(); ++i) {
      if (mPublishList[i].mSamplingPeriod < minPeriod)
        minPeriod = mPublishList[i].mSamplingPeriod;
      if (mPublishList[i].mSamplingPeriod == req.value)
        copy++;
    }
    if (copy == 0) {
      mPublishList.push_back(publisherDetails());
      mPublishList.back().mSamplingPeriod = req.value;
      mPublishList.back().mNewPublisher = true;
      mPublishList.back().mPublisher = createPublisher();
      if (minPeriod == req.value)
        rosEnable(req.value);
    }
    res.success = true;
  } else {
    ROS_WARN("Wrong sampling period: %d for device: %s.", req.value, deviceName().c_str());
    res.success = false;
  }
  return true;
}

bool RosSensor::samplingPeriodCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  res.value = rosSamplingPeriod();
  return true;
}

// get values from the sensors and publish it if needed for each active topic
void RosSensor::publishValues(int step) {
  for (unsigned int i = 0; i < mPublishList.size(); ++i) {
    if (step % mPublishList[i].mSamplingPeriod == 0) {
      if (mPublishList[i].mPublisher.getNumSubscribers() > 0) {
        if (mPublishList[i].mNewPublisher)
          mPublishList[i].mNewPublisher = false;
        // publish the values from the corresponding device
        publishValue(mPublishList[i].mPublisher);
      }
      publishAuxiliaryValue();
    }
  }
}
