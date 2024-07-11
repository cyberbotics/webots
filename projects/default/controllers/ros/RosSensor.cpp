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

#include "RosSensor.hpp"

RosSensor::RosSensor(std::string deviceName, Device *device, Ros *ros, bool enableDefaultServices) :
  RosDevice(device, ros, enableDefaultServices),
  mFrameIdPrefix("") {
  std::string fixedDeviceName = Ros::fixedNameString(deviceName);
  mSensorEnableServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/enable", &RosSensor::sensorEnableCallback);
  mSamplingPeriodServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/get_sampling_period", &RosSensor::samplingPeriodCallback);
  if (mRos->rosNameSpace() != "")
    mFrameIdPrefix = mRos->rosNameSpace() + "/";
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
// cppcheck-suppress constParameter
// cppcheck-suppress constParameterCallback
bool RosSensor::sensorEnableCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  res.success = enableSensor(req.value);
  return true;
}

bool RosSensor::enableSensor(int timestep) {
  if (timestep == 0) {
    for (unsigned int i = 0; i < mPublishList.size(); ++i)
      mPublishList[i].mPublisher.shutdown();
    mPublishList.clear();
    rosDisable();
    return true;
  }

  if (timestep % (mRos->stepSize()) == 0) {
    int copy = 0;
    int minPeriod = timestep;
    for (unsigned int i = 0; i < mPublishList.size(); ++i) {
      if (mPublishList[i].mSamplingPeriod < minPeriod)
        minPeriod = mPublishList[i].mSamplingPeriod;
      if (mPublishList[i].mSamplingPeriod == timestep)
        copy++;
    }
    if (copy == 0) {
      if (minPeriod == timestep)
        rosEnable(timestep);
      publisherDetails details;
      details.mPublisher = createPublisher();
      details.mSamplingPeriod = timestep;
      details.mNewPublisher = true;
      mPublishList.push_back(details);
    }
    return true;
  }

  ROS_WARN("Wrong sampling period: %d for device: %s.", timestep, deviceName().c_str());
  return false;
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
