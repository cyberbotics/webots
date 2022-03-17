// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "RosRangeFinder.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

RosRangeFinder::RosRangeFinder(RangeFinder *range_finder, Ros *ros) : RosSensor(range_finder->getName(), range_finder, ros) {
  mRangeFinder = range_finder;
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mInfoServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/get_info", &RosRangeFinder::getInfoCallback);
  mImageServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/save_image", &RosRangeFinder::saveImageCallback);
}

RosRangeFinder::~RosRangeFinder() {
  mInfoServer.shutdown();
  mImageServer.shutdown();
  cleanup();
}

// creates a publisher for range_finder image with
// a [ImageWidth x ImageHeight] {float} array
ros::Publisher RosRangeFinder::createPublisher() {
  sensor_msgs::Image type;
  type.height = mRangeFinder->getHeight();
  type.width = mRangeFinder->getWidth();
  type.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  type.step = sizeof(float) * mRangeFinder->getWidth();

  mRangeTopic = mRos->name() + '/' + RosDevice::fixedDeviceName() + "/range_image";
  return RosDevice::rosAdvertiseTopic(mRangeTopic, type);
}

// get image from the RangeFinder and publish it
void RosRangeFinder::publishValue(ros::Publisher publisher) {
  const char *rangeImageVector;
  rangeImageVector = (const char *)(void *)mRangeFinder->getRangeImage();
  sensor_msgs::Image image;
  image.header.stamp = ros::Time::now();
  image.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();
  image.height = mRangeFinder->getHeight();
  image.width = mRangeFinder->getWidth();
  image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image.step = sizeof(float) * mRangeFinder->getWidth();

  image.data.resize(sizeof(float) * mRangeFinder->getWidth() * mRangeFinder->getHeight());
  memcpy(&image.data[0], rangeImageVector, sizeof(float) * mRangeFinder->getWidth() * mRangeFinder->getHeight());

  publisher.publish(image);
}

bool RosRangeFinder::getInfoCallback(webots_ros::range_finder_get_info::Request &req,
                                     webots_ros::range_finder_get_info::Response &res) {
  assert(mRangeFinder);
  res.width = mRangeFinder->getWidth();
  res.height = mRangeFinder->getHeight();
  res.Fov = mRangeFinder->getFov();
  res.minRange = mRangeFinder->getMinRange();
  res.maxRange = mRangeFinder->getMaxRange();
  return true;
}

bool RosRangeFinder::saveImageCallback(webots_ros::save_image::Request &req, webots_ros::save_image::Response &res) {
  assert(mRangeFinder);
  res.success = 1 + mRangeFinder->saveImage(req.filename, req.quality);
  return true;
}
