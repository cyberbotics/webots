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

#include "RosRangeFinder.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

RosRangeFinder::RosRangeFinder(RangeFinder *range_finder, Ros *ros) : RosSensor(range_finder->getName(), range_finder, ros) {
  mRangeFinder = range_finder;
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mInfoServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_info", &RosRangeFinder::getInfoCallback);
  mImageServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/save_image", &RosRangeFinder::saveImageCallback);
}

RosRangeFinder::~RosRangeFinder() {
  mInfoServer.shutdown();
  mImageServer.shutdown();
  cleanup();
}

// creates a publisher for range_finder image with
// a [ImageWidth x ImageHeight] {float} array
ros::Publisher RosRangeFinder::createPublisher() {
  createCameraInfoPublisher();
  sensor_msgs::Image type;
  type.height = mRangeFinder->getHeight();
  type.width = mRangeFinder->getWidth();
  type.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  type.step = sizeof(float) * mRangeFinder->getWidth();

  mRangeTopic = RosDevice::fixedDeviceName() + "/range_image";
  return RosDevice::rosAdvertiseTopic(mRangeTopic, type);
}

void RosRangeFinder::createCameraInfoPublisher() {
  sensor_msgs::CameraInfo type;
  mCameraInfoPublisher = RosDevice::rosAdvertiseTopic(RosDevice::fixedDeviceName() + "/camera_info", type);
}

// get image from the RangeFinder and publish it
void RosRangeFinder::publishValue(ros::Publisher publisher) {
  const char *rangeImageVector;
  rangeImageVector = static_cast<const char *>(static_cast<void *>(const_cast<float *>(mRangeFinder->getRangeImage())));
  sensor_msgs::Image image;
  image.header.stamp = ros::Time::now();
  image.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  image.height = mRangeFinder->getHeight();
  image.width = mRangeFinder->getWidth();
  image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image.step = sizeof(float) * mRangeFinder->getWidth();

  image.data.resize(sizeof(float) * mRangeFinder->getWidth() * mRangeFinder->getHeight());
  memcpy(&image.data[0], rangeImageVector, sizeof(float) * mRangeFinder->getWidth() * mRangeFinder->getHeight());

  publisher.publish(image);
}

sensor_msgs::CameraInfo RosRangeFinder::createCameraInfoMessage() {
  sensor_msgs::CameraInfo info;
  info.header.stamp = ros::Time::now();
  info.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();

  const double width = mRangeFinder->getWidth();
  const double height = mRangeFinder->getHeight();
  info.width = width;
  info.height = height;

  const double horizontalFov = mRangeFinder->getFov();
  const double focalLength = width / (2.0 * tan(horizontalFov / 2.0));

  const double fx = focalLength;
  const double fy = focalLength;
  const double cx = (width + 1.0) / 2.0;
  const double cy = (height + 1.0) / 2.0;

  const boost::array<double, 9> K = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info.K = K;

  const boost::array<double, 9> R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.R = R;

  const boost::array<double, 12> P = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  info.P = P;

  return info;
}

void RosRangeFinder::publishAuxiliaryValue() {
  if (mCameraInfoPublisher.getNumSubscribers() > 0)
    mCameraInfoPublisher.publish(createCameraInfoMessage());
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
