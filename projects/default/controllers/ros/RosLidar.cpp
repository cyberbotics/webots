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

#include "RosLidar.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/image_encodings.h"

RosLidar::RosLidar(Lidar *lidar, Ros *ros) : RosSensor(lidar->getName(), lidar, ros) {
  mLidar = lidar;
  mIsPointCloudEnabled = false;
  mLaserScanPublisher = new ros::Publisher[mLidar->getNumberOfLayers()];
  std::string deviceNameFixed = RosDevice::fixedDeviceName();
  mEnablePointCloudServer = RosDevice::rosAdvertiseService((ros->name()) + '/' + deviceNameFixed + '/' + "enable_point_cloud",
                                                           &RosLidar::enablePointCloudCallback);
  mGetFrequencyInfoServer = RosDevice::rosAdvertiseService((ros->name()) + '/' + deviceNameFixed + '/' + "get_frequency_info",
                                                           &RosLidar::getFrequencyInfoCallback);
  mGetInfoServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + deviceNameFixed + '/' + "get_info", &RosLidar::getInfoCallback);
  mIsPointCloudEnabledServer = RosDevice::rosAdvertiseService(
    (ros->name()) + '/' + deviceNameFixed + '/' + "is_point_cloud_enabled", &RosLidar::isPointCloudEnabledCallback);
  mSetFrequencyServer = RosDevice::rosAdvertiseService((ros->name()) + '/' + deviceNameFixed + '/' + "set_frequency",
                                                       &RosLidar::setFrequencyCallback);
  mGetLayerRangeImage = RosDevice::rosAdvertiseService((ros->name()) + '/' + deviceNameFixed + '/' + "get_layer_range_image",
                                                       &RosLidar::getLayerRangeImage);
  mGetLayerPointCloud = RosDevice::rosAdvertiseService((ros->name()) + '/' + deviceNameFixed + '/' + "get_layer_point_cloud",
                                                       &RosLidar::getLayerPointCloud);
}

RosLidar::~RosLidar() {
  mEnablePointCloudServer.shutdown();
  mGetFrequencyInfoServer.shutdown();
  mGetInfoServer.shutdown();
  mIsPointCloudEnabledServer.shutdown();
  mSetFrequencyServer.shutdown();
  mGetLayerRangeImage.shutdown();
  mGetLayerPointCloud.shutdown();
  for (int i = 0; i < mLidar->getNumberOfLayers(); ++i) {
    if (mLaserScanPublisher[i])
      mLaserScanPublisher[i].shutdown();
  }
  delete mLaserScanPublisher;
  cleanup();
}

// creates a publisher for lidar image with
// a [ImageWidth x ImageHeight] {float} array
ros::Publisher RosLidar::createPublisher() {
  std::string deviceNameFixed = RosDevice::fixedDeviceName();
  sensor_msgs::LaserScan LaserScaneType;
  for (int i = 0; i < mLidar->getNumberOfLayers(); ++i) {
    std::ostringstream s;
    s << i;
    mLaserScanPublisher[i] =
      RosDevice::rosAdvertiseTopic(mRos->name() + '/' + deviceNameFixed + "/laser_scan/layer" + s.str(), LaserScaneType);
  }
  sensor_msgs::Image type;
  type.height = mLidar->getNumberOfLayers();
  type.width = mLidar->getHorizontalResolution();
  type.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  type.step = sizeof(float) * mLidar->getHorizontalResolution();
  return RosDevice::rosAdvertiseTopic(mRos->name() + '/' + deviceNameFixed + "/range_image", type);
}

// get image from the Lidar and publish it
void RosLidar::publishValue(ros::Publisher publisher) {
  const char *rangeImageVector;
  rangeImageVector = (const char *)(void *)mLidar->getRangeImage();
  sensor_msgs::Image image;
  image.header.stamp = ros::Time::now();
  image.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();
  image.height = mLidar->getNumberOfLayers();
  image.width = mLidar->getHorizontalResolution();
  image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image.step = sizeof(float) * mLidar->getHorizontalResolution();

  image.data.resize(sizeof(float) * mLidar->getHorizontalResolution() * mLidar->getNumberOfLayers());
  memcpy(&image.data[0], rangeImageVector, sizeof(float) * mLidar->getHorizontalResolution() * mLidar->getNumberOfLayers());

  publisher.publish(image);
}

void RosLidar::publishAuxiliaryValue() {
  if (mIsPointCloudEnabled)
    publishPointCloud();
  for (int i = 0; i < mLidar->getNumberOfLayers(); ++i)
    publishLaserScan(i);
}

void RosLidar::publishLaserScan(int layer) {
  if (layer < mLidar->getNumberOfLayers()) {
    const float *rangeImageVector = mLidar->getLayerRangeImage(layer);
    sensor_msgs::LaserScan laserScan;
    laserScan.header.stamp = ros::Time::now();
    laserScan.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();
    laserScan.angle_min = -mLidar->getFov() / 2.0;
    laserScan.angle_max = mLidar->getFov() / 2.0;
    laserScan.angle_increment = mLidar->getFov() / mLidar->getHorizontalResolution();
    laserScan.time_increment = (double)mLidar->getSamplingPeriod() / (1000.0 * mLidar->getHorizontalResolution());
    laserScan.scan_time = (double)mLidar->getSamplingPeriod() / 1000.0;
    laserScan.range_min = mLidar->getMinRange();
    laserScan.range_max = mLidar->getMaxRange();
    for (int i = 0; i < mLidar->getHorizontalResolution(); ++i)
      laserScan.ranges.push_back(rangeImageVector[i]);
    mLaserScanPublisher[layer].publish(laserScan);
  }
}

void RosLidar::publishPointCloud() {
  const WbLidarPoint *pointCloud = mLidar->getPointCloud();
  sensor_msgs::PointCloud cloud;
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();
  sensor_msgs::ChannelFloat32 layerChannel;
  layerChannel.name = "layer";
  for (int i = 0; i < mLidar->getNumberOfPoints(); i++) {
    geometry_msgs::Point32 point;
    point.x = pointCloud[i].x;
    point.y = pointCloud[i].y;
    point.z = pointCloud[i].z;
    cloud.points.push_back(point);
    layerChannel.values.push_back(pointCloud[i].layer_id);
  }
  cloud.channels.push_back(layerChannel);
  mPointCloudPublisher.publish(cloud);
}

bool RosLidar::getInfoCallback(webots_ros::lidar_get_info::Request &req, webots_ros::lidar_get_info::Response &res) {
  assert(mLidar);
  res.horizontalResolution = mLidar->getHorizontalResolution();
  res.numberOfLayers = mLidar->getNumberOfLayers();
  res.fov = mLidar->getFov();
  res.verticalFov = mLidar->getVerticalFov();
  res.minRange = mLidar->getMinRange();
  res.maxRange = mLidar->getMaxRange();
  return true;
}

bool RosLidar::isPointCloudEnabledCallback(webots_ros::node_get_status::Request &req,
                                           webots_ros::node_get_status::Response &res) {
  assert(mLidar);
  res.status = mLidar->isPointCloudEnabled();
  return true;
}

bool RosLidar::getFrequencyInfoCallback(webots_ros::lidar_get_frequency_info::Request &req,
                                        webots_ros::lidar_get_frequency_info::Response &res) {
  assert(mLidar);
  res.frequency = mLidar->getFrequency();
  res.minFrequency = mLidar->getMinFrequency();
  res.maxFrequency = mLidar->getMaxFrequency();
  return true;
}

bool RosLidar::setFrequencyCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mLidar->setFrequency(req.value);
  res.success = true;
  return true;
}

// cppcheck-suppress constParameter
// cppcheck-suppress constParameterCallback
bool RosLidar::enablePointCloudCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res) {
  if (!mIsPointCloudEnabled && req.value) {
    mIsPointCloudEnabled = true;

    std::string deviceNameFixed = RosDevice::fixedDeviceName();
    sensor_msgs::PointCloud type;
    type.header.frame_id = deviceNameFixed;

    mPointCloudPublisher = RosDevice::rosAdvertiseTopic(mRos->name() + '/' + deviceNameFixed + "/point_cloud", type);
    mLidar->enablePointCloud();
  } else if (mIsPointCloudEnabled && !req.value) {
    mIsPointCloudEnabled = false;
    mPointCloudPublisher.shutdown();
    mLidar->disablePointCloud();
  }
  res.success = true;
  return true;
}

bool RosLidar::getLayerRangeImage(webots_ros::lidar_get_layer_range_image::Request &req,
                                  webots_ros::lidar_get_layer_range_image::Response &res) {
  const char *rangeImageVector = (const char *)(void *)mLidar->getLayerRangeImage(req.layer);
  res.image.header.stamp = ros::Time::now();
  res.image.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();
  res.image.height = 1;
  res.image.width = mLidar->getHorizontalResolution();
  res.image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  res.image.step = sizeof(float) * mLidar->getHorizontalResolution();

  res.image.data.resize(sizeof(float) * mLidar->getHorizontalResolution());
  memcpy(&res.image.data[0], rangeImageVector, sizeof(float) * mLidar->getHorizontalResolution());

  return true;
}

bool RosLidar::getLayerPointCloud(webots_ros::lidar_get_layer_point_cloud::Request &req,
                                  webots_ros::lidar_get_layer_point_cloud::Response &res) {
  assert(mLidar);
  if (!mIsPointCloudEnabled)
    return false;

  const WbLidarPoint *pointCloud = mLidar->getLayerPointCloud(req.layer);
  for (int i = 0; i < mLidar->getHorizontalResolution(); i++) {
    geometry_msgs::Point32 point;
    point.x = pointCloud[i].x;
    point.y = pointCloud[i].y;
    point.z = pointCloud[i].z;
    res.pointCloud.points.push_back(point);
  }
  return true;
}
