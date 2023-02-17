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

#ifndef ROS_LIDAR_HPP
#define ROS_LIDAR_HPP

#include <webots/Lidar.hpp>
#include "RosSensor.hpp"

#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>

#include <webots_ros/lidar_get_frequency_info.h>
#include <webots_ros/lidar_get_info.h>
#include <webots_ros/lidar_get_layer_point_cloud.h>
#include <webots_ros/lidar_get_layer_range_image.h>
#include <webots_ros/node_get_status.h>

using namespace webots;

class RosLidar : public RosSensor {
public:
  RosLidar(Lidar *lidar, Ros *ros);
  virtual ~RosLidar();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void publishAuxiliaryValue() override;
  bool enablePointCloudCallback(webots_ros::set_bool::Request &req, webots_ros::set_bool::Response &res);
  bool getFrequencyInfoCallback(webots_ros::lidar_get_frequency_info::Request &req,
                                webots_ros::lidar_get_frequency_info::Response &res);
  bool getInfoCallback(webots_ros::lidar_get_info::Request &req, webots_ros::lidar_get_info::Response &res);
  bool isPointCloudEnabledCallback(webots_ros::node_get_status::Request &req, webots_ros::node_get_status::Response &res);
  bool setFrequencyCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool getLayerRangeImage(webots_ros::lidar_get_layer_range_image::Request &req,
                          webots_ros::lidar_get_layer_range_image::Response &res);
  bool getLayerPointCloud(webots_ros::lidar_get_layer_point_cloud::Request &req,
                          webots_ros::lidar_get_layer_point_cloud::Response &res);

  void rosEnable(int samplingPeriod) override { mLidar->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mLidar->getSamplingPeriod(); }

private:
  void publishPointCloud();
  void publishLaserScan();
  RosLidar(const RosLidar &);             // non constructor-copyable
  RosLidar &operator=(const RosLidar &);  // non copyable
  void cleanup() { mLidar->disable(); }

  Lidar *mLidar;
  bool mIsPointCloudEnabled;
  ros::Publisher mPointCloudPublisher;
  ros::Publisher mLaserScanPublisher;

  ros::ServiceServer mEnablePointCloudServer;
  ros::ServiceServer mGetFrequencyInfoServer;
  ros::ServiceServer mGetInfoServer;
  ros::ServiceServer mIsPointCloudEnabledServer;
  ros::ServiceServer mSetFrequencyServer;
  ros::ServiceServer mGetLayerRangeImage;
  ros::ServiceServer mGetLayerPointCloud;
};

#endif  // ROS_LIDAR_HPP
