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

#ifndef ROS_CAMERA_HPP
#define ROS_CAMERA_HPP

#include <webots/Camera.hpp>
#include "RosSensor.hpp"

#include <webots_ros/camera_get_focus_info.h>
#include <webots_ros/camera_get_info.h>
#include <webots_ros/camera_get_zoom_info.h>
#include <webots_ros/get_bool.h>
#include <webots_ros/get_float.h>
#include <webots_ros/save_image.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>
#include "sensor_msgs/CameraInfo.h"

using namespace webots;

class RosCamera : public RosSensor {
public:
  RosCamera(Camera *camera, Ros *ros);
  virtual ~RosCamera();

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void publishAuxiliaryValue() override;
  bool getInfoCallback(webots_ros::camera_get_info::Request &req, webots_ros::camera_get_info::Response &res);
  bool getFocusInfoCallback(webots_ros::camera_get_focus_info::Request &req, webots_ros::camera_get_focus_info::Response &res);
  bool getZoomInfoCallback(webots_ros::camera_get_zoom_info::Request &req, webots_ros::camera_get_zoom_info::Response &res);
  bool saveImageCallback(webots_ros::save_image::Request &req, webots_ros::save_image::Response &res);
  bool setFovCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setFocalDistanceCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setExposureCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool getExposureCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res);
  bool recognitionEnableCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);
  bool recognitionSamplingPeriodCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res);
  bool hasRecognitionCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool hasRecognitionSegmentationCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool enableRecognitionSegmentationCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool disableRecognitionSegmentationCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool isRecognitionSegmentationEnabledCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool saveRecognitionSegmentationImageCallback(webots_ros::save_image::Request &req, webots_ros::save_image::Response &res);

  void rosEnable(int samplingPeriod) override { mCamera->enable(samplingPeriod); }
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override { return mCamera->getSamplingPeriod(); }

private:
  ros::Publisher createImagePublisher(const std::string &name);
  void cleanup() { mCamera->disable(); }
  void createCameraInfoPublisher();
  sensor_msgs::CameraInfo createCameraInfoMessage();

  bool mIsRecognitionSegmentationEnabled;

  Camera *mCamera;
  ros::Publisher mRecognitionObjectsPublisher;
  ros::Publisher mRecognitionSegmentationPublisher;
  ros::Publisher mCameraInfoPublisher;
  ros::ServiceServer mInfoServer;
  ros::ServiceServer mFocusInfoServer;
  ros::ServiceServer mZoomInfoServer;
  ros::ServiceServer mImageServer;
  ros::ServiceServer mSetFovServer;
  ros::ServiceServer mSetFocalDistanceServer;
  ros::ServiceServer mSetExposureServer;
  ros::ServiceServer mGetExposureServer;
  ros::ServiceServer mRecognitionEnableServer;
  ros::ServiceServer mRecognitionSamplingPeriodServer;
  ros::ServiceServer mHasRecognitionServer;
  ros::ServiceServer mHasRecognitionSegmentationServer;
  ros::ServiceServer mEnableRecognitionSegmentationServer;
  ros::ServiceServer mIsRecognitionSegmentationEnabledServer;
  ros::ServiceServer mDisableRecognitionSegmentationServer;
  ros::ServiceServer mSaveRecognitionSegmentationImageServer;
};

#endif  // ROS_CAMERA_HPP
