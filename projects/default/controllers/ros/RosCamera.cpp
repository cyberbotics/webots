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

#include "RosCamera.hpp"
#include "RosMathUtils.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "webots_ros/RecognitionObject.h"

RosCamera::RosCamera(Camera *camera, Ros *ros) : RosSensor(camera->getName(), camera, ros) {
  mCamera = camera;
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mInfoServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/get_info", &RosCamera::getInfoCallback);
  mFocusInfoServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/get_focus_info", &RosCamera::getFocusInfoCallback);
  mZoomInfoServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/get_zoom_info", &RosCamera::getZoomInfoCallback);
  mImageServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/save_image", &RosCamera::saveImageCallback);
  mSetFovServer =
    RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/set_fov", &RosCamera::setFovCallback);
  mSetFocalDistanceServer = RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/set_focal_distance",
                                                           &RosCamera::setFocalDistanceCallback);
  mHasRecognitionServer = RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/has_recognition",
                                                         &RosCamera::hasRecognitionCallback);
  if (camera->hasRecognition()) {
    mRecognitionEnableServer = RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/recognition_enable",
                                                              &RosCamera::recognitionEnableCallback);
    mRecognitionSamplingPeriodServer =
      RosDevice::rosAdvertiseService((ros->name()) + '/' + fixedDeviceName + "/recognition_get_sampling_period",
                                     &RosCamera::recognitionSamplingPeriodCallback);
  }
}

RosCamera::~RosCamera() {
  mInfoServer.shutdown();
  mFocusInfoServer.shutdown();
  mZoomInfoServer.shutdown();
  mImageServer.shutdown();
  mSetFovServer.shutdown();
  mSetFocalDistanceServer.shutdown();
  mHasRecognitionServer.shutdown();
  if (mCamera->hasRecognition()) {
    mRecognitionEnableServer.shutdown();
    mRecognitionSamplingPeriodServer.shutdown();
    mRecognitionObjectsPublisher.shutdown();
  }
  cleanup();
}

// creates a publisher for camera image with
// a [4 x ImageWidth x ImageHeight] {unsigned char} array
ros::Publisher RosCamera::createPublisher() {
  sensor_msgs::Image type;
  type.height = mCamera->getHeight();
  type.width = mCamera->getWidth();
  type.encoding = sensor_msgs::image_encodings::BGRA8;
  // type.is_bigendian = 0;
  type.step = sizeof(char) * 4 * mCamera->getWidth();

  mColorTopic = mRos->name() + '/' + RosDevice::fixedDeviceName() + "/image";
  return RosDevice::rosAdvertiseTopic(mColorTopic, type);
}

// get image from the Camera and publish it
void RosCamera::publishValue(ros::Publisher publisher) {
  const unsigned char *colorImage = mCamera->getImage();
  sensor_msgs::Image image;
  image.header.stamp = ros::Time::now();
  image.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();
  image.height = mCamera->getHeight();
  image.width = mCamera->getWidth();
  image.encoding = sensor_msgs::image_encodings::BGRA8;
  // image.is_bigendian = 0;
  image.step = sizeof(char) * 4 * mCamera->getWidth();

  image.data.resize(4 * mCamera->getWidth() * mCamera->getHeight());
  memcpy(&image.data[0], colorImage, sizeof(char) * 4 * mCamera->getWidth() * mCamera->getHeight());

  publisher.publish(image);

  if (mCamera->hasRecognition() && mCamera->getRecognitionSamplingPeriod() > 0) {
    const CameraRecognitionObject *objects = mCamera->getRecognitionObjects();
    webots_ros::RecognitionObject object;
    object.header.stamp = ros::Time::now();
    object.header.frame_id = mRos->name() + '/' + RosDevice::fixedDeviceName();
    for (int i = 0; i < mCamera->getRecognitionNumberOfObjects(); ++i) {
      object.position.x = objects[i].position[0];
      object.position.y = objects[i].position[1];
      object.position.z = objects[i].position[2];
      RosMathUtils::axisAngleToQuaternion(objects[i].orientation, object.orientation);
      object.position_on_image.x = objects[i].position_on_image[0];
      object.position_on_image.y = objects[i].position_on_image[1];
      object.size_on_image.x = objects[i].size_on_image[0];
      object.size_on_image.y = objects[i].size_on_image[1];
      object.number_of_colors = objects[i].number_of_colors;
      object.model = std::string(objects[i].model);
      for (int j = 0; j < object.number_of_colors; j++) {
        geometry_msgs::Vector3 color;
        color.x = objects[i].colors[3 * j];
        color.y = objects[i].colors[3 * j + 1];
        color.z = objects[i].colors[3 * j + 2];
        object.colors.push_back(color);
      }
      mRecognitionObjectsPublisher.publish(object);
    }
  }
}

bool RosCamera::getInfoCallback(webots_ros::camera_get_info::Request &req, webots_ros::camera_get_info::Response &res) {
  assert(mCamera);
  res.width = mCamera->getWidth();
  res.height = mCamera->getHeight();
  res.Fov = mCamera->getFov();
  res.nearRange = mCamera->getNear();
  return true;
}

bool RosCamera::getFocusInfoCallback(webots_ros::camera_get_focus_info::Request &req,
                                     webots_ros::camera_get_focus_info::Response &res) {
  assert(mCamera);
  res.focalLength = mCamera->getFocalLength();
  res.focalDistance = mCamera->getFocalDistance();
  res.maxFocalDistance = mCamera->getMaxFocalDistance();
  res.minFocalDistance = mCamera->getMinFocalDistance();
  return true;
}

bool RosCamera::getZoomInfoCallback(webots_ros::camera_get_zoom_info::Request &req,
                                    webots_ros::camera_get_zoom_info::Response &res) {
  assert(mCamera);
  res.minFov = mCamera->getMinFov();
  res.maxFov = mCamera->getMaxFov();
  return true;
}

bool RosCamera::saveImageCallback(webots_ros::save_image::Request &req, webots_ros::save_image::Response &res) {
  assert(mCamera);
  res.success = 1 + mCamera->saveImage(req.filename, req.quality);
  return true;
}

bool RosCamera::setFovCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mCamera->setFov(req.value);
  res.success = true;
  return true;
}

bool RosCamera::setFocalDistanceCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mCamera->setFocalDistance(req.value);
  res.success = true;
  return true;
}

bool RosCamera::recognitionEnableCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  if (req.value == 0) {
    res.success = true;
    mCamera->recognitionDisable();
    mRecognitionObjectsPublisher.shutdown();
  } else if (req.value > 0) {
    res.success = true;
    mCamera->recognitionEnable(req.value);

    std::string deviceNameFixed = RosDevice::fixedDeviceName();
    webots_ros::RecognitionObject type;
    type.header.frame_id = deviceNameFixed;
    mRecognitionObjectsPublisher =
      RosDevice::rosAdvertiseTopic(mRos->name() + '/' + deviceNameFixed + "/recognition_objects", type);
  } else {
    ROS_WARN("Wrong sampling period: %d for device: %s.", req.value, RosDevice::fixedDeviceName().c_str());
    res.success = false;
  }
  return true;
}

bool RosCamera::recognitionSamplingPeriodCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mCamera);
  res.value = mCamera->getRecognitionSamplingPeriod();
  return true;
}

bool RosCamera::hasRecognitionCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  assert(mCamera);
  if (mCamera->hasRecognition())
    res.value = true;
  else
    res.value = false;
  return true;
}
