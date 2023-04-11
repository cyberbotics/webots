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

#include "RosCamera.hpp"
#include "RosMathUtils.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "webots_ros/RecognitionObject.h"
#include "webots_ros/RecognitionObjects.h"

RosCamera::RosCamera(Camera *camera, Ros *ros) : RosSensor(camera->getName(), camera, ros) {
  mIsRecognitionSegmentationEnabled = false;

  mCamera = camera;
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mInfoServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_info", &RosCamera::getInfoCallback);
  mFocusInfoServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_focus_info", &RosCamera::getFocusInfoCallback);
  mZoomInfoServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_zoom_info", &RosCamera::getZoomInfoCallback);
  mImageServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/save_image", &RosCamera::saveImageCallback);
  mSetFovServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_fov", &RosCamera::setFovCallback);
  mSetFocalDistanceServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/set_focal_distance", &RosCamera::setFocalDistanceCallback);
  mSetExposureServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_exposure", &RosCamera::setExposureCallback);
  mGetExposureServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_exposure", &RosCamera::getExposureCallback);
  mHasRecognitionServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/has_recognition", &RosCamera::hasRecognitionCallback);

  mHasRecognitionSegmentationServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/recognition_has_segmentation",
                                                                     &RosCamera::hasRecognitionSegmentationCallback);
  if (camera->hasRecognition()) {
    mRecognitionEnableServer =
      RosDevice::rosAdvertiseService(fixedDeviceName + "/recognition_enable", &RosCamera::recognitionEnableCallback);
    mRecognitionSamplingPeriodServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/recognition_get_sampling_period",
                                                                      &RosCamera::recognitionSamplingPeriodCallback);
    mEnableRecognitionSegmentationServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/recognition_enable_segmentation",
                                                                          &RosCamera::enableRecognitionSegmentationCallback);
    mDisableRecognitionSegmentationServer = RosDevice::rosAdvertiseService(
      fixedDeviceName + "/recognition_disable_segmentation", &RosCamera::disableRecognitionSegmentationCallback);
    mIsRecognitionSegmentationEnabledServer = RosDevice::rosAdvertiseService(
      fixedDeviceName + "/recognition_is_segmentation_enabled", &RosCamera::isRecognitionSegmentationEnabledCallback);
    mSaveRecognitionSegmentationImageServer = RosDevice::rosAdvertiseService(
      fixedDeviceName + "/recognition_save_segmentation_image", &RosCamera::saveRecognitionSegmentationImageCallback);
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
  mSetExposureServer.shutdown();
  mGetExposureServer.shutdown();
  if (mCamera->hasRecognition()) {
    mRecognitionEnableServer.shutdown();
    mRecognitionSamplingPeriodServer.shutdown();
    mRecognitionObjectsPublisher.shutdown();
    mRecognitionSegmentationPublisher.shutdown();
  }
  cleanup();
}

// creates a publisher for camera image with
// a [4 x ImageWidth x ImageHeight] {unsigned char} array
ros::Publisher RosCamera::createPublisher() {
  createCameraInfoPublisher();
  return createImagePublisher("image");
}

void RosCamera::createCameraInfoPublisher() {
  sensor_msgs::CameraInfo type;
  mCameraInfoPublisher = RosDevice::rosAdvertiseTopic(RosDevice::fixedDeviceName() + "/camera_info", type);
}

ros::Publisher RosCamera::createImagePublisher(const std::string &name) {
  sensor_msgs::Image type;
  type.height = mCamera->getHeight();
  type.width = mCamera->getWidth();
  type.encoding = sensor_msgs::image_encodings::BGRA8;
  // type.is_bigendian = 0;
  type.step = sizeof(char) * 4 * mCamera->getWidth();
  return RosDevice::rosAdvertiseTopic(RosDevice::fixedDeviceName() + "/" + name, type);
}

// get image from the Camera and publish it
void RosCamera::publishValue(ros::Publisher publisher) {
  const unsigned char *colorImage = mCamera->getImage();
  sensor_msgs::Image image;
  image.header.stamp = ros::Time::now();
  image.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();
  image.height = mCamera->getHeight();
  image.width = mCamera->getWidth();
  image.encoding = sensor_msgs::image_encodings::BGRA8;
  // image.is_bigendian = 0;
  image.step = sizeof(char) * 4 * mCamera->getWidth();

  image.data.resize(4 * mCamera->getWidth() * mCamera->getHeight());
  memcpy(&image.data[0], colorImage, sizeof(char) * 4 * mCamera->getWidth() * mCamera->getHeight());

  publisher.publish(image);
}

sensor_msgs::CameraInfo RosCamera::createCameraInfoMessage() {
  sensor_msgs::CameraInfo info;
  info.header.stamp = ros::Time::now();
  info.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName();

  const double width = mCamera->getWidth();
  const double height = mCamera->getHeight();
  info.width = width;
  info.height = height;

  const double horizontalFov = mCamera->getFov();
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

void RosCamera::publishAuxiliaryValue() {
  if (mCameraInfoPublisher.getNumSubscribers() > 0)
    mCameraInfoPublisher.publish(createCameraInfoMessage());

  if (mCamera->hasRecognition() && mCamera->getRecognitionSamplingPeriod() > 0) {
    const CameraRecognitionObject *cameraObjects = mCamera->getRecognitionObjects();
    webots_ros::RecognitionObjects objects;
    objects.header.stamp = ros::Time::now();
    objects.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName() + "/recognition_objects";
    for (int i = 0; i < mCamera->getRecognitionNumberOfObjects(); ++i) {
      webots_ros::RecognitionObject object;
      object.position.x = cameraObjects[i].position[0];
      object.position.y = cameraObjects[i].position[1];
      object.position.z = cameraObjects[i].position[2];
      RosMathUtils::axisAngleToQuaternion(cameraObjects[i].orientation, object.orientation);
      object.position_on_image.x = cameraObjects[i].position_on_image[0];
      object.position_on_image.y = cameraObjects[i].position_on_image[1];
      object.size_on_image.x = cameraObjects[i].size_on_image[0];
      object.size_on_image.y = cameraObjects[i].size_on_image[1];
      object.number_of_colors = cameraObjects[i].number_of_colors;
      object.model = std::string(cameraObjects[i].model);
      for (int j = 0; j < object.number_of_colors; j++) {
        geometry_msgs::Vector3 color;
        color.x = cameraObjects[i].colors[3 * j];
        color.y = cameraObjects[i].colors[3 * j + 1];
        color.z = cameraObjects[i].colors[3 * j + 2];
        object.colors.push_back(color);
      }
      objects.objects.push_back(object);
    }
    mRecognitionObjectsPublisher.publish(objects);

    if (mIsRecognitionSegmentationEnabled) {
      const unsigned char *colorImage = mCamera->getRecognitionSegmentationImage();
      sensor_msgs::Image image;
      image.header.stamp = ros::Time::now();
      image.header.frame_id = mFrameIdPrefix + RosDevice::fixedDeviceName() + "/recognition_segmentation_image";
      image.height = mCamera->getHeight();
      image.width = mCamera->getWidth();
      image.encoding = sensor_msgs::image_encodings::BGRA8;
      // image.is_bigendian = 0;
      image.step = sizeof(char) * 4 * mCamera->getWidth();
      image.data.resize(4 * mCamera->getWidth() * mCamera->getHeight());
      memcpy(&image.data[0], colorImage, sizeof(char) * 4 * mCamera->getWidth() * mCamera->getHeight());
      mRecognitionSegmentationPublisher.publish(image);
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

bool RosCamera::setExposureCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  assert(mCamera);
  mCamera->setExposure(req.value);
  res.success = true;
  return true;
}

bool RosCamera::getExposureCallback(webots_ros::get_float::Request &req, webots_ros::get_float::Response &res) {
  assert(mCamera);
  res.value = mCamera->getExposure();
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
    webots_ros::RecognitionObjects type;
    type.header.frame_id = deviceNameFixed;
    mRecognitionObjectsPublisher = RosDevice::rosAdvertiseTopic(deviceNameFixed + "/recognition_objects", type);
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
  res.value = mCamera->hasRecognition();
  return true;
}

bool RosCamera::hasRecognitionSegmentationCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  assert(mCamera);
  res.value = mCamera->hasRecognitionSegmentation();
  return true;
}

bool RosCamera::enableRecognitionSegmentationCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  assert(mCamera);
  if (mCamera->hasRecognition()) {
    mCamera->enableRecognitionSegmentation();
    if (!mRecognitionSegmentationPublisher && mCamera->hasRecognitionSegmentation() &&
        mCamera->getRecognitionSamplingPeriod() > 0) {
      mRecognitionSegmentationPublisher = createImagePublisher("recognition_segmentation_image");
      mIsRecognitionSegmentationEnabled = true;
    }
    res.value = true;
  } else
    res.value = false;
  return true;
}

bool RosCamera::disableRecognitionSegmentationCallback(webots_ros::get_bool::Request &req,
                                                       webots_ros::get_bool::Response &res) {
  assert(mCamera);
  if (mCamera->hasRecognition()) {
    mCamera->disableRecognitionSegmentation();
    mRecognitionSegmentationPublisher.shutdown();
    mIsRecognitionSegmentationEnabled = false;
    res.value = true;
  } else
    res.value = false;
  return true;
}

bool RosCamera::isRecognitionSegmentationEnabledCallback(webots_ros::get_bool::Request &req,
                                                         webots_ros::get_bool::Response &res) {
  assert(mCamera);
  res.value = mCamera->isRecognitionSegmentationEnabled();
  return true;
}

bool RosCamera::saveRecognitionSegmentationImageCallback(webots_ros::save_image::Request &req,
                                                         webots_ros::save_image::Response &res) {
  assert(mCamera);
  res.success = 1 + mCamera->saveRecognitionSegmentationImage(req.filename, req.quality);
  return true;
}
