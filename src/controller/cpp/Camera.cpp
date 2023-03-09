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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/camera.h>
#include <webots/Camera.hpp>

using namespace std;
using namespace webots;

void Camera::enable(int sampling_period) {
  wb_camera_enable(getTag(), sampling_period);
}

void Camera::disable() {
  wb_camera_disable(getTag());
}

int Camera::getSamplingPeriod() const {
  return wb_camera_get_sampling_period(getTag());
}

const unsigned char *Camera::getImage() const {
  return wb_camera_get_image(getTag());
}

unsigned char Camera::imageGetRed(const unsigned char *image, int width, int x, int y) {
  return wb_camera_image_get_red(image, width, x, y);
}

unsigned char Camera::imageGetGreen(const unsigned char *image, int width, int x, int y) {
  return wb_camera_image_get_green(image, width, x, y);
}

unsigned char Camera::imageGetBlue(const unsigned char *image, int width, int x, int y) {
  return wb_camera_image_get_blue(image, width, x, y);
}

unsigned char Camera::imageGetGray(const unsigned char *image, int width, int x, int y) {
  return wb_camera_image_get_gray(image, width, x, y);
}

unsigned char Camera::imageGetGrey(const unsigned char *image, int width, int x, int y) {
  return wb_camera_image_get_grey(image, width, x, y);
}

int Camera::getWidth() const {
  return wb_camera_get_width(getTag());
}

int Camera::getHeight() const {
  return wb_camera_get_height(getTag());
}

double Camera::getFov() const {
  return wb_camera_get_fov(getTag());
}

double Camera::getMaxFov() const {
  return wb_camera_get_max_fov(getTag());
}

double Camera::getMinFov() const {
  return wb_camera_get_min_fov(getTag());
}

void Camera::setFov(double fov) {
  wb_camera_set_fov(getTag(), fov);
}

double Camera::getExposure() const {
  return wb_camera_get_exposure(getTag());
}

void Camera::setExposure(double exposure) {
  wb_camera_set_exposure(getTag(), exposure);
}

double Camera::getFocalLength() const {
  return wb_camera_get_focal_length(getTag());
}

double Camera::getFocalDistance() const {
  return wb_camera_get_focal_distance(getTag());
}

double Camera::getMaxFocalDistance() const {
  return wb_camera_get_max_focal_distance(getTag());
}

double Camera::getMinFocalDistance() const {
  return wb_camera_get_min_focal_distance(getTag());
}

void Camera::setFocalDistance(double focalDistance) {
  wb_camera_set_focal_distance(getTag(), focalDistance);
}

double Camera::getNear() const {
  return wb_camera_get_near(getTag());
}

int Camera::saveImage(const string &filename, int quality) const {
  return wb_camera_save_image(getTag(), filename.c_str(), quality);
}

bool Camera::hasRecognition() const {
  return wb_camera_has_recognition(getTag());
}

void Camera::recognitionEnable(int samplingPeriod) {
  wb_camera_recognition_enable(getTag(), samplingPeriod);
}

void Camera::recognitionDisable() {
  wb_camera_recognition_disable(getTag());
}

int Camera::getRecognitionSamplingPeriod() const {
  return wb_camera_recognition_get_sampling_period(getTag());
}

int Camera::getRecognitionNumberOfObjects() const {
  return wb_camera_recognition_get_number_of_objects(getTag());
}

const CameraRecognitionObject *Camera::getRecognitionObjects() const {
  return wb_camera_recognition_get_objects(getTag());
}

bool Camera::hasRecognitionSegmentation() const {
  return wb_camera_recognition_has_segmentation(getTag());
}

void Camera::enableRecognitionSegmentation() {
  wb_camera_recognition_enable_segmentation(getTag());
}

void Camera::disableRecognitionSegmentation() {
  wb_camera_recognition_disable_segmentation(getTag());
}

bool Camera::isRecognitionSegmentationEnabled() const {
  return wb_camera_recognition_is_segmentation_enabled(getTag());
}

const unsigned char *Camera::getRecognitionSegmentationImage() const {
  return wb_camera_recognition_get_segmentation_image(getTag());
}

int Camera::saveRecognitionSegmentationImage(const string &filename, int quality) const {
  return wb_camera_recognition_save_segmentation_image(getTag(), filename.c_str(), quality);
}
