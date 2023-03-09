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

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <webots/Device.hpp>
#include "../../c/webots/camera_recognition_object.h"

namespace webots {
  typedef WbCameraRecognitionObject CameraRecognitionObject;

  class Camera : public Device {
  public:
    explicit Camera(const std::string &name) : Device(name) {}  // Use Robot::getCamera() instead
    virtual ~Camera() {}
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    const unsigned char *getImage() const;
    int getWidth() const;
    int getHeight() const;
    double getFov() const;
    double getMaxFov() const;
    double getMinFov() const;
    virtual void setFov(double fov);
    double getExposure() const;
    void setExposure(double exposure);
    double getFocalLength() const;
    double getFocalDistance() const;
    double getMaxFocalDistance() const;
    double getMinFocalDistance() const;
    virtual void setFocalDistance(double focalDistance);
    double getNear() const;
    int saveImage(const std::string &filename, int quality) const;
    bool hasRecognition() const;
    void recognitionEnable(int samplingPeriod);
    void recognitionDisable();
    int getRecognitionSamplingPeriod() const;
    int getRecognitionNumberOfObjects() const;
    const CameraRecognitionObject *getRecognitionObjects() const;
    bool hasRecognitionSegmentation() const;
    void enableRecognitionSegmentation();
    void disableRecognitionSegmentation();
    bool isRecognitionSegmentationEnabled() const;
    const unsigned char *getRecognitionSegmentationImage() const;
    int saveRecognitionSegmentationImage(const std::string &filename, int quality) const;
    static unsigned char imageGetRed(const unsigned char *image, int width, int x, int y);
    static unsigned char imageGetGreen(const unsigned char *image, int width, int x, int y);
    static unsigned char imageGetBlue(const unsigned char *image, int width, int x, int y);
    static unsigned char imageGetGray(const unsigned char *image, int width, int x, int y);
    // alias
    static unsigned char imageGetGrey(const unsigned char *image, int width, int x, int y);
  };
}  // namespace webots
#endif  // CAMERA_HPP
