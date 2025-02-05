// Copyright 1996-2024 Cyberbotics Ltd.
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

#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <webots/Device.hpp>
#include <webots/ImageRef.hpp>

namespace webots {
  class Camera;
  class Display : public Device {
  public:
    enum { RGB = 3, RGBA, ARGB, BGRA, ABGR };
    explicit Display(const std::string &name) : Device(name) {}  // Use Robot::getDisplay() instead
    explicit Display(WbDeviceTag tag) : Device(tag) {}
    virtual ~Display() {}
    int getWidth() const;
    int getHeight() const;
    virtual void setColor(int color);
    virtual void setAlpha(double alpha);
    virtual void setOpacity(double opacity);
    virtual void setFont(const std::string &font, int size, bool antiAliasing);
    virtual void attachCamera(Camera *camera);
    virtual void detachCamera();
    virtual void drawPixel(int x1, int y1);
    virtual void drawLine(int x1, int y1, int x2, int y2);
    virtual void drawRectangle(int x, int y, int width, int height);
    virtual void drawOval(int cx, int cy, int a, int b);
    virtual void drawPolygon(const int *x, const int *y, int size);
    virtual void drawText(const std::string &txt, int x, int y);
    virtual void fillRectangle(int x, int y, int width, int height);
    virtual void fillOval(int cx, int cy, int a, int b);
    virtual void fillPolygon(const int *x, const int *y, int size);
    ImageRef *imageNew(int width, int height, const void *data, int format) const;
    ImageRef *imageCopy(int x, int y, int width, int height) const;
    virtual void imagePaste(ImageRef *ir, int x, int y, bool blend = false);
    ImageRef *imageLoad(const std::string &filename) const;
    void imageSave(ImageRef *ir, const std::string &filename) const;
    void imageDelete(ImageRef *ir) const;
  };
}  // namespace webots
#endif  // DISPLAY_HPP
