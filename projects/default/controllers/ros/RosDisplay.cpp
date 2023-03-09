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

#include "RosDisplay.hpp"

#include <webots/Camera.hpp>

RosDisplay::RosDisplay(Display *display, Ros *ros) : RosDevice(display, ros) {
  mDisplay = display;
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mInfoServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_info", &RosDisplay::getInfoCallback);
  mColorServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_color", &RosDisplay::setColorCallback);
  mAlphaServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_alpha", &RosDisplay::setAlphaCallback);
  mOpacityServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_opacity", &RosDisplay::setOpacityCallback);
  mFontServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_font", &RosDisplay::setFontCallback);
  mAttachCameraServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/attach_camera", &RosDisplay::attachCameraCallback);
  mDetachCameraServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/detach_camera", &RosDisplay::detachCameraCallback);
  mDrawPixelServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/draw_pixel", &RosDisplay::drawPixelCallback);
  mDrawLineServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/draw_line", &RosDisplay::drawLineCallback);
  mDrawRectangleServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/draw_rectangle", &RosDisplay::drawRectangleCallback);
  mDrawOvalServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/draw_oval", &RosDisplay::drawOvalCallback);
  mDrawPolygonServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/draw_polygon", &RosDisplay::drawPolygonCallback);
  mDrawTextServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/draw_text", &RosDisplay::drawTextCallback);
  mFillRectangleServer =
    RosDevice::rosAdvertiseService(fixedDeviceName + "/fill_rectangle", &RosDisplay::fillRectangleCallback);
  mFillOvalServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/fill_oval", &RosDisplay::fillOvalCallback);
  mFillPolygonServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/fill_polygon", &RosDisplay::fillPolygonCallback);
  mImageNewServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/image_new", &RosDisplay::imageNewCallback);
  mImageCopyServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/image_copy", &RosDisplay::imageCopyCallback);
  mImagePasteServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/image_paste", &RosDisplay::imagePasteCallback);
  mImageLoadServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/image_load", &RosDisplay::imageLoadCallback);
  mImageSaveServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/image_save", &RosDisplay::imageSaveCallback);
  mImageDeleteServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/image_delete", &RosDisplay::imageDeleteCallback);
}

RosDisplay::~RosDisplay() {
  mInfoServer.shutdown();
  mColorServer.shutdown();
  mAlphaServer.shutdown();
  mOpacityServer.shutdown();
  mFontServer.shutdown();
  mAttachCameraServer.shutdown();
  mDetachCameraServer.shutdown();
  mDrawPixelServer.shutdown();
  mDrawLineServer.shutdown();
  mDrawRectangleServer.shutdown();
  mDrawOvalServer.shutdown();
  mDrawPolygonServer.shutdown();
  mDrawTextServer.shutdown();
  mFillRectangleServer.shutdown();
  mFillOvalServer.shutdown();
  mFillPolygonServer.shutdown();
  mImageNewServer.shutdown();
  mImageCopyServer.shutdown();
  mImagePasteServer.shutdown();
  mImageLoadServer.shutdown();
  mImageSaveServer.shutdown();
  mImageDeleteServer.shutdown();
}

bool RosDisplay::getInfoCallback(webots_ros::display_get_info::Request &req, webots_ros::display_get_info::Response &res) {
  assert(mDisplay);
  res.width = mDisplay->getWidth();
  res.height = mDisplay->getHeight();
  return true;
}

bool RosDisplay::setColorCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res) {
  mDisplay->setColor(req.value);
  res.success = true;
  return true;
}

bool RosDisplay::setAlphaCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mDisplay->setAlpha(req.value);
  res.success = true;
  return true;
}

bool RosDisplay::setOpacityCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res) {
  mDisplay->setOpacity(req.value);
  res.success = true;
  return true;
}

bool RosDisplay::setFontCallback(webots_ros::display_set_font::Request &req, webots_ros::display_set_font::Response &res) {
  mDisplay->setFont(req.font, req.size, req.antiAliasing == 1);
  res.success = 1;
  return true;
}

// cppcheck-suppress constParameter
bool RosDisplay::attachCameraCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res) {
  Device *device = mRos->getDevice(req.value);
  Camera *camera = static_cast<Camera *>(device);
  if (camera) {
    mDisplay->attachCamera(camera);
    res.success = true;
  } else
    res.success = false;
  return true;
}

bool RosDisplay::detachCameraCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  mDisplay->detachCamera();
  res.value = true;
  return true;
}

bool RosDisplay::drawPixelCallback(webots_ros::display_draw_pixel::Request &req,
                                   webots_ros::display_draw_pixel::Response &res) {
  mDisplay->drawPixel(req.x1, req.y1);
  res.success = 1;
  return true;
}

bool RosDisplay::drawLineCallback(webots_ros::display_draw_line::Request &req, webots_ros::display_draw_line::Response &res) {
  mDisplay->drawLine(req.x1, req.y1, req.x2, req.y2);
  res.success = 1;
  return true;
}

bool RosDisplay::drawRectangleCallback(webots_ros::display_draw_rectangle::Request &req,
                                       webots_ros::display_draw_rectangle::Response &res) {
  mDisplay->drawRectangle(req.x, req.y, req.width, req.height);
  res.success = 1;
  return true;
}

bool RosDisplay::drawOvalCallback(webots_ros::display_draw_oval::Request &req, webots_ros::display_draw_oval::Response &res) {
  mDisplay->drawOval(req.cx, req.cy, req.a, req.b);
  res.success = 1;
  return true;
}

bool RosDisplay::drawPolygonCallback(webots_ros::display_draw_polygon::Request &req,
                                     webots_ros::display_draw_polygon::Response &res) {
  int xTemp[req.x.size()];
  int yTemp[req.y.size()];
  for (unsigned int i = 0; i < req.x.size(); i++)
    xTemp[i] = req.x[i];
  for (unsigned int i = 0; i < req.y.size(); i++)
    yTemp[i] = req.y[i];
  mDisplay->drawPolygon(xTemp, yTemp, req.size);
  res.success = 1;
  return true;
}

bool RosDisplay::drawTextCallback(webots_ros::display_draw_text::Request &req, webots_ros::display_draw_text::Response &res) {
  mDisplay->drawText(req.text, req.x, req.y);
  res.success = 1;
  return true;
}

bool RosDisplay::fillRectangleCallback(webots_ros::display_draw_rectangle::Request &req,
                                       webots_ros::display_draw_rectangle::Response &res) {
  mDisplay->fillRectangle(req.x, req.y, req.width, req.height);
  res.success = 1;
  return true;
}

bool RosDisplay::fillOvalCallback(webots_ros::display_draw_oval::Request &req, webots_ros::display_draw_oval::Response &res) {
  mDisplay->fillOval(req.cx, req.cy, req.a, req.b);
  res.success = 1;
  return true;
}

bool RosDisplay::fillPolygonCallback(webots_ros::display_draw_polygon::Request &req,
                                     webots_ros::display_draw_polygon::Response &res) {
  int xTemp[req.x.size()];
  int yTemp[req.y.size()];
  for (unsigned int i = 0; i < req.x.size(); i++)
    xTemp[i] = req.x[i];
  for (unsigned int i = 0; i < req.y.size(); i++)
    yTemp[i] = req.y[i];
  mDisplay->fillPolygon(xTemp, yTemp, req.size);
  res.success = 1;
  return true;
}

bool RosDisplay::imageNewCallback(webots_ros::display_image_new::Request &req, webots_ros::display_image_new::Response &res) {
  assert(mDisplay);
  void *data;
  char buffer[req.data.size()];
  for (unsigned int i = 0; i < req.data.size(); i++)
    buffer[i] = req.data[i];
  data = buffer;
  res.ir = reinterpret_cast<uint64_t>(mDisplay->imageNew(req.width, req.height, data, req.format));
  return true;
}

bool RosDisplay::imageCopyCallback(webots_ros::display_image_copy::Request &req,
                                   webots_ros::display_image_copy::Response &res) {
  assert(mDisplay);
  res.ir = reinterpret_cast<uint64_t>(mDisplay->imageCopy(req.x, req.y, req.width, req.height));
  return true;
}

bool RosDisplay::imagePasteCallback(webots_ros::display_image_paste::Request &req,
                                    webots_ros::display_image_paste::Response &res) {
  assert(mDisplay);
  mDisplay->imagePaste(reinterpret_cast<ImageRef *>(req.ir), req.x, req.y, req.blend);
  res.success = 1;
  return true;
}

bool RosDisplay::imageLoadCallback(webots_ros::display_image_load::Request &req,
                                   webots_ros::display_image_load::Response &res) {
  assert(mDisplay);
  res.ir = reinterpret_cast<uint64_t>(mDisplay->imageLoad(req.filename));
  return true;
}

bool RosDisplay::imageSaveCallback(webots_ros::display_image_save::Request &req,
                                   webots_ros::display_image_save::Response &res) {
  assert(mDisplay);
  mDisplay->imageSave(reinterpret_cast<ImageRef *>(req.ir), req.filename);
  res.success = 1;
  return true;
}

bool RosDisplay::imageDeleteCallback(webots_ros::display_image_delete::Request &req,
                                     webots_ros::display_image_delete::Response &res) {
  assert(mDisplay);
  mDisplay->imageDelete(reinterpret_cast<ImageRef *>(req.ir));
  res.success = 1;
  return true;
}
