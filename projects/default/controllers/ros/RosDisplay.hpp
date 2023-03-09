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

#ifndef ROS_DISPLAY_HPP
#define ROS_DISPLAY_HPP

#include <webots/Display.hpp>
#include "RosDevice.hpp"

#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_string.h>

#include <webots_ros/display_draw_line.h>
#include <webots_ros/display_draw_oval.h>
#include <webots_ros/display_draw_pixel.h>
#include <webots_ros/display_draw_polygon.h>
#include <webots_ros/display_draw_rectangle.h>
#include <webots_ros/display_draw_text.h>
#include <webots_ros/display_get_info.h>
#include <webots_ros/display_image_copy.h>
#include <webots_ros/display_image_delete.h>
#include <webots_ros/display_image_load.h>
#include <webots_ros/display_image_new.h>
#include <webots_ros/display_image_paste.h>
#include <webots_ros/display_image_save.h>
#include <webots_ros/display_set_font.h>

using namespace webots;

class RosDisplay : public RosDevice {
public:
  RosDisplay(Display *display, Ros *ros);
  virtual ~RosDisplay();

  bool getInfoCallback(webots_ros::display_get_info::Request &req, webots_ros::display_get_info::Response &res);
  bool setColorCallback(webots_ros::set_int::Request &req, webots_ros::set_int::Response &res);
  bool setAlphaCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setOpacityCallback(webots_ros::set_float::Request &req, webots_ros::set_float::Response &res);
  bool setFontCallback(webots_ros::display_set_font::Request &req, webots_ros::display_set_font::Response &res);
  bool attachCameraCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res);
  bool detachCameraCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool drawPixelCallback(webots_ros::display_draw_pixel::Request &req, webots_ros::display_draw_pixel::Response &res);
  bool drawLineCallback(webots_ros::display_draw_line::Request &req, webots_ros::display_draw_line::Response &res);
  bool drawRectangleCallback(webots_ros::display_draw_rectangle::Request &req,
                             webots_ros::display_draw_rectangle::Response &res);
  bool drawOvalCallback(webots_ros::display_draw_oval::Request &req, webots_ros::display_draw_oval::Response &res);
  bool drawPolygonCallback(webots_ros::display_draw_polygon::Request &req, webots_ros::display_draw_polygon::Response &res);
  bool drawTextCallback(webots_ros::display_draw_text::Request &req, webots_ros::display_draw_text::Response &res);
  bool fillRectangleCallback(webots_ros::display_draw_rectangle::Request &req,
                             webots_ros::display_draw_rectangle::Response &res);
  bool fillOvalCallback(webots_ros::display_draw_oval::Request &req, webots_ros::display_draw_oval::Response &res);
  bool fillPolygonCallback(webots_ros::display_draw_polygon::Request &req, webots_ros::display_draw_polygon::Response &res);
  bool imageNewCallback(webots_ros::display_image_new::Request &req, webots_ros::display_image_new::Response &res);
  bool imageCopyCallback(webots_ros::display_image_copy::Request &req, webots_ros::display_image_copy::Response &res);
  bool imagePasteCallback(webots_ros::display_image_paste::Request &req, webots_ros::display_image_paste::Response &res);
  bool imageLoadCallback(webots_ros::display_image_load::Request &req, webots_ros::display_image_load::Response &res);
  bool imageSaveCallback(webots_ros::display_image_save::Request &req, webots_ros::display_image_save::Response &res);
  bool imageDeleteCallback(webots_ros::display_image_delete::Request &req, webots_ros::display_image_delete::Response &res);

private:
  Display *mDisplay;
  ros::ServiceServer mInfoServer;
  ros::ServiceServer mColorServer;
  ros::ServiceServer mAlphaServer;
  ros::ServiceServer mOpacityServer;
  ros::ServiceServer mFontServer;
  ros::ServiceServer mAttachCameraServer;
  ros::ServiceServer mDetachCameraServer;
  ros::ServiceServer mDrawPixelServer;
  ros::ServiceServer mDrawLineServer;
  ros::ServiceServer mDrawRectangleServer;
  ros::ServiceServer mDrawOvalServer;
  ros::ServiceServer mDrawPolygonServer;
  ros::ServiceServer mDrawTextServer;
  ros::ServiceServer mFillRectangleServer;
  ros::ServiceServer mFillOvalServer;
  ros::ServiceServer mFillPolygonServer;
  ros::ServiceServer mImageNewServer;
  ros::ServiceServer mImageCopyServer;
  ros::ServiceServer mImagePasteServer;
  ros::ServiceServer mImageLoadServer;
  ros::ServiceServer mImageSaveServer;
  ros::ServiceServer mImageDeleteServer;
};

#endif  // ROS_DISPLAY_HPP
