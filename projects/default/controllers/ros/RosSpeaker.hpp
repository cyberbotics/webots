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

#ifndef ROS_SPEAKER_HPP
#define ROS_SPEAKER_HPP

#include <webots/Speaker.hpp>
#include "RosDevice.hpp"

#include <webots_ros/get_bool.h>
#include <webots_ros/get_string.h>
#include <webots_ros/set_string.h>

#include <webots_ros/speaker_is_sound_playing.h>
#include <webots_ros/speaker_play_sound.h>
#include <webots_ros/speaker_speak.h>

using namespace webots;

class RosSpeaker : public RosDevice {
public:
  RosSpeaker(Speaker *speaker, Ros *ros);
  virtual ~RosSpeaker();

  bool stopCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res);
  bool getEngineCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool getLanguageCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res);
  bool setEngineCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res);
  bool setLanguageCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res);
  bool speakCallback(webots_ros::speaker_speak::Request &req, webots_ros::speaker_speak::Response &res);
  bool playCallback(webots_ros::speaker_play_sound::Request &req, webots_ros::speaker_play_sound::Response &res);
  bool isSpeakingCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res);
  bool isPlayingCallback(webots_ros::speaker_is_sound_playing::Request &req,
                         webots_ros::speaker_is_sound_playing::Response &res);

private:
  Speaker *mSpeaker;
  ros::ServiceServer mStopServer;
  ros::ServiceServer mGetEngineServer;
  ros::ServiceServer mGetLanguageServer;
  ros::ServiceServer mSetEngineServer;
  ros::ServiceServer mSetLanguageServer;
  ros::ServiceServer mSpeakServer;
  ros::ServiceServer mPlayServer;
  ros::ServiceServer mIsSpeakingServer;
  ros::ServiceServer mIsPlayingServer;
};

#endif  // ROS_SPEAKER_HPP
