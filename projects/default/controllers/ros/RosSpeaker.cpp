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

#include "RosSpeaker.hpp"

RosSpeaker::RosSpeaker(Speaker *speaker, Ros *ros) : RosDevice(speaker, ros) {
  std::string fixedDeviceName = RosDevice::fixedDeviceName();
  mStopServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/stop", &RosSpeaker::stopCallback);
  mGetEngineServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_engine", &RosSpeaker::getEngineCallback);
  mGetLanguageServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/get_language", &RosSpeaker::getLanguageCallback);
  mSetEngineServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_engine", &RosSpeaker::setEngineCallback);
  mSetLanguageServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/set_language", &RosSpeaker::setLanguageCallback);
  mSpeakServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/speak", &RosSpeaker::speakCallback);
  mPlayServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/play_sound", &RosSpeaker::playCallback);
  mIsSpeakingServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/is_speaking", &RosSpeaker::isSpeakingCallback);
  mIsPlayingServer = RosDevice::rosAdvertiseService(fixedDeviceName + "/is_sound_playing", &RosSpeaker::isPlayingCallback);
  mSpeaker = speaker;
}

RosSpeaker::~RosSpeaker() {
  mStopServer.shutdown();
  mGetEngineServer.shutdown();
  mGetLanguageServer.shutdown();
  mSetEngineServer.shutdown();
  mSetLanguageServer.shutdown();
  mSpeakServer.shutdown();
  mPlayServer.shutdown();
  mIsSpeakingServer.shutdown();
  mIsPlayingServer.shutdown();
}

bool RosSpeaker::stopCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res) {
  mSpeaker->stop(req.value);
  res.success = true;
  return true;
}

bool RosSpeaker::getEngineCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res) {
  res.value = mSpeaker->getEngine();
  return true;
}

bool RosSpeaker::getLanguageCallback(webots_ros::get_string::Request &req, webots_ros::get_string::Response &res) {
  res.value = mSpeaker->getLanguage();
  return true;
}

bool RosSpeaker::setEngineCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res) {
  res.success = mSpeaker->setEngine(req.value);
  return true;
}

bool RosSpeaker::setLanguageCallback(webots_ros::set_string::Request &req, webots_ros::set_string::Response &res) {
  res.success = mSpeaker->setLanguage(req.value);
  return true;
}

bool RosSpeaker::speakCallback(webots_ros::speaker_speak::Request &req, webots_ros::speaker_speak::Response &res) {
  mSpeaker->speak(req.text, req.volume);
  res.success = 1;
  return true;
}

bool RosSpeaker::playCallback(webots_ros::speaker_play_sound::Request &req, webots_ros::speaker_play_sound::Response &res) {
  Speaker::playSound(mSpeaker, mSpeaker, req.sound, req.volume, req.pitch, req.balance, req.loop);
  res.success = 1;
  return true;
}

bool RosSpeaker::isSpeakingCallback(webots_ros::get_bool::Request &req, webots_ros::get_bool::Response &res) {
  res.value = mSpeaker->isSpeaking();
  return true;
}

bool RosSpeaker::isPlayingCallback(webots_ros::speaker_is_sound_playing::Request &req,
                                   webots_ros::speaker_is_sound_playing::Response &res) {
  res.value = mSpeaker->isSoundPlaying(req.sound);
  return true;
}
