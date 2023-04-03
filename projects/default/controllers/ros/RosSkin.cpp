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

#include "RosSkin.hpp"

#include "RosMathUtils.hpp"

RosSkin::RosSkin(Skin *skin, Ros *ros) : RosDevice(skin, ros) {
  std::string deviceNameFixed = skin->getName();
  for (size_t i = 0; i < deviceNameFixed.size(); i++) {
    if (deviceNameFixed[i] == '-' || deviceNameFixed[i] == ' ')
      deviceNameFixed[i] = '_';
  }

  mSetBoneOrientationServer =
    RosDevice::rosAdvertiseService(deviceNameFixed + '/' + "set_bone_orientation", &RosSkin::setBoneOrientationCallback);
  mSetBonePositionServer =
    RosDevice::rosAdvertiseService(deviceNameFixed + '/' + "set_bone_position", &RosSkin::setBonePositionCallback);
  mGetBoneNameServer = RosDevice::rosAdvertiseService(deviceNameFixed + '/' + "get_bone_name", &RosSkin::getBoneNameCallback);
  mGetBoneCountServer =
    RosDevice::rosAdvertiseService(deviceNameFixed + '/' + "get_bone_count", &RosSkin::getBoneCountCallback);
  mGetBoneOrientationServer =
    RosDevice::rosAdvertiseService(deviceNameFixed + '/' + "get_bone_orientation", &RosSkin::getBoneOrientationCallback);
  mGetBonePositionServer =
    RosDevice::rosAdvertiseService(deviceNameFixed + '/' + "get_bone_position", &RosSkin::getBonePositionCallback);

  mSkin = skin;
}

bool RosSkin::setBoneOrientationCallback(webots_ros::skin_set_bone_orientation::Request &req,
                                         webots_ros::skin_set_bone_orientation::Response &res) {
  double axisAngleValues[4];
  RosMathUtils::quaternionToAxisAngle(req.orientation, axisAngleValues);
  mSkin->setBoneOrientation(req.index, axisAngleValues, req.absolute);
  res.success = 1;
  return true;
}

bool RosSkin::setBonePositionCallback(webots_ros::skin_set_bone_position::Request &req,
                                      webots_ros::skin_set_bone_position::Response &res) {
  double values[4];
  values[0] = req.position.x;
  values[1] = req.position.y;
  values[2] = req.position.z;
  const double *value = values;
  mSkin->setBonePosition(req.index, value, req.absolute);
  res.success = 1;
  return true;
}

bool RosSkin::getBoneNameCallback(webots_ros::skin_get_bone_name::Request &req, webots_ros::skin_get_bone_name::Response &res) {
  assert(mSkin);
  res.name = mSkin->getBoneName(req.index);
  return true;
}

bool RosSkin::getBoneCountCallback(webots_ros::get_int::Request &req, webots_ros::get_int::Response &res) {
  assert(mSkin);
  res.value = mSkin->getBoneCount();
  return true;
}

bool RosSkin::getBoneOrientationCallback(webots_ros::skin_get_bone_orientation::Request &req,
                                         webots_ros::skin_get_bone_orientation::Response &res) {
  assert(mSkin);
  const double *axisAngleValues = mSkin->getBoneOrientation(req.index, req.absolute);
  RosMathUtils::axisAngleToQuaternion(axisAngleValues, res.orientation);
  return true;
}

bool RosSkin::getBonePositionCallback(webots_ros::skin_get_bone_position::Request &req,
                                      webots_ros::skin_get_bone_position::Response &res) {
  assert(mSkin);
  const double *values = mSkin->getBonePosition(req.index, req.absolute);
  res.position.x = values[0];
  res.position.y = values[1];
  res.position.z = values[2];
  return true;
}
