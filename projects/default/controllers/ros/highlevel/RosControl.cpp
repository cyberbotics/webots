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

// Description: Webots integration with for `ros_control`.

#include "RosControl.hpp"

namespace highlevel {

  RosControl::RosControl(webots::Robot *robot, ros::NodeHandle *nodeHandle) {
    mWebotsHW = new WebotsHW(robot);
    mControllerManager = new controller_manager::ControllerManager(mWebotsHW, *nodeHandle);
    mLastUpdate = ros::Time::now();
  }

  RosControl::~RosControl() {
    delete mWebotsHW;
    delete mControllerManager;
  }

  void RosControl::read() {
    const ros::Duration duration = ros::Time::now() - mLastUpdate;
    mWebotsHW->read(duration);
    mControllerManager->update(ros::Time::now(), duration);
    mLastUpdate = ros::Time::now();
  }

  void RosControl::write() {
    mWebotsHW->write();
  }
}  // namespace highlevel
