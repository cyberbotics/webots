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

#ifndef WEBOTS_HW_HPP
#define WEBOTS_HW_HPP

#include <vector>

#include <webots/Device.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace highlevel {
  struct ControlledMotor {
    webots::Motor *motor;
    double commandPosition;
    double commandVelocity;
    double position;
    double velocity;
    double effort;
  };

  class WebotsHW : public hardware_interface::RobotHW {
  public:
    explicit WebotsHW(webots::Robot *robot);
    WebotsHW() = delete;
    void read(const ros::Duration &duration);
    void write();
    void doSwitch(const std::list<hardware_interface::ControllerInfo> &startList,
                  const std::list<hardware_interface::ControllerInfo> &stopList) override;

  private:
    hardware_interface::JointStateInterface mJointStateInterface;
    hardware_interface::PositionJointInterface mPositionJointInteraface;
    hardware_interface::VelocityJointInterface mVelocityJointInteraface;
    std::vector<ControlledMotor> mControlledMotors;
    webots::Robot *mRobot;
  };
}  // namespace highlevel

#endif
