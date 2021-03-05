// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Description: Webots integration with for `ros_control`.

#include "RosControl.hpp"

namespace highlevel {

  RosControl::RosControl(webots::Robot *robot) : mRobot(robot) {}

  void RosControl::init() {
    for (ControlledMotor &controlledMotor : mControlledMotors) {
      webots::PositionSensor *positionSensor = controlledMotor.motor->getPositionSensor();
      if (positionSensor) {
        positionSensor->enable(mRobot->getBasicTimeStep());

        // Register `state` handle
        hardware_interface::JointStateHandle stateHandle(controlledMotor.motor->getName(), &controlledMotor.position,
                                                         &controlledMotor.velocity, &controlledMotor.effort);
        mJointStateInterface.registerHandle(stateHandle);

        // Register `position` handle
        hardware_interface::JointHandle positionHandle(mJointStateInterface.getHandle(controlledMotor.motor->getName()),
                                                       &controlledMotor.command_position);
        mPositionJointInteraface.registerHandle(positionHandle);

        // Register `velocity` handle
        hardware_interface::JointHandle velocityHandle(mJointStateInterface.getHandle(controlledMotor.motor->getName()),
                                                       &controlledMotor.command_velocity);
        mVelocityJointInteraface.registerHandle(velocityHandle);
      }
    }
    registerInterface(&mJointStateInterface);
    registerInterface(&mPositionJointInteraface);
    registerInterface(&mVelocityJointInteraface);
  }

  void RosControl::addMotor(webots::Motor *motor) {
    ControlledMotor controlledMotor = {
      .motor = motor, .command_position = NAN, .command_velocity = NAN, .position = 0, .velocity = 0, .effort = 0};
    mControlledMotors.push_back(controlledMotor);
  }

  void RosControl::read() {
    for (ControlledMotor &controlledMotor : mControlledMotors) {
      controlledMotor.position = controlledMotor.motor->getPositionSensor()->getValue();
      controlledMotor.velocity = controlledMotor.motor->getVelocity();
    }
  }

  void RosControl::write() {
    for (ControlledMotor &controlledMotor : mControlledMotors) {
      if (!isnan(controlledMotor.command_velocity))
        controlledMotor.motor->setVelocity(controlledMotor.command_velocity);
      if (!isnan(controlledMotor.command_position))
        controlledMotor.motor->setPosition(controlledMotor.command_position);
    }
  }

  void RosControl::doSwitch(const std::list<hardware_interface::ControllerInfo> &startList,
                            const std::list<hardware_interface::ControllerInfo> &stopList) {
    for (ControlledMotor &controlledMotor : mControlledMotors) {
      controlledMotor.command_velocity = NAN;
      controlledMotor.command_position = NAN;
    }

    // TODO: Prepare for resource for the standard controllers
    for (hardware_interface::ControllerInfo controllerInfo : startList) {
        for (hardware_interface::InterfaceResources interfaceResources : controllerInfo.claimed_resources) {
            ROS_INFO("%s", interfaceResources.hardware_interface.c_str());
            for (std::string resource : interfaceResources.resources)
                ROS_INFO("%s", interfaceResources.hardware_interface.c_str());
        }
    }

    hardware_interface::RobotHW::doSwitch(startList, stopList);
  }
}  // namespace highlevel