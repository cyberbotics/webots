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

#include "WebotsHW.hpp"

namespace highlevel {

  WebotsHW::WebotsHW(webots::Robot *robot) : mRobot(robot) {
    // Find all motors
    const int nDevices = mRobot->getNumberOfDevices();
    for (int i = 0; i < nDevices; i++) {
      webots::Device *tempDevice = mRobot->getDeviceByIndex(i);
      if (tempDevice->getNodeType() == webots::Node::ROTATIONAL_MOTOR ||
          tempDevice->getNodeType() == webots::Node::LINEAR_MOTOR) {
        webots::Motor *motor = dynamic_cast<webots::Motor *>(tempDevice);
        ControlledMotor controlledMotor = {
          .motor = motor, .commandPosition = NAN, .commandVelocity = NAN, .position = 0, .velocity = 0, .effort = 0};
        mControlledMotors.push_back(controlledMotor);
      }
    }

    // Create RobotHW interface
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
                                                       &controlledMotor.commandPosition);
        mPositionJointInteraface.registerHandle(positionHandle);

        // Register `velocity` handle
        hardware_interface::JointHandle velocityHandle(mJointStateInterface.getHandle(controlledMotor.motor->getName()),
                                                       &controlledMotor.commandVelocity);
        mVelocityJointInteraface.registerHandle(velocityHandle);
      }
    }
    registerInterface(&mJointStateInterface);
    registerInterface(&mPositionJointInteraface);
    registerInterface(&mVelocityJointInteraface);
  }

  void WebotsHW::read(const ros::Duration &duration) {
    for (ControlledMotor &controlledMotor : mControlledMotors) {
      if (controlledMotor.motor->getPositionSensor()) {
        const double previousPosition = controlledMotor.position;
        const double newPosition = controlledMotor.motor->getPositionSensor()->getValue();

        controlledMotor.position = newPosition;
        controlledMotor.velocity = (newPosition - previousPosition) / duration.toSec();
      }
    }
  }

  void WebotsHW::write() {
    for (ControlledMotor &controlledMotor : mControlledMotors) {
      if (!std::isnan(controlledMotor.commandVelocity))
        controlledMotor.motor->setVelocity(controlledMotor.commandVelocity);
      if (!std::isnan(controlledMotor.commandPosition))
        controlledMotor.motor->setPosition(controlledMotor.commandPosition);
    }
  }

  void WebotsHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &startList,
                          const std::list<hardware_interface::ControllerInfo> &stopList) {
    for (ControlledMotor &controlledMotor : mControlledMotors) {
      controlledMotor.commandVelocity = NAN;
      controlledMotor.commandPosition = NAN;
    }

    // Change motor's behavior depending on control type (e.g. position or velocity)
    for (hardware_interface::ControllerInfo controllerInfo : startList) {
      for (hardware_interface::InterfaceResources interfaceResources : controllerInfo.claimed_resources) {
        for (std::string resource : interfaceResources.resources) {
          if (interfaceResources.hardware_interface == "hardware_interface::VelocityJointInterface")
            mRobot->getMotor(resource)->setPosition(INFINITY);
          else if (interfaceResources.hardware_interface == "hardware_interface::PositionJointInterface")
            mRobot->getMotor(resource)->setVelocity(mRobot->getMotor(resource)->getMaxVelocity());
        }
      }
    }

    hardware_interface::RobotHW::doSwitch(startList, stopList);
  }
}  // namespace highlevel
