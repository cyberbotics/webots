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

#ifndef ROS_MOTOR_SENSOR_HPP
#define ROS_MOTOR_SENSOR_HPP

#include <webots/Motor.hpp>
#include "RosSensor.hpp"

using namespace webots;

class RosMotorSensor : public RosSensor {
public:
  RosMotorSensor(Motor *motor, const std::string &sensorName, Ros *ros);
  virtual ~RosMotorSensor() { cleanup(); }

  ros::Publisher createPublisher() override;
  void publishValue(ros::Publisher publisher) override;
  void rosEnable(int samplingPeriod) override;
  void rosDisable() override { cleanup(); }
  int rosSamplingPeriod() override;

private:
  void cleanup();

  Motor *mMotor;
};

#endif  // ROS_MOTOR_SENSOR_HPP
