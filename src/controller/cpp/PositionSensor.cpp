// Copyright 1996-2024 Cyberbotics Ltd.
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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/device.h>
#include <webots/position_sensor.h>
#include <webots/Brake.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

using namespace webots;

void PositionSensor::enable(int sampling_period) {
  wb_position_sensor_enable(getTag(), sampling_period);
}

void PositionSensor::disable() {
  wb_position_sensor_disable(getTag());
}

int PositionSensor::getSamplingPeriod() const {
  return wb_position_sensor_get_sampling_period(getTag());
}

double PositionSensor::getValue() const {
  return wb_position_sensor_get_value(getTag());
}

PositionSensor::Type PositionSensor::getType() const {
  return Type(wb_position_sensor_get_type(getTag()));
}

Brake *PositionSensor::getBrake() {
  if (brake == NULL)
    brake = dynamic_cast<Brake *>(Robot::getDeviceFromTag(getBrakeTag()));
  return brake;
}

int PositionSensor::getBrakeTag() const {
  return wb_position_sensor_get_brake(getTag());
}

Motor *PositionSensor::getMotor() {
  if (motor == NULL)
    motor = dynamic_cast<Motor *>(Robot::getDeviceFromTag(getMotorTag()));
  return motor;
}

int PositionSensor::getMotorTag() const {
  return wb_position_sensor_get_motor(getTag());
}
