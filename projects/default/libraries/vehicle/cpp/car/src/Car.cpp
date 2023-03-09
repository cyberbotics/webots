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

/*
 * Description:   CPP wrapper of the car library
 * Comments:      Sponsored by the CTI project RO2IVSim
 *                (http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)
 */

#include <webots/vehicle/car.h>
#include <webots/vehicle/Car.hpp>

#include <cassert>

using namespace webots;

Car::Type Car::getType() {
  assert(this);
  return Type(wbu_car_get_type());
}

Car::EngineType Car::getEngineType() {
  assert(this);
  return EngineType(wbu_car_get_engine_type());
}

void Car::setIndicatorPeriod(double period) {
  assert(this);
  wbu_car_set_indicator_period(period);
}

double Car::getIndicatorPeriod() {
  assert(this);
  return wbu_car_get_indicator_period();
}

bool Car::getBackwardsLights() {
  assert(this);
  return wbu_car_get_backwards_lights();
}

bool Car::getBrakeLights() {
  assert(this);
  return wbu_car_get_brake_lights();
}

double Car::getTrackFront() {
  assert(this);
  return wbu_car_get_track_front();
}

double Car::getTrackRear() {
  assert(this);
  return wbu_car_get_track_rear();
}

double Car::getWheelbase() {
  assert(this);
  return wbu_car_get_wheelbase();
}

double Car::getFrontWheelRadius() {
  assert(this);
  return wbu_car_get_front_wheel_radius();
}

double Car::getRearWheelRadius() {
  assert(this);
  return wbu_car_get_rear_wheel_radius();
}

double Car::getWheelEncoder(WheelIndex wheel) {
  assert(this);
  return wbu_car_get_wheel_encoder(WbuCarWheelIndex(wheel));
}

double Car::getWheelSpeed(WheelIndex wheel) {
  assert(this);
  return wbu_car_get_wheel_speed(WbuCarWheelIndex(wheel));
}

void Car::setRightSteeringAngle(double angle) {
  assert(this);
  wbu_car_set_right_steering_angle(angle);
}

void Car::setLeftSteeringAngle(double angle) {
  assert(this);
  wbu_car_set_left_steering_angle(angle);
}

double Car::getRightSteeringAngle() {
  assert(this);
  return wbu_car_get_right_steering_angle();
}

double Car::getLeftSteeringAngle() {
  assert(this);
  return wbu_car_get_left_steering_angle();
}

void Car::enableLimitedSlipDifferential(bool enable) {
  assert(this);
  wbu_car_enable_limited_slip_differential(enable);
}

void Car::enableIndicatorAutoDisabling(bool enable) {
  assert(this);
  wbu_car_enable_indicator_auto_disabling(enable);
}
