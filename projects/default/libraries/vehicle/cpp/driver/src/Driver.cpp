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
 * Description:   CPP wrapper of the driver library
 * Comments:      Sponsored by the CTI project RO2IVSim
 *                (http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)
 */

#include <webots/vehicle/driver.h>
#include <webots/vehicle/Driver.hpp>

#include <cassert>

using namespace webots;

Driver *Driver::dInstance = NULL;

Driver::Driver() {
  if (dInstance == NULL) {
    dInstance = this;
    wbu_driver_init();
  } else {
    std::cerr << "Only one instance of the Driver class should be created" << std::endl;
    exit(-1);
  }
}

Driver::~Driver() {
  wbu_driver_cleanup();
}

bool Driver::isInitialisationPossible() {
  return wbu_driver_initialization_is_possible();
}

Driver *Driver::getDriverInstance() {
  if (dInstance)
    return dInstance;
  return new Driver();
}

int Driver::step() {
  assert(this);
  return wbu_driver_step();
}

void Driver::setSteeringAngle(double steeringAngle) {
  assert(this);
  wbu_driver_set_steering_angle(steeringAngle);
}

double Driver::getSteeringAngle() {
  assert(this);
  return wbu_driver_get_steering_angle();
}

void Driver::setCruisingSpeed(double speed) {
  assert(this);
  wbu_driver_set_cruising_speed(speed);
}

double Driver::getTargetCruisingSpeed() {
  assert(this);
  return wbu_driver_get_target_cruising_speed();
}

double Driver::getCurrentSpeed() {
  assert(this);
  return wbu_driver_get_current_speed();
}

void Driver::setThrottle(double throttle) {
  assert(this);
  wbu_driver_set_throttle(throttle);
}

double Driver::getThrottle() {
  assert(this);
  return wbu_driver_get_throttle();
}

void Driver::setBrakeIntensity(double intensity) {
  assert(this);
  wbu_driver_set_brake_intensity(intensity);
}

double Driver::getBrakeIntensity() {
  assert(this);
  return wbu_driver_get_brake_intensity();
}

void Driver::setBrake(double brake) {
  assert(this);
  std::cerr << "Warning: Deprecated 'setBrake' use 'setBrakeIntensity' instead." << std::endl;
  setBrakeIntensity(brake);
}

void Driver::setIndicator(IndicatorState state) {
  assert(this);
  wbu_driver_set_indicator(WbuDriverIndicatorState(state));
}

void Driver::setHazardFlashers(bool state) {
  assert(this);
  wbu_driver_set_hazard_flashers(state);
}

Driver::IndicatorState Driver::getIndicator() {
  assert(this);
  return IndicatorState(wbu_driver_get_indicator());
}

bool Driver::getHazardFlashers() {
  assert(this);
  return wbu_driver_get_hazard_flashers();
}

void Driver::setDippedBeams(bool state) {
  assert(this);
  wbu_driver_set_dipped_beams(state);
}

void Driver::setAntifogLights(bool state) {
  assert(this);
  wbu_driver_set_antifog_lights(state);
}

bool Driver::getDippedBeams() {
  assert(this);
  return wbu_driver_get_dipped_beams();
}

bool Driver::getAntifogLights() {
  assert(this);
  return wbu_driver_get_antifog_lights();
}

double Driver::getRpm() {
  assert(this);
  return wbu_driver_get_rpm();
}

int Driver::getGear() {
  assert(this);
  return wbu_driver_get_gear();
}

void Driver::setGear(int gear) {
  assert(this);
  wbu_driver_set_gear(gear);
}

int Driver::getGearNumber() {
  assert(this);
  return wbu_driver_get_gear_number();
}

Driver::ControlMode Driver::getControlMode() {
  assert(this);
  return ControlMode(wbu_driver_get_control_mode());
}

void Driver::setWiperMode(Driver::WiperMode mode) {
  assert(this);
  wbu_driver_set_wiper_mode(WbuDriverWiperMode(mode));
}

Driver::WiperMode Driver::getWiperMode() {
  assert(this);
  return WiperMode(wbu_driver_get_wiper_mode());
}

void Driver::setWipersMode(Driver::WiperMode mode) {
  assert(this);
  std::cerr << "Warning: Deprecated 'setWipersMode' use 'setWiperMode' instead." << std::endl;
  wbu_driver_set_wiper_mode(WbuDriverWiperMode(mode));
}

Driver::WiperMode Driver::getWipersMode() {
  assert(this);
  std::cerr << "Warning: Deprecated 'getWipersMode' use 'getWiperMode' instead." << std::endl;
  return WiperMode(wbu_driver_get_wiper_mode());
}
