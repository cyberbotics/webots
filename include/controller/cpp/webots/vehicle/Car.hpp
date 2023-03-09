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

#ifndef CAR_HPP
#define CAR_HPP

#include <webots/vehicle/Driver.hpp>

namespace webots {
  class Car : public Driver {
  public:
    typedef enum { TRACTION = 0, PROPULSION, FOUR_BY_FOUR } Type;

    typedef enum { COMBUSTION_ENGINE = 0, ELECTRIC_ENGINE, PARALLEL_HYBRID_ENGINE, POWER_SPLIT_HYBRID_ENGINE } EngineType;

    typedef enum { WHEEL_FRONT_RIGHT = 0, WHEEL_FRONT_LEFT, WHEEL_REAR_RIGHT, WHEEL_REAR_LEFT, WHEEL_NB } WheelIndex;

    Car() : Driver() {}
    virtual ~Car() {}

    Type getType();
    EngineType getEngineType();

    void setIndicatorPeriod(double period);
    double getIndicatorPeriod();

    bool getBackwardsLights();
    bool getBrakeLights();

    double getTrackFront();
    double getTrackRear();
    double getWheelbase();
    double getFrontWheelRadius();
    double getRearWheelRadius();

    double getWheelEncoder(WheelIndex wheel);
    double getWheelSpeed(WheelIndex wheel);
    void setLeftSteeringAngle(double angle);
    void setRightSteeringAngle(double angle);
    double getRightSteeringAngle();
    double getLeftSteeringAngle();

    void enableLimitedSlipDifferential(bool enable);
    void enableIndicatorAutoDisabling(bool enable);
  };
}  // namespace webots

#endif  // CAR_HPP
