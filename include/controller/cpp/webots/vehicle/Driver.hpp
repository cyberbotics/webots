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

/*
 * Description:   CPP wrapper of the driver library
 * Comments:      Sponsored by the CTI project RO2IVSim
 *                (http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)
 */

#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <webots/Supervisor.hpp>

namespace webots {
  class Driver : public Supervisor {
  public:
    typedef enum { INDICATOR_OFF, INDICATOR_RIGHT, INDICATOR_LEFT } IndicatorState;

    typedef enum { SPEED, TORQUE } ControlMode;

    typedef enum { DOWN, SLOW, NORMAL, FAST } WiperMode;

    // private function for webots_ros2 to identify robots that can use libdriver
    static bool isInitialisationPossible();

    Driver();
    static Driver *getDriverInstance();
    virtual ~Driver();

    virtual int step();

    // positive: turn right, negative: turn left
    void setSteeringAngle(double steeringAngle);
    double getSteeringAngle();

    void setCruisingSpeed(double speed);
    double getTargetCruisingSpeed();

    double getCurrentSpeed();

    void setThrottle(double throttle);
    double getThrottle();

    void setBrakeIntensity(double intensity);
    double getBrakeIntensity();

    void setIndicator(IndicatorState state);
    void setHazardFlashers(bool state);

    IndicatorState getIndicator();
    bool getHazardFlashers();

    void setDippedBeams(bool state);
    void setAntifogLights(bool state);

    bool getDippedBeams();
    bool getAntifogLights();

    double getRpm();
    int getGear();
    void setGear(int gear);
    int getGearNumber();
    ControlMode getControlMode();

    void setWiperMode(WiperMode mode);
    WiperMode getWiperMode();

    // kept for backward compatibility only
    void setBrake(double brake);

    void setWipersMode(WiperMode mode);
    WiperMode getWipersMode();

  private:
    virtual int step(int t) { return Supervisor::step(t); }
    static Driver *dInstance;
  };
}  // namespace webots

#endif  // DRIVER_HPP
