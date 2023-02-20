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

/*******************************************************************************************************/
/* Description:  Wrapper of the Robot Webots API for the ROBOTIS OP2 real robot                          */
/*******************************************************************************************************/

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <sys/time.h>
#include <map>
#include <string>

#include <minIni.h>

namespace Robot {
  class CM730;
  class LinuxCM730;
}  // namespace Robot

namespace webots {
  class Device;
  class Accelerometer;
  class Camera;
  class Gyro;
  class LED;
  class Motor;
  class PositionSensor;
  class Speaker;
  class Keyboard;

  class Robot {
  public:
    Robot();
    virtual ~Robot();

    virtual int step(int duration);
    std::string getName() const;
    double getTime() const;
    int getMode() const;
    double getBasicTimeStep() const;
    Accelerometer *getAccelerometer(const std::string &name) const;
    Camera *getCamera(const std::string &name) const;
    Gyro *getGyro(const std::string &name) const;
    LED *getLED(const std::string &name) const;
    Motor *getMotor(const std::string &name) const;
    PositionSensor *getPositionSensor(const std::string &name) const;
    Speaker *getSpeaker(const std::string &name) const;
    Keyboard *getKeyboard() const { return mKeyboard; }

    // not member(s) of the Webots API function: please don't use
    ::Robot::CM730 *getCM730() const { return mCM730; }
    static Robot *getInstance() { return cInstance; }

  private:
    void initDevices();
    void initRobotisOp2();
    void LoadINISettings(minIni *ini, const std::string &section);
    Device *getDevice(const std::string &name) const;

    static Robot *cInstance;

    std::map<const std::string, Device *> mDevices;

    int mTimeStep;
    Keyboard *mKeyboard;
    ::Robot::LinuxCM730 *mLinuxCM730;
    ::Robot::CM730 *mCM730;
    struct timeval mStart;
    double mPreviousStepTime;
  };
}  // namespace webots

#endif  // ROBOT_HPP
