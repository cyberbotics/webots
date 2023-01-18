// Copyright 1996-2023 Cyberbotics Ltd.
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

// Description:   Remote serveur for ROBOTIS OP2 remote-control

#ifndef REMOTE_HPP
#define REMOTE_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace webots {
  class Motor;
  class PositionSensor;
  class LED;
  class Camera;
  class Accelerometer;
  class Gyro;

  class Remote : public Robot {
  public:
    Remote();
    virtual ~Remote();

    void remoteStep();

    const double *getRemoteAccelerometer() const;
    const double *getRemoteGyro() const;
    const unsigned char *getRemoteImage() const;
    double getRemotePositionSensor(int index);
    double getRemoteMotorTorque(int index);
    double getRemoteTime() const;

    void setRemoteLED(int index, int value);
    void setRemoteMotorPosition(int index, int value);
    void setRemoteMotorVelocity(int index, int value);
    void setRemoteMotorAcceleration(int index, int value);
    void setRemoteMotorAvailableTorque(int index, int value);
    void setRemoteMotorTorque(int index, int value);
    void setRemoteMotorControlPID(int index, int p, int i, int d);

  private:
    void wait(int ms);
    void myStep();

    int mTimeStep;

    Motor *mMotors[NMOTORS];
    PositionSensor *mPositionSensor[NMOTORS];
    LED *mEyeLed;
    LED *mHeadLed;
    LED *mBackLedRed;
    LED *mBackLedGreen;
    LED *mBackLedBlue;
    Camera *mCamera;
    Accelerometer *mAccelerometer;
    Gyro *mGyro;
  };
};  // namespace webots

#endif
