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
/* Description:  Wrapper of the Motor Webots API for the ROBOTIS OP2 real robot                          */
/*******************************************************************************************************/

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <map>
#include <webots/Device.hpp>
#include <webots/Robot.hpp>

namespace webots {
  class Motor : public Device {
  public:
    enum { ROTATIONAL = 0 };

    Motor(const std::string &name);  // Use Robot::getMotor() instead
    virtual ~Motor();

    virtual void setPosition(double position);
    double getTargetPosition() const;
    virtual void setVelocity(double vel);
    virtual void setAcceleration(double acceleration);
    virtual void setAvailableTorque(double availableTorque);
    virtual void setControlPID(double p, double i, double d);
    double getMinPosition() const;
    double getMaxPosition() const;

    // note: *Force* functions are not implemented because ROBOTIS OP2 is controlled in torques

    // torque feedback API is useless on the real robot:
    // indeed, the torque feedback is available at each step no matter the sampling period
    virtual void enableTorqueFeedback(int samplingPeriod);
    virtual void disableTorqueFeedback();
    int getTorqueFeedbackSamplingPeriod() const;
    double getTorqueFeedback() const;

    virtual void setTorque(double torque);

    int getType() const;

    // functions not implemented in the regular Webots API
    void updateSpeed(int duration);

  private:
    static void initStaticMap();

    static std::map<const std::string, int> mNamesToIDs;
    static std::map<const std::string, int> mNamesToLimUp;
    static std::map<const std::string, int> mNamesToLimDown;
    static std::map<const std::string, int> mNamesToInitPos;

    int getGoalPosition() const;
    int getTorqueEnable() const;
    int getPGain() const;
    int getMovingSpeed() const;
    int getTorqueLimit() const;
    double getSpeed() const;

    void setPresentSpeed(int speed);
    void setPresentLoad(int load);

    // For acceleration module //
    double mAcceleration;
    double mActualVelocity;
    double mMaxVelocity;

    // For SynchWrite //
    int mGoalPosition;
    int mTorqueEnable;
    int mPGain;
    int mMovingSpeed;
    int mTorqueLimit;
    int mTorqueFeedback;

    // For Bulk Read //
    int mPresentSpeed;
    int mPresentLoad;

    friend int Robot::step(int duration);
    friend Robot::Robot();
  };
}  // namespace webots

#endif  // MOTOR_HPP
