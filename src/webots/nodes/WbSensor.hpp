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

#ifndef WB_SENSOR_HPP
#define WB_SENSOR_HPP

#include <QtCore/QObject>

//
// Description: a sensor placed on a device
//
class WbRobot;
class WbSensor : public QObject {
  Q_OBJECT
public:
  WbSensor();
  virtual ~WbSensor() {}

  // refresh rate
  int refreshRate() { return mRefreshRate; }
  void setRefreshRate(int rate);
  bool isEnabled() const { return mRefreshRate > 0; }
  bool isFirstValueReady() const { return mIsFirstValueReady; }

  // status of controller sensor value update
  // sensor value could be updated some steps before the value is sent to the controller
  bool hasPendingValue() const { return mHasPendingValue; }
  void resetPendingValue() { mHasPendingValue = false; }

  // update timer to reset elapsed time
  void updateTimer();

  // check if refresh needed based on elapsed time
  bool needToRefresh();
  // check if refresh will be needed in 'ms' millisecons
  bool needToRefreshInMs(int ms);

  // return time elapsed since last refresh
  double elapsedTime() const;

  // last refresh
  double lastUpdate() const { return mLastUpdate; }

  // remote control
  bool isRemoteModeEnabled() const { return mIsRemoteMode; }
  void connectToRobotSignal(const WbRobot *robot, bool connectRemoteMode = true);

  void reset();

signals:
  void stateChanged();

private:
  int mRefreshRate;
  double mLastUpdate;
  bool mIsRemoteMode;
  bool mIsFirstValueReady;
  bool mHasPendingValue;

private slots:
  void toggleRemoteMode(bool enabled);
};

#endif
