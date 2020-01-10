// Copyright 1996-2020 Cyberbotics Ltd.
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

/*
 * Description:  Overview tab allowing to have a quick check on the car state
 */

#ifndef GENERAL_INFORMATION_WIDGET_HPP
#define GENERAL_INFORMATION_WIDGET_HPP

#include <QtCore/QString>
#include <QtCore/QTime>
#include <QtWidgets/QWidget>

#include <webots/vehicle/car.h>

class QCheckBox;

class GeneralInformationWidget : public QWidget {
  Q_OBJECT

public:
  explicit GeneralInformationWidget(QWidget *parent = 0);
  virtual ~GeneralInformationWidget();
  void updateInformation();

public slots:
  void updateEnableCheckBoxText();

protected:
  virtual void paintEvent(QPaintEvent *event);
  void drawAxes(QPainter &painter);
  void drawSpeed(QPainter &painter);
  void drawDirectionArray(QPainter &painter);
  void drawTypes(QPainter &painter);
  void drawWheels(QPainter &painter);
  void drawWheelsInformation(QPainter &painter);
  void drawGearboxStateOrTargetSpeed(QPainter &painter);

private:
  void init();

  QCheckBox *mEnableCheckBox;
  QTime mLastRefreshTime;
  bool mIsinitialized;

  // drawing positions
  int mCenterPosition[2];
  int mFrontAxisRightPosition[2];  // right front wheel center
  int mFrontAxisLeftPosition[2];   // left front wheel center
  int mFrontAxisCenterPosition[2];
  int mRearAxisRightPosition[2];  // right rear wheel center
  int mRearAxisLeftPosition[2];   // left rear wheel center
  int mRearAxisCenterPosition[2];

  // fix values
  double mFrontTrack;
  double mRearTrack;
  double mWheelBase;
  double mFrontWheelRadius;
  double mRearWheelRadius;
  int mGearNumber;
  QString mTransmissionType;
  QString mEngineType;

  // variable values
  double mSpeed;
  double mSteeringAngle;
  double mRightSteeringAngle;
  double mLeftSteeringAngle;
  double mThrottle;
  double mBrake;
  double mWheelSpeeds[WBU_CAR_WHEEL_NB];
  double mWheelEncoders[WBU_CAR_WHEEL_NB];
  double mRPM;
  double mTargetSpeed;
  int mGearbox;
  bool mIsTorqueControl;
};

#endif  // GENERAL_INFORMATION_WIDGET_HPP
