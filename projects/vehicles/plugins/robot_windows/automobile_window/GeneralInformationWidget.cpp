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

#include "GeneralInformationWidget.hpp"

#include <QtCore/QtMath>
#include <QtGui/QFont>
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QCheckBox>

#include <webots/vehicle/driver.h>

#define WHEEL_WIDTH 20
#define WHEEL_HEIGHT 50
#define DIRECTION_ARRAY_LENGTH 100

// using namespace webotsQtUtils;
using namespace std;

GeneralInformationWidget::GeneralInformationWidget(QWidget *parent) : QWidget(parent) {
  mEnableCheckBox = new QCheckBox("Disabled", this);
  connect(mEnableCheckBox, &QCheckBox::stateChanged, this, &GeneralInformationWidget::updateEnableCheckBoxText);

  // define drawing position
  int height = parent->height();
  int width = parent->width();

  mCenterPosition[0] = width / 2;
  mCenterPosition[1] = 5 * height / 8;

  mFrontAxisCenterPosition[0] = mCenterPosition[0];
  mRearAxisCenterPosition[0] = mCenterPosition[0];
  mFrontAxisCenterPosition[1] = mCenterPosition[1] - height / 3;
  mRearAxisCenterPosition[1] = mCenterPosition[1] + height / 3;
  mFrontAxisRightPosition[0] = mFrontAxisCenterPosition[0] + width / 6;
  mFrontAxisRightPosition[1] = mFrontAxisCenterPosition[1];
  mFrontAxisLeftPosition[0] = mFrontAxisCenterPosition[0] - width / 6;
  mFrontAxisLeftPosition[1] = mFrontAxisCenterPosition[1];
  mRearAxisRightPosition[0] = mRearAxisCenterPosition[0] + width / 6;
  mRearAxisRightPosition[1] = mRearAxisCenterPosition[1];
  mRearAxisLeftPosition[0] = mRearAxisCenterPosition[0] - width / 6;
  mRearAxisLeftPosition[1] = mRearAxisCenterPosition[1];

  mIsinitialized = false;

  // initialize variable
  mSpeed = 0.0;
  mSteeringAngle = 0.0;
  mRightSteeringAngle = 0.0;
  mLeftSteeringAngle = 0.0;
  mThrottle = 0.0;
  mBrake = 0.0;
  mRPM = 0.0;
  mTargetSpeed = 0.0;
  mGearbox = 0;
  mIsTorqueControl = false;
  for (int i = 0; i < WBU_CAR_WHEEL_NB; ++i) {
    mWheelSpeeds[i] = 0.0;
    mWheelEncoders[i] = 0.0;
  }

  mFrontTrack = 0.0;
  mRearTrack = 0.0;
  mWheelBase = 0.0;
  mFrontWheelRadius = 0.0;
  mRearWheelRadius = 0.0;
  mGearNumber = 0;
  mTransmissionType = QString(tr("unkown"));
  mEngineType = QString(tr("unkown"));
}

GeneralInformationWidget::~GeneralInformationWidget() {
  // cleanup driver if not done by controller
  // (if already done by controller, no problem libdriver can handle this)
  wbu_driver_cleanup();
}

void GeneralInformationWidget::init() {
  // init driver if not done by controller
  // (if already done by controller, no problem libdriver can handle this)
  wbu_driver_init();

  // get fix values
  mFrontTrack = wbu_car_get_track_front();
  mRearTrack = wbu_car_get_track_rear();
  mWheelBase = wbu_car_get_wheelbase();
  mFrontWheelRadius = wbu_car_get_front_wheel_radius();
  mRearWheelRadius = wbu_car_get_rear_wheel_radius();
  mGearNumber = wbu_driver_get_gear_number();

  if (wbu_car_get_type() == WBU_CAR_TRACTION)
    mTransmissionType = QString(tr("traction"));
  else if (wbu_car_get_type() == WBU_CAR_PROPULSION)
    mTransmissionType = QString(tr("propulsion"));
  else if (wbu_car_get_type() == WBU_CAR_FOUR_BY_FOUR)
    mTransmissionType = QString(tr("four by four"));

  if (wbu_car_get_engine_type() == WBU_CAR_COMBUSTION_ENGINE)
    mEngineType = QString(tr("combustion"));
  else if (wbu_car_get_engine_type() == WBU_CAR_ELECTRIC_ENGINE)
    mEngineType = QString(tr("electric"));
  else if (wbu_car_get_engine_type() == WBU_CAR_PARALLEL_HYBRID_ENGINE)
    mEngineType = QString(tr("parallel hybrid"));
  else if (wbu_car_get_engine_type() == WBU_CAR_POWER_SPLIT_HYBRID_ENGINE)
    mEngineType = QString(tr("Epower-split hybrid"));

  mIsinitialized = true;
  mLastRefreshTime.start();
}

void GeneralInformationWidget::updateEnableCheckBoxText() {
  if (mEnableCheckBox->isChecked())
    mEnableCheckBox->setText("Enabled");
  else
    mEnableCheckBox->setText("Disabled");
}

void GeneralInformationWidget::updateInformation() {
  if (!mIsinitialized)
    init();

  if (!mEnableCheckBox->isChecked() || mLastRefreshTime.elapsed() < 100)  // refresh at 10Hz
    return;

  // update variable values
  mSpeed = wbu_driver_get_current_speed();
  mSteeringAngle = wbu_driver_get_steering_angle();
  mRightSteeringAngle = wbu_car_get_right_steering_angle();
  mLeftSteeringAngle = wbu_car_get_left_steering_angle();
  mThrottle = wbu_driver_get_throttle();
  mBrake = wbu_driver_get_brake_intensity();
  if (wbu_driver_get_control_mode() == TORQUE) {
    mRPM = wbu_driver_get_rpm();
    mGearbox = wbu_driver_get_gear();
    mTargetSpeed = 0.0;
    mIsTorqueControl = true;
  } else {
    mRPM = 0.0;
    mGearbox = 0;
    mTargetSpeed = wbu_driver_get_target_cruising_speed();
    mIsTorqueControl = false;
  }
  for (int i = 0; i < WBU_CAR_WHEEL_NB; ++i) {
    mWheelSpeeds[i] = wbu_car_get_wheel_speed(WbuCarWheelIndex(i));
    mWheelEncoders[i] = wbu_car_get_wheel_encoder(WbuCarWheelIndex(i));
  }
  this->repaint();
  mLastRefreshTime.restart();
}

// draw the widget
void GeneralInformationWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  drawAxes(painter);
  drawSpeed(painter);
  drawDirectionArray(painter);
  drawTypes(painter);
  drawWheels(painter);
  drawWheelsInformation(painter);
  drawGearboxStateOrTargetSpeed(painter);
}

// draw the main axes and dimensions
void GeneralInformationWidget::drawAxes(QPainter &painter) {
  QPen pen;

  // draw vehicle axes
  pen.setWidth(4);
  pen.setBrush(Qt::black);
  painter.setPen(pen);
  painter.drawLine(mFrontAxisCenterPosition[0], mFrontAxisCenterPosition[1], mRearAxisCenterPosition[0],
                   mRearAxisCenterPosition[1]);
  painter.drawLine(mFrontAxisRightPosition[0], mFrontAxisRightPosition[1], mFrontAxisLeftPosition[0],
                   mFrontAxisLeftPosition[1]);
  painter.drawLine(mRearAxisRightPosition[0], mRearAxisRightPosition[1], mRearAxisLeftPosition[0], mRearAxisLeftPosition[1]);

  // draw axes dimension
  painter.setFont(QFont("Arial", 10));
  pen.setBrush(Qt::darkGreen);
  painter.setPen(pen);
  painter.drawText(mFrontAxisCenterPosition[0] + 5, mFrontAxisCenterPosition[1] + 12,
                   QString::number(mFrontTrack) + QString(" m"));
  painter.drawText(mRearAxisCenterPosition[0] - 25, mRearAxisCenterPosition[1] + 15,
                   QString::number(mRearTrack) + QString(" m"));

  painter.translate(mCenterPosition[0], mCenterPosition[1]);
  painter.rotate(90);
  painter.drawText(-20, -5, QString::number(mWheelBase) + QString(" m"));
  painter.rotate(-90);
  painter.translate(-mCenterPosition[0], -mCenterPosition[1]);
}

// draw the curent speed
void GeneralInformationWidget::drawSpeed(QPainter &painter) {
  QPen pen;
  pen.setStyle(Qt::DashLine);
  pen.setBrush(Qt::red);
  painter.setPen(pen);
  painter.setFont(QFont("Arial", 20));

  painter.drawText(mCenterPosition[0] - 90, 30, QString("Speed: ") + QString::number(mSpeed) + QString(" km/h"));
}

// draw the direction array
void GeneralInformationWidget::drawDirectionArray(QPainter &painter) {
  QPen pen;
  pen.setStyle(Qt::DashLine);
  pen.setBrush(Qt::red);
  pen.setWidth(2);
  painter.setPen(pen);
  painter.setFont(QFont("Arial", 10));

  painter.translate(mFrontAxisCenterPosition[0], mFrontAxisCenterPosition[1]);
  painter.rotate(qRadiansToDegrees(mSteeringAngle));
  painter.drawLine(0, 0, 0, -DIRECTION_ARRAY_LENGTH);
  painter.drawLine(0, -DIRECTION_ARRAY_LENGTH, DIRECTION_ARRAY_LENGTH / 10, -9 * DIRECTION_ARRAY_LENGTH / 10);
  painter.drawLine(0, -DIRECTION_ARRAY_LENGTH, -DIRECTION_ARRAY_LENGTH / 10, -9 * DIRECTION_ARRAY_LENGTH / 10);
  painter.rotate(-qRadiansToDegrees(mSteeringAngle));
  painter.translate(-mFrontAxisCenterPosition[0], -mFrontAxisCenterPosition[1]);

  painter.drawText(mFrontAxisCenterPosition[0] - 95, mFrontAxisCenterPosition[1] - 8,
                   QString("Steering angle:    ") + QString::number(mSteeringAngle));
}

// draw the gearbox state and RPM or target speed
void GeneralInformationWidget::drawGearboxStateOrTargetSpeed(QPainter &painter) {
  QPen pen;
  pen.setStyle(Qt::DashLine);
  pen.setBrush(Qt::black);
  pen.setWidth(3);
  painter.setPen(pen);
  painter.setFont(QFont("Arial", 10));

  if (mIsTorqueControl) {
    painter.drawText(mCenterPosition[0] - 250, mCenterPosition[1] - 7.5, QString("RPM: ") + QString::number(mRPM, 'g', 3));
    painter.drawText(mCenterPosition[0] - 250, mCenterPosition[1] + 7.5,
                     QString("Engaged gear: ") + QString::number(mGearbox) + QString(" / ") + QString::number(mGearNumber));
  } else
    painter.drawText(mCenterPosition[0] - 250, mCenterPosition[1],
                     QString("Target Speed: ") + QString::number(mTargetSpeed, 'g', 3) + QString(" km/h"));
}

// draw engine and transmission types
void GeneralInformationWidget::drawTypes(QPainter &painter) {
  QPen pen;
  pen.setStyle(Qt::DashLine);
  pen.setBrush(Qt::black);
  pen.setWidth(3);
  painter.setPen(pen);
  painter.setFont(QFont("Arial", 10));

  painter.drawText((mCenterPosition[0] + mFrontAxisRightPosition[0]) / 2, mCenterPosition[1] - 7.5,
                   QString("Engine type: ") + mEngineType);
  painter.drawText((mCenterPosition[0] + mFrontAxisRightPosition[0]) / 2, mCenterPosition[1] + 7.5,
                   QString("Transmission type: ") + mTransmissionType);
}

// draw the wheels
void GeneralInformationWidget::drawWheels(QPainter &painter) {
  QPen pen;
  pen.setWidth(2);
  pen.setBrush(Qt::blue);
  painter.setPen(pen);

  // front right
  painter.translate(mFrontAxisRightPosition[0], mFrontAxisRightPosition[1]);
  painter.rotate(qRadiansToDegrees(mRightSteeringAngle));
  painter.drawRect(-WHEEL_WIDTH / 2, -WHEEL_HEIGHT / 2, WHEEL_WIDTH, WHEEL_HEIGHT);
  painter.rotate(-qRadiansToDegrees(mRightSteeringAngle));
  painter.translate(-mFrontAxisRightPosition[0], -mFrontAxisRightPosition[1]);
  // front left
  painter.translate(mFrontAxisLeftPosition[0], mFrontAxisLeftPosition[1]);
  painter.rotate(qRadiansToDegrees(mLeftSteeringAngle));
  painter.drawRect(-WHEEL_WIDTH / 2, -WHEEL_HEIGHT / 2, WHEEL_WIDTH, WHEEL_HEIGHT);
  painter.rotate(-qRadiansToDegrees(mLeftSteeringAngle));
  painter.translate(-mFrontAxisLeftPosition[0], -mFrontAxisLeftPosition[1]);
  // rear right
  painter.drawRect(mRearAxisRightPosition[0] - WHEEL_WIDTH / 2, mRearAxisRightPosition[1] - WHEEL_HEIGHT / 2, WHEEL_WIDTH,
                   WHEEL_HEIGHT);
  // rear left
  painter.drawRect(mRearAxisLeftPosition[0] - WHEEL_WIDTH / 2, mRearAxisLeftPosition[1] - WHEEL_HEIGHT / 2, WHEEL_WIDTH,
                   WHEEL_HEIGHT);

  // display wheels radius
  painter.setFont(QFont("Arial", 10));
  pen.setBrush(Qt::darkGreen);
  painter.setPen(pen);
  painter.drawText(mRearAxisRightPosition[0] - 25, mRearAxisRightPosition[1] - WHEEL_HEIGHT / 2 - 5,
                   QString("Radius: ") + QString::number(mRearWheelRadius) + QString(" m"));
  painter.drawText(mRearAxisLeftPosition[0] - 25, mRearAxisRightPosition[1] - WHEEL_HEIGHT / 2 - 5,
                   QString("Radius: ") + QString::number(mRearWheelRadius) + QString(" m"));
  painter.drawText(mFrontAxisRightPosition[0] - 25, mFrontAxisRightPosition[1] + WHEEL_HEIGHT / 2 + 15,
                   QString("Radius: ") + QString::number(mFrontWheelRadius) + QString(" m"));
  painter.drawText(mFrontAxisLeftPosition[0] - 25, mFrontAxisRightPosition[1] + WHEEL_HEIGHT / 2 + 15,
                   QString("Radius: ") + QString::number(mFrontWheelRadius) + QString(" m"));
}

void GeneralInformationWidget::drawWheelsInformation(QPainter &painter) {
  QPen pen;
  pen.setBrush(Qt::black);
  painter.setPen(pen);
  painter.setFont(QFont("Arial", 10));

  // front right
  painter.drawText(mFrontAxisRightPosition[0] + 30, mFrontAxisRightPosition[1] - 15,
                   QString("Speed: ") + QString::number(mWheelSpeeds[WBU_CAR_WHEEL_FRONT_RIGHT], 'g', 3) + QString(" rad/s"));
  painter.drawText(mFrontAxisRightPosition[0] + 30, mFrontAxisRightPosition[1],
                   QString("Encoder: ") + QString::number(mWheelEncoders[WBU_CAR_WHEEL_FRONT_RIGHT], 'g', 3) + QString(" rad"));
  painter.drawText(mFrontAxisRightPosition[0] + 30, mFrontAxisRightPosition[1] + 15,
                   QString("Angle: ") + QString::number(mRightSteeringAngle, 'g', 5) + QString(" rad"));
  // rear right
  painter.drawText(mRearAxisRightPosition[0] + 30, mRearAxisRightPosition[1] - 7.5,
                   QString("Speed: ") + QString::number(mWheelSpeeds[WBU_CAR_WHEEL_REAR_RIGHT], 'g', 3) + QString(" rad/s"));
  painter.drawText(mRearAxisRightPosition[0] + 30, mRearAxisRightPosition[1] + 7.5,
                   QString("Encoder: ") + QString::number(mWheelEncoders[WBU_CAR_WHEEL_REAR_RIGHT], 'g', 3) + QString(" rad"));
  // front left
  painter.drawText(mFrontAxisLeftPosition[0] - 150, mFrontAxisLeftPosition[1] - 15,
                   QString("Speed: ") + QString::number(mWheelSpeeds[WBU_CAR_WHEEL_FRONT_LEFT], 'g', 3) + QString(" rad/s"));
  painter.drawText(mFrontAxisLeftPosition[0] - 150, mFrontAxisLeftPosition[1],
                   QString("Encoder: ") + QString::number(mWheelEncoders[WBU_CAR_WHEEL_FRONT_LEFT], 'g', 3) + QString(" rad"));
  painter.drawText(mFrontAxisLeftPosition[0] - 150, mFrontAxisLeftPosition[1] + 15,
                   QString("Angle: ") + QString::number(mLeftSteeringAngle, 'g', 5) + QString(" rad"));
  // rear left
  painter.drawText(mRearAxisLeftPosition[0] - 150, mRearAxisLeftPosition[1] - 7.5,
                   QString("Speed: ") + QString::number(mWheelSpeeds[WBU_CAR_WHEEL_REAR_LEFT], 'g', 3) + QString(" rad/s"));
  painter.drawText(mRearAxisLeftPosition[0] - 150, mRearAxisLeftPosition[1] + 7.5,
                   QString("Encoder: ") + QString::number(mWheelEncoders[WBU_CAR_WHEEL_REAR_LEFT], 'g', 3) + QString(" rad"));
}
