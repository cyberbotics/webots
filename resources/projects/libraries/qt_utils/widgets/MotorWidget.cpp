#include "MotorWidget.hpp"
#include <devices/Device.hpp>
#include "CommonProperties.hpp"

#include "../graph2d/Graph2D.hpp"
#include "../graph2d/Point2D.hpp"

#include <webots/motor.h>

#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>
#include <QtWidgets/QVBoxLayout>

using namespace webotsQtUtils;

MotorWidget::MotorWidget(Device *device, QWidget *parent) :
  ScalarSensorWidget(device, parent),
  mRangeLevel(0),
  mCommandRequest(false),
  mTargetPosition(0.0) {
  mGraph2D->setUpdateRangeOnClick(false, false);

  mSlider = new QSlider(Qt::Vertical, mMainWidget);
  mHBoxLayout->addWidget(mSlider);

  connect(mSlider, SIGNAL(valueChanged(int)), this, SLOT(sendCommand()));

  mLabel = new QLabel("Red: target position\nBlack: force/torque feedback");
  mVBoxLayout->addWidget(mLabel);
}

double MotorWidget::defaultAbsoluteRange() const {
  if (mDevice->type() == WB_NODE_LINEAR_MOTOR)
    return 1.0;
  else
    return M_PI;
}

void MotorWidget::setupBounds() {
  if (unbounded())
    mGraph2D->setYRange(-defaultAbsoluteRange(), defaultAbsoluteRange());
  else
    mGraph2D->setYRange(minPosition(), maxPosition());
}

void MotorWidget::sendCommand() {
  double min = mGraph2D->yMinRange();
  double max = mGraph2D->yMaxRange();
  double delta = max - min;
  int sliderValue = mSlider->value();

  mTargetPosition = (delta * sliderValue) / 100.0 + min;
  mCommandRequest = true;
}

void MotorWidget::readSensors() {
  SensorWidget::readSensors();  // bypass ScalarSensorWidget::readSensors()
  if (isEnabled()) {
    double v = value();
    double target = targetPosition();
    double currentPosition = value();
    double time = wb_robot_get_time();

    mGraph2D->addPoint2D(Point2D(time, v));
    mGraph2D->addPoint2D(Point2D(time, target, Qt::red));
    mGraph2D->keepNPoints(2 * CommonProperties::historySize());
    mGraph2D->updateXRange();

    if (unbounded()) {
      computeRangeLevel(currentPosition);
      mGraph2D->setYRange(computeMinRange(), computeMaxRange());
    }

    if (!mSlider->isSliderDown())
      setSliderPosition(target);

    setTitleSuffix(QString::number(v, 'g', CommonProperties::precision()));
  }

  if (mSlider->isSliderDown())
    sendCommand();
}

void MotorWidget::writeActuators() {
  ScalarSensorWidget::writeActuators();
  if (mCommandRequest) {
    setPosition(mTargetPosition);
    mCommandRequest = false;
  }
}

void MotorWidget::computeRangeLevel(double position) {
  mRangeLevel = floor(0.5 * floor(position / defaultAbsoluteRange() + 1.0));
}

double MotorWidget::computeMinRange() const {
  if (unbounded())
    return 2.0 * defaultAbsoluteRange() * mRangeLevel - defaultAbsoluteRange();
  else
    return minPosition();
}

double MotorWidget::computeMaxRange() const {
  if (unbounded())
    return 2.0 * defaultAbsoluteRange() * mRangeLevel + defaultAbsoluteRange();
  else
    return maxPosition();
}

void MotorWidget::setSliderPosition(double value) {
  double min = mGraph2D->yMinRange();
  double max = mGraph2D->yMaxRange();
  double delta = max - min;

  mSlider->blockSignals(true);
  if (delta == 0.0)
    mSlider->setSliderPosition(50);
  else
    mSlider->setSliderPosition(100 * ((value - min) / delta));
  mSlider->blockSignals(false);
}

void MotorWidget::enable(bool enable) {
  WbDeviceTag tag = mDevice->tag();
  if (wb_motor_get_type(tag) == WB_ROTATIONAL) {
    if (enable)
      wb_motor_enable_torque_feedback(tag, static_cast<int>(wb_robot_get_basic_time_step()));
    else
      wb_motor_disable_torque_feedback(tag);
  } else {
    if (enable)
      wb_motor_enable_force_feedback(tag, static_cast<int>(wb_robot_get_basic_time_step()));
    else
      wb_motor_disable_force_feedback(tag);
  }
}

bool MotorWidget::isEnabled() const {
  if (wb_motor_get_type(mDevice->tag()) == WB_ROTATIONAL)
    return wb_motor_get_torque_feedback_sampling_period(mDevice->tag()) > 0;
  else
    return wb_motor_get_force_feedback_sampling_period(mDevice->tag()) > 0;
}

double MotorWidget::value() {
  if (wb_motor_get_type(mDevice->tag()) == WB_ROTATIONAL)
    return wb_motor_get_torque_feedback(mDevice->tag());
  else
    return wb_motor_get_force_feedback(mDevice->tag());
}

double MotorWidget::minPosition() const {
  return wb_motor_get_min_position(mDevice->tag());
}

double MotorWidget::maxPosition() const {
  return wb_motor_get_max_position(mDevice->tag());
}

double MotorWidget::targetPosition() const {
  return wb_motor_get_target_position(mDevice->tag());
}

void MotorWidget::setPosition(double position) {
  wb_motor_set_position(mDevice->tag(), position);
}
