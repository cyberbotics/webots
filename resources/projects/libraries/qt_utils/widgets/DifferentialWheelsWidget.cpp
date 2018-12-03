#include "DifferentialWheelsWidget.hpp"
#include "CommonProperties.hpp"

#include "../graph2d/Graph2D.hpp"
#include "../graph2d/Point2D.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPushButton>

#include <webots/differential_wheels.h>
#include <webots/robot.h>

using namespace webotsQtUtils;

static const QString iconFileNames[9] = {"up_left.png", "up.png",        "up_right.png", "left.png",      "stop.png",
                                         "right.png",   "down_left.png", "down.png",     "down_right.png"};

static const double commandWeights[9][2] = {{0.5, 1.0},  {1.0, 1.0},   {1.0, 0.5},   {-0.5, 0.5}, {0.0, 0.0},
                                            {0.5, -0.5}, {-0.5, -1.0}, {-1.0, -1.0}, {-1.0, -0.5}};

DifferentialWheelsWidget::DifferentialWheelsWidget(Device *device, QWidget *parent) :
  SensorWidget(device, parent),
  mCommandRequest(0) {
  mTargetSpeeds[0] = 0.0;
  mTargetSpeeds[1] = 0.0;

  mButtonWidget = new QWidget(mMainWidget);
  mGridLayout = new QGridLayout(mButtonWidget);
  for (int i = 0; i < 9; i++) {
    mPushButtons[i] = new QPushButton(QIcon("icons:" + iconFileNames[i]), "", mButtonWidget);
    mPushButtons[i]->setIconSize(QSize(25, 25));
    mPushButtons[i]->setFixedSize(QSize(30, 30));
    mGridLayout->addWidget(mPushButtons[i], i / 3, i % 3);
    connect(mPushButtons[i], SIGNAL(pressed()), this, SLOT(sendCommand()));
    connect(mPushButtons[i], SIGNAL(released()), this, SLOT(sendStopCommand()));
  }
  mButtonWidget->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  mButtonWidget->setLayout(mGridLayout);

  mHBoxLayout = new QHBoxLayout(mMainWidget);
  mGraph2D = new Graph2D(mMainWidget);

  mHBoxLayout->addWidget(mGraph2D, 0);
  mHBoxLayout->addWidget(mButtonWidget, 0);
  mMainWidget->setLayout(mHBoxLayout);

  mMaxSpeed = wb_differential_wheels_get_max_speed() / wb_differential_wheels_get_speed_unit();
}

DifferentialWheelsWidget::~DifferentialWheelsWidget() {
}

void DifferentialWheelsWidget::readSensors() {
  SensorWidget::readSensors();
  if (isEnabled()) {
    double leftEncoder = wb_differential_wheels_get_left_encoder();
    double rightEncoder = wb_differential_wheels_get_right_encoder();

    QString suffix = "{";
    suffix += QString::number(leftEncoder, 'g', CommonProperties::precision());
    suffix += ", ";
    suffix += QString::number(rightEncoder, 'g', CommonProperties::precision());
    suffix += "}";
    setTitleSuffix(suffix);

    double time = wb_robot_get_time();
    mGraph2D->addPoint2D(Point2D(time, leftEncoder, Qt::red));
    mGraph2D->addPoint2D(Point2D(time, rightEncoder, Qt::blue));
    mGraph2D->keepNPoints(2 * CommonProperties::historySize());
    mGraph2D->updateXRange();
    mGraph2D->updateYRange();
  }

  if (isAButtonDown())
    sendCommand();
}

void DifferentialWheelsWidget::writeActuators() {
  SensorWidget::writeActuators();
  if (mCommandRequest) {
    wb_differential_wheels_set_speed(mTargetSpeeds[0], mTargetSpeeds[1]);
    mCommandRequest = false;
  }
}

void DifferentialWheelsWidget::enable(bool enable) {
  if (enable)
    wb_differential_wheels_enable_encoders(static_cast<int>(wb_robot_get_basic_time_step()));
  else
    wb_differential_wheels_disable_encoders();
}

void DifferentialWheelsWidget::sendCommand() {
  // retrieve the pushed button
  int i;
  for (i = 0; i < 9; i++) {
    if (mPushButtons[i]->isDown())
      break;
  }
  if (i >= 9)
    return;

  // perform the action
  mTargetSpeeds[0] = mMaxSpeed * commandWeights[i][0];
  mTargetSpeeds[1] = mMaxSpeed * commandWeights[i][1];
  mCommandRequest = true;
}

void DifferentialWheelsWidget::sendStopCommand() {
  mTargetSpeeds[0] = 0.0;
  mTargetSpeeds[1] = 0.0;
  mCommandRequest = true;
}

bool DifferentialWheelsWidget::isEnabled() const {
  return wb_differential_wheels_get_encoders_sampling_period() > 0;
}

bool DifferentialWheelsWidget::isAButtonDown() {
  for (int i = 0; i < 9; i++) {
    if (mPushButtons[i]->isDown())
      return true;
  }
  return false;
}
