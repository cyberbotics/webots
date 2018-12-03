#include "ScalarSensorWidget.hpp"
#include <devices/Device.hpp>
#include "CommonProperties.hpp"

#include "../graph2d/Graph2D.hpp"
#include "../graph2d/Point2D.hpp"

#include <QtWidgets/QHBoxLayout>

using namespace webotsQtUtils;

ScalarSensorWidget::ScalarSensorWidget(Device *device, QWidget *parent) : SensorWidget(device, parent) {
  mHBoxLayout = new QHBoxLayout(mMainWidget);
  mGraph2D = new Graph2D(mMainWidget);
  mGraph2D->setUpdateRangeOnClick(false, true);

  mHBoxLayout->addWidget(mGraph2D, 0);
  mMainWidget->setLayout(mHBoxLayout);
}

void ScalarSensorWidget::readSensors() {
  SensorWidget::readSensors();
  if (isEnabled()) {
    double v = value();
    double time = wb_robot_get_time();
    mGraph2D->addPoint2D(Point2D(time, v));
    mGraph2D->keepNPoints(CommonProperties::historySize());
    mGraph2D->updateXRange();
    mGraph2D->extendYRange();
    setTitleSuffix(QString::number(v, 'g', CommonProperties::precision()));
  }
}
