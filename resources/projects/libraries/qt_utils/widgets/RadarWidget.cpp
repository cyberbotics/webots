#include "RadarWidget.hpp"
#include <devices/Device.hpp>

#include "../graph2d/Graph2D.hpp"
#include "../graph2d/Line2D.hpp"
#include "../graph2d/Point2D.hpp"

#include <QtGui/QColor>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>

#include <webots/radar.h>

#include <math.h>

using namespace webotsQtUtils;

RadarWidget::RadarWidget(Device *device, QWidget *parent) : SensorWidget(device, parent) {
  mTargetNumberLabel = new QLabel(this);
  mTargetNumberLabel->setText(tr("Number of targets: "));
  mTitleLayout->addWidget(mTargetNumberLabel);

  mHBox = new QHBoxLayout(mMainWidget);
  mGraph2D = new Graph2D(mMainWidget);

  mHBox->addWidget(mGraph2D, 0);
  mMainWidget->setLayout(mHBox);
}

void RadarWidget::readSensors() {
  SensorWidget::readSensors();

  WbDeviceTag tag = mDevice->tag();
  if (wb_radar_get_sampling_period(tag) > 0) {
    int numberOftargets = wb_radar_get_number_of_targets(tag);
    mTargetNumberLabel->setText(tr("Number of targets: ") + QString::number(numberOftargets));
    mGraph2D->clear();
    double maxRange = wb_radar_get_max_range(tag);
    double horizontalFov = wb_radar_get_horizontal_fov(tag);
    mGraph2D->setXRange(-maxRange, maxRange);
    if (horizontalFov > M_PI)
      mGraph2D->setYRange(-maxRange * 1.2, maxRange * 1.2);
    else
      mGraph2D->setYRange(-maxRange * 0.2, maxRange * 1.2);
    mGraph2D->addLine2D(Line2D(0.0, 0.0, maxRange * sin(horizontalFov / 2), maxRange * cos(horizontalFov / 2), Qt::blue));
    mGraph2D->addLine2D(Line2D(0.0, 0.0, maxRange * sin(-horizontalFov / 2), maxRange * cos(-horizontalFov / 2), Qt::blue));
    for (int i = 0; i < 16; ++i)
      mGraph2D->addLine2D(Line2D(maxRange * sin(horizontalFov / 2 - i * horizontalFov / 16),
                                 maxRange * cos(horizontalFov / 2 - i * horizontalFov / 16),
                                 maxRange * sin(horizontalFov / 2 - (i + 1) * horizontalFov / 16),
                                 maxRange * cos(horizontalFov / 2 - (i + 1) * horizontalFov / 16), Qt::blue));
    const WbRadarTarget *targets = wb_radar_get_targets(tag);
    for (int i = 0; i < numberOftargets; ++i) {
      double y = targets[i].distance * cos(targets[i].azimuth);
      double x = targets[i].distance * sin(targets[i].azimuth);
      mGraph2D->addPoint2D(Point2D(x, y, Qt::red));
    }
  }
}

void RadarWidget::enable(bool enable) {
  WbDeviceTag tag = mDevice->tag();
  if (enable)
    wb_radar_enable(tag, static_cast<int>(wb_robot_get_basic_time_step()));
  else
    wb_radar_disable(tag);
}

bool RadarWidget::isEnabled() const {
  WbDeviceTag tag = mDevice->tag();
  return wb_radar_get_sampling_period(tag) > 0;
}
