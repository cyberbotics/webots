#include "GyroWidget.hpp"
#include <devices/Device.hpp>

#include <webots/gyro.h>

using namespace webotsQtUtils;

GyroWidget::GyroWidget(Device *device, QWidget *parent) : VectorialSensorWidget(device, parent) {
}

void GyroWidget::enable(bool enable) {
  WbDeviceTag tag = mDevice->tag();
  if (enable)
    wb_gyro_enable(tag, static_cast<int>(wb_robot_get_basic_time_step()));
  else
    wb_gyro_disable(tag);
}

bool GyroWidget::isEnabled() const {
  WbDeviceTag tag = mDevice->tag();
  return wb_gyro_get_sampling_period(tag) > 0;
}

const double *GyroWidget::values() {
  WbDeviceTag tag = mDevice->tag();
  return wb_gyro_get_values(tag);
}
