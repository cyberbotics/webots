#include "PositionSensorWidget.hpp"
#include <devices/Device.hpp>

#include <webots/position_sensor.h>

using namespace webotsQtUtils;

PositionSensorWidget::PositionSensorWidget(Device *device, QWidget *parent) : ScalarSensorWidget(device, parent) {
}

void PositionSensorWidget::enable(bool enable) {
  WbDeviceTag tag = mDevice->tag();
  if (enable)
    wb_position_sensor_enable(tag, static_cast<int>(wb_robot_get_basic_time_step()));
  else
    wb_position_sensor_disable(tag);
}

bool PositionSensorWidget::isEnabled() const {
  WbDeviceTag tag = mDevice->tag();
  return wb_position_sensor_get_sampling_period(tag) > 0;
}

double PositionSensorWidget::value() {
  WbDeviceTag tag = mDevice->tag();
  return wb_position_sensor_get_value(tag);
}
