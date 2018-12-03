#include "TouchSensorScalarWidget.hpp"
#include <devices/Device.hpp>

#include <webots/touch_sensor.h>

using namespace webotsQtUtils;

TouchSensorScalarWidget::TouchSensorScalarWidget(Device *device, QWidget *parent) : ScalarSensorWidget(device, parent) {
}

void TouchSensorScalarWidget::enable(bool enable) {
  WbDeviceTag tag = mDevice->tag();
  if (enable)
    wb_touch_sensor_enable(tag, static_cast<int>(wb_robot_get_basic_time_step()));
  else
    wb_touch_sensor_disable(tag);
}

bool TouchSensorScalarWidget::isEnabled() const {
  WbDeviceTag tag = mDevice->tag();
  return wb_touch_sensor_get_sampling_period(tag) > 0;
}

double TouchSensorScalarWidget::value() {
  WbDeviceTag tag = mDevice->tag();
  return wb_touch_sensor_get_value(tag);
}
