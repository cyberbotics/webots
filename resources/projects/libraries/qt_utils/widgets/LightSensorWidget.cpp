#include "LightSensorWidget.hpp"
#include <devices/Device.hpp>

#include <webots/light_sensor.h>

using namespace webotsQtUtils;

LightSensorWidget::LightSensorWidget(Device *device, QWidget *parent) : ScalarSensorWidget(device, parent) {
}

void LightSensorWidget::enable(bool enable) {
  WbDeviceTag tag = mDevice->tag();
  if (enable)
    wb_light_sensor_enable(tag, static_cast<int>(wb_robot_get_basic_time_step()));
  else
    wb_light_sensor_disable(tag);
}

bool LightSensorWidget::isEnabled() const {
  WbDeviceTag tag = mDevice->tag();
  return wb_light_sensor_get_sampling_period(tag) > 0;
}

double LightSensorWidget::value() {
  WbDeviceTag tag = mDevice->tag();
  return wb_light_sensor_get_value(tag);
}
