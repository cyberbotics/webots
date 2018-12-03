#include "TouchSensorVectorialWidget.hpp"
#include <devices/Device.hpp>

#include <webots/touch_sensor.h>

using namespace webotsQtUtils;

TouchSensorVectorialWidget::TouchSensorVectorialWidget(Device *device, QWidget *parent) :
  VectorialSensorWidget(device, parent) {
}

void TouchSensorVectorialWidget::enable(bool enable) {
  WbDeviceTag tag = mDevice->tag();
  if (enable)
    wb_touch_sensor_enable(tag, static_cast<int>(wb_robot_get_basic_time_step()));
  else
    wb_touch_sensor_disable(tag);
}

bool TouchSensorVectorialWidget::isEnabled() const {
  WbDeviceTag tag = mDevice->tag();
  return wb_touch_sensor_get_sampling_period(tag) > 0;
}

const double *TouchSensorVectorialWidget::values() {
  WbDeviceTag tag = mDevice->tag();
  return wb_touch_sensor_get_values(tag);
}
