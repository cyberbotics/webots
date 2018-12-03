/*
 * Description:  Widget displaying a webots touch sensor device (bumper or force type)
 */

#ifndef TOUCH_SENSOR_SCALAR_WIDGET_HPP
#define TOUCH_SENSOR_SCALAR_WIDGET_HPP

#include "ScalarSensorWidget.hpp"

namespace webotsQtUtils {
  class TouchSensorScalarWidget : public ScalarSensorWidget {
    Q_OBJECT

  public:
    TouchSensorScalarWidget(Device *device, QWidget *parent = NULL);
    virtual ~TouchSensorScalarWidget() {}

  protected slots:
    virtual void enable(bool enable);

  protected:
    virtual bool isEnabled() const;
    virtual double value();
  };
}  // namespace webotsQtUtils

#endif
