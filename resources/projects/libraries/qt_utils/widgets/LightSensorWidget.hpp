/*
 * Description:  Widget displaying a webots light sensor device
 */

#ifndef LIGHT_SENSOR_WIDGET_HPP
#define LIGHT_SENSOR_WIDGET_HPP

#include "ScalarSensorWidget.hpp"

namespace webotsQtUtils {
  class LightSensorWidget : public ScalarSensorWidget {
    Q_OBJECT

  public:
    LightSensorWidget(Device *device, QWidget *parent = NULL);
    virtual ~LightSensorWidget() {}

  protected slots:
    virtual void enable(bool enable);

  protected:
    virtual bool isEnabled() const;
    virtual double value();
  };
}  // namespace webotsQtUtils

#endif
