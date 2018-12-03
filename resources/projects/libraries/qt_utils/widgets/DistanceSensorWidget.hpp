/*
 * Description:  Widget displaying a webots distance sensor device
 */

#ifndef DISTANCE_SENSOR_WIDGET_HPP
#define DISTANCE_SENSOR_WIDGET_HPP

#include "ScalarSensorWidget.hpp"

namespace webotsQtUtils {
  class DistanceSensorWidget : public ScalarSensorWidget {
    Q_OBJECT

  public:
    DistanceSensorWidget(Device *device, QWidget *parent = NULL);
    virtual ~DistanceSensorWidget() {}

  protected slots:
    virtual void enable(bool enable);

  protected:
    virtual bool isEnabled() const;
    virtual double value();
  };
}  // namespace webotsQtUtils

#endif
