/*
 * Description:  Widget displaying a webots position sensor device
 */

#ifndef POSITION_SENSOR_WIDGET_HPP
#define POSITION_SENSOR_WIDGET_HPP

#include "ScalarSensorWidget.hpp"

namespace webotsQtUtils {
  class PositionSensorWidget : public ScalarSensorWidget {
    Q_OBJECT

  public:
    PositionSensorWidget(Device *device, QWidget *parent = NULL);
    virtual ~PositionSensorWidget() {}

  protected slots:
    virtual void enable(bool enable);

  protected:
    virtual bool isEnabled() const;
    virtual double value();
  };
}  // namespace webotsQtUtils

#endif
