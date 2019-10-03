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
    explicit TouchSensorScalarWidget(Device *device, QWidget *parent = NULL);
    virtual ~TouchSensorScalarWidget() {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    double value() override;
  };
}  // namespace webotsQtUtils

#endif
