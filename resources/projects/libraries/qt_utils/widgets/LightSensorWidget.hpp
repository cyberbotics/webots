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
    explicit LightSensorWidget(Device *device, QWidget *parent = NULL);
    virtual ~LightSensorWidget() {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    double value() override;
  };
}  // namespace webotsQtUtils

#endif
