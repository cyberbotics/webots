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
    explicit DistanceSensorWidget(Device *device, QWidget *parent = NULL);
    virtual ~DistanceSensorWidget() {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    double value() override;
  };
}  // namespace webotsQtUtils

#endif
