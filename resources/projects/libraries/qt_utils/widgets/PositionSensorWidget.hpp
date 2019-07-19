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
    explicit PositionSensorWidget(Device *device, QWidget *parent = NULL);
    virtual ~PositionSensorWidget() {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    double value() override;
  };
}  // namespace webotsQtUtils

#endif
