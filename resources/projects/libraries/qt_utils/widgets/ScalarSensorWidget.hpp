/*
 * Description:  Widget displaying a webots scalar sensor device
 */

#ifndef SCALAR_SENSOR_WIDGET_HPP
#define SCALAR_SENSOR_WIDGET_HPP

#include "SensorWidget.hpp"

class QHBoxLayout;

namespace webotsQtUtils {
  class Graph2D;

  class ScalarSensorWidget : public SensorWidget {
  public:
    explicit ScalarSensorWidget(Device *device, QWidget *parent = NULL);
    virtual ~ScalarSensorWidget() {}

    void readSensors() override;

  protected:
    virtual double value() = 0;

    Graph2D *mGraph2D;
    QHBoxLayout *mHBoxLayout;
  };
}  // namespace webotsQtUtils

#endif
