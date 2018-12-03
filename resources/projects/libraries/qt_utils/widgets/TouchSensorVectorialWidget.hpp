/*
 * Description:  Widget displaying a webots touch sensor device (force3d type)
 */

#ifndef TOUCH_SENSOR_VECTORIAL_WIDGET_HPP
#define TOUCH_SENSOR_VECTORIAL_WIDGET_HPP

#include "VectorialSensorWidget.hpp"

namespace webotsQtUtils {
  class TouchSensorVectorialWidget : public VectorialSensorWidget {
    Q_OBJECT

  public:
    TouchSensorVectorialWidget(Device *device, QWidget *parent = NULL);
    virtual ~TouchSensorVectorialWidget() {}

  protected slots:
    virtual void enable(bool enable);

  protected:
    virtual bool isEnabled() const;
    virtual const double *values();
  };
}  // namespace webotsQtUtils

#endif
