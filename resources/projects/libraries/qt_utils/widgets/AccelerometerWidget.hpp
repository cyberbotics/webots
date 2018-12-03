/*
 * Description:  Widget displaying a webots accelerometer sensor device
 */

#ifndef ACCELEROMETER_WIDGET_HPP
#define ACCELEROMETER_WIDGET_HPP

#include "VectorialSensorWidget.hpp"

namespace webotsQtUtils {
  class AccelerometerWidget : public VectorialSensorWidget {
    Q_OBJECT

  public:
    AccelerometerWidget(Device *device, QWidget *parent = NULL);
    virtual ~AccelerometerWidget() {}

  protected slots:
    virtual void enable(bool enable);

  protected:
    virtual bool isEnabled() const;
    virtual const double *values();
  };
}  // namespace webotsQtUtils

#endif
