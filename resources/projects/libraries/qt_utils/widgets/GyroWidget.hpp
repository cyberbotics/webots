/*
 * Description:  Widget displaying a webots gyro sensor device
 */

#ifndef GYRO_WIDGET_HPP
#define GYRO_WIDGET_HPP

#include "VectorialSensorWidget.hpp"

namespace webotsQtUtils {
  class GyroWidget : public VectorialSensorWidget {
    Q_OBJECT

  public:
    GyroWidget(Device *device, QWidget *parent = NULL);
    virtual ~GyroWidget() {}

  protected slots:
    virtual void enable(bool enable);

  protected:
    virtual bool isEnabled() const;
    virtual const double *values();
  };
}  // namespace webotsQtUtils

#endif
