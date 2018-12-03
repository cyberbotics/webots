/*
 * Description:  Widget displaying a webots compass sensor device
 */

#ifndef COMPASS_WIDGET_HPP
#define COMPASS_WIDGET_HPP

#include "VectorialSensorWidget.hpp"

namespace webotsQtUtils {
  class CompassWidget : public VectorialSensorWidget {
    Q_OBJECT

  public:
    CompassWidget(Device *device, QWidget *parent = NULL);
    virtual ~CompassWidget() {}

  protected slots:
    virtual void enable(bool enable);

  protected:
    virtual bool isEnabled() const;
    virtual const double *values();
  };
}  // namespace webotsQtUtils

#endif
