/*
 * Description:  Widget displaying a webots inertial unit device
 */

#ifndef INERTIAL_UNIT_WIDGET_HPP
#define INERTIAL_UNIT_WIDGET_HPP

#include "VectorialSensorWidget.hpp"

namespace webotsQtUtils {
  class InertialUnitWidget : public VectorialSensorWidget {
    Q_OBJECT

  public:
    InertialUnitWidget(Device *device, QWidget *parent = NULL);
    virtual ~InertialUnitWidget() {}

  protected slots:
    virtual void enable(bool enable);

  protected:
    virtual bool isEnabled() const;
    virtual const double *values();
  };
}  // namespace webotsQtUtils

#endif
