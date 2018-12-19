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
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    virtual const double *values();
  };
}  // namespace webotsQtUtils

#endif
