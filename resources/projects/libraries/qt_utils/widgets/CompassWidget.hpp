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
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    virtual const double *values();
  };
}  // namespace webotsQtUtils

#endif
