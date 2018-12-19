/*
 * Description:  Widget displaying a webots gps sensor device
 */

#ifndef GPS_WIDGET_HPP
#define GPS_WIDGET_HPP

#include "VectorialSensorWidget.hpp"

namespace webotsQtUtils {
  class GPSWidget : public VectorialSensorWidget {
    Q_OBJECT

  public:
    GPSWidget(Device *device, QWidget *parent = NULL);
    virtual ~GPSWidget() {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    virtual const double *values();
  };
}  // namespace webotsQtUtils

#endif
