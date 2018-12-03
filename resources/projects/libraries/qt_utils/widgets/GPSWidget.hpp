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
    virtual void enable(bool enable);

  protected:
    virtual bool isEnabled() const;
    virtual const double *values();
  };
}  // namespace webotsQtUtils

#endif
