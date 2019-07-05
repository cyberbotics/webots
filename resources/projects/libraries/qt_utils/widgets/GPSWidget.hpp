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
    explicit GPSWidget(Device *device, QWidget *parent = NULL);
    virtual ~GPSWidget() {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    const double *values() override;
  };
}  // namespace webotsQtUtils

#endif
