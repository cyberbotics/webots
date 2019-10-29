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
    explicit AccelerometerWidget(Device *device, QWidget *parent = NULL);
    virtual ~AccelerometerWidget() {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    const double *values() override;
  };
}  // namespace webotsQtUtils

#endif
