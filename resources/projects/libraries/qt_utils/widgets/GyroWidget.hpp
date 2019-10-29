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
    explicit GyroWidget(Device *device, QWidget *parent = NULL);
    virtual ~GyroWidget() {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    const double *values() override;
  };
}  // namespace webotsQtUtils

#endif
