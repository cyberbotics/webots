/*
 * Description:  Widget displaying a webots touch sensor device (force3d type)
 */

#ifndef TOUCH_SENSOR_VECTORIAL_WIDGET_HPP
#define TOUCH_SENSOR_VECTORIAL_WIDGET_HPP

#include "VectorialSensorWidget.hpp"

namespace webotsQtUtils {
  class TouchSensorVectorialWidget : public VectorialSensorWidget {
    Q_OBJECT

  public:
    explicit TouchSensorVectorialWidget(Device *device, QWidget *parent = NULL);
    virtual ~TouchSensorVectorialWidget() {}

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;
    const double *values() override;
  };
}  // namespace webotsQtUtils

#endif
