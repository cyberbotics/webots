/*
 * Description:  Widget displaying a webots sensor device
 */

#ifndef SENSOR_WIDGET_HPP
#define SENSOR_WIDGET_HPP

#include "DeviceWidget.hpp"

class QCheckBox;

namespace webotsQtUtils {
  class SensorWidget : public DeviceWidget {
    Q_OBJECT

  public:
    explicit SensorWidget(Device *device, QWidget *parent = NULL);
    virtual ~SensorWidget() {}

    void readSensors() override;

  protected slots:
    virtual void enable(bool enable) {}

  protected:
    virtual bool isEnabled() const = 0;

    QCheckBox *mCheckBox;
  };
}  // namespace webotsQtUtils

#endif
