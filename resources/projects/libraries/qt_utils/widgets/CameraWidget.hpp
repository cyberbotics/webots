/*
 * Description:  Widget displaying a webots camera device
 */

#ifndef CAMERA_WIDGET_HPP
#define CAMERA_WIDGET_HPP

#include "SensorWidget.hpp"

class QLabel;
class QHBoxLayout;

namespace webotsQtUtils {
  class CameraWidget : public SensorWidget {
    Q_OBJECT

  public:
    explicit CameraWidget(Device *device, QWidget *parent = NULL);
    virtual ~CameraWidget() {}

    void readSensors() override;

  protected slots:
    void enable(bool enable) override;

  protected:
    bool isEnabled() const override;

    QLabel *mLabel;
    QHBoxLayout *mHBox;
  };
}  // namespace webotsQtUtils

#endif
