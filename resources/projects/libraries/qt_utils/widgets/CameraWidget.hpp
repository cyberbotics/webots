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
    CameraWidget(Device *device, QWidget *parent = NULL);
    virtual ~CameraWidget() {}

    virtual void readSensors();

  protected slots:
    virtual void enable(bool enable);

  protected:
    bool isEnabled() const;

    QLabel *mLabel;
    QHBoxLayout *mHBox;
  };
}  // namespace webotsQtUtils

#endif
