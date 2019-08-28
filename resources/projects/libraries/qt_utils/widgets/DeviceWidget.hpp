/*
 * Description:  Widget displaying a webots robot device
 */

#ifndef DEVICE_WIDGET_HPP
#define DEVICE_WIDGET_HPP

class QLabel;
class QVBoxLayout;
class QHBoxLayout;

#include <QtWidgets/QWidget>

namespace webotsQtUtils {
  class Device;

  class DeviceWidget : public QWidget {
  public:
    explicit DeviceWidget(Device *device, QWidget *parent = NULL);
    virtual ~DeviceWidget() {}

    virtual void readSensors() {}
    virtual void writeActuators() {}

  protected:
    void setTitleSuffix(const QString &suffix);

    Device *mDevice;

    QVBoxLayout *mVBoxLayout;
    QHBoxLayout *mTitleLayout;
    QWidget *mMainWidget;
    QWidget *mTitleWidget;
    QLabel *mTitleLabel;
  };
}  // namespace webotsQtUtils

#endif
