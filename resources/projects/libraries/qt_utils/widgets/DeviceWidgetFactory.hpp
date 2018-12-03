/*
 * Description:  Create the appropriate widget for a given device
 */

#ifndef DEVICE_WIDGET_FACTORY_HPP
#define DEVICE_WIDGET_FACTORY_HPP

class QWidget;

namespace webotsQtUtils {
  class Device;
  class DeviceWidget;

  class DeviceWidgetFactory {
  public:
    static DeviceWidget *createDeviceWidget(Device *device, QWidget *parent = 0);

  private:
    DeviceWidgetFactory();
  };
}  // namespace webotsQtUtils

#endif
