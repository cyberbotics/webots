/*
 * Description:  Abstraction of a Webots robot device
 */

#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <webots/device.h>
#include <webots/robot.h>

#include <QtCore/QString>

namespace webotsQtUtils {
  class Device {
  public:
    explicit Device(WbDeviceTag tag);
    virtual ~Device() {}

    WbDeviceTag tag() const { return mTag; }
    WbNodeType type() const { return mType; }
    const QString &name() const { return mName; }
    const QString &category() const { return mCategory; }

  protected:
    WbDeviceTag mTag;
    WbNodeType mType;
    QString mName;
    QString mCategory;
  };
}  // namespace webotsQtUtils

#endif
