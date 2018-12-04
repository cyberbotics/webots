/*
 * Description:  Abstraction of a Webots device
 */

#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <webots/nodes.h>
#include <webots/types.h>

class Device {
public:
  // Device Manager is responsible to create/destroy devices
  Device(WbDeviceTag tag, int index);
  virtual ~Device() {}

  WbDeviceTag tag() const { return mTag; }

  int index() const { return mIndex; }

private:
  WbDeviceTag mTag;
  WbNodeType mType;
  const char *mName;

  int mIndex;
};

#endif
