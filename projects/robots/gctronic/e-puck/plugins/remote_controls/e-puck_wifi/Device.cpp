#include "Device.hpp"

#include <webots/device.h>

Device::Device(WbDeviceTag tag, int index) : mTag(tag), mIndex(index) {
  mName = wb_device_get_name(tag);
  mType = wb_device_get_node_type(tag);
}
