#include "Device.hpp"

#include <stdio.h>
using namespace webotsQtUtils;

Device::Device(WbDeviceTag tag) : mTag(tag), mType(WB_NODE_NO_NODE), mName(""), mCategory("Unknown") {
  if (tag) {
    mType = wb_device_get_node_type(tag);
    mName = QString::fromUtf8(wb_device_get_name(tag));

    switch (mType) {
      case WB_NODE_ACCELEROMETER:
        mCategory = "Accelerometer";
        break;
      case WB_NODE_CAMERA:
        mCategory = "Camera";
        break;
      case WB_NODE_COMPASS:
        mCategory = "Compass";
        break;
      case WB_NODE_DISTANCE_SENSOR:
        mCategory = "DistanceSensor";
        break;
      case WB_NODE_GPS:
        mCategory = "GPS";
        break;
      case WB_NODE_GYRO:
        mCategory = "Gyro";
        break;
      case WB_NODE_INERTIAL_UNIT:
        mCategory = "InertialUnit";
        break;
      case WB_NODE_LIDAR:
        mCategory = "Lidar";
        break;
      case WB_NODE_LIGHT_SENSOR:
        mCategory = "LightSensor";
        break;
      case WB_NODE_LINEAR_MOTOR:
      case WB_NODE_ROTATIONAL_MOTOR:
        mCategory = "Motor";
        break;
      case WB_NODE_POSITION_SENSOR:
        mCategory = "PositionSensor";
        break;
      case WB_NODE_RADAR:
        mCategory = "Radar";
        break;
      case WB_NODE_RANGE_FINDER:
        mCategory = "RangeFinder";
        break;
      case WB_NODE_TOUCH_SENSOR:
        mCategory = "TouchSensor";
        break;
      default:
        break;
    }
  } else {  // robot
    mType = WB_NODE_ROBOT;
    mName = QString::fromUtf8(wb_robot_get_name());
  }
}
