#include "DeviceWidgetFactory.hpp"

#include <devices/Device.hpp>
#include "DeviceWidget.hpp"

#include "AccelerometerWidget.hpp"
#include "CameraWidget.hpp"
#include "CompassWidget.hpp"
#include "DistanceSensorWidget.hpp"
#include "GPSWidget.hpp"
#include "GyroWidget.hpp"
#include "InertialUnitWidget.hpp"
#include "LidarWidget.hpp"
#include "LightSensorWidget.hpp"
#include "MotorWidget.hpp"
#include "PositionSensorWidget.hpp"
#include "RadarWidget.hpp"
#include "RangeFinderWidget.hpp"
#include "TouchSensorScalarWidget.hpp"
#include "TouchSensorVectorialWidget.hpp"

#include <webots/touch_sensor.h>

using namespace webotsQtUtils;

DeviceWidget *DeviceWidgetFactory::createDeviceWidget(Device *device, QWidget *parent) {
  DeviceWidget *widget = NULL;
  switch (device->type()) {
    case WB_NODE_ACCELEROMETER:
      widget = new AccelerometerWidget(device, parent);
      break;
    case WB_NODE_CAMERA:
      widget = new CameraWidget(device, parent);
      break;
    case WB_NODE_COMPASS:
      widget = new CompassWidget(device, parent);
      break;
    case WB_NODE_DISTANCE_SENSOR:
      widget = new DistanceSensorWidget(device, parent);
      break;
    case WB_NODE_GPS:
      widget = new GPSWidget(device, parent);
      break;
    case WB_NODE_GYRO:
      widget = new GyroWidget(device, parent);
      break;
    case WB_NODE_INERTIAL_UNIT:
      widget = new InertialUnitWidget(device, parent);
      break;
    case WB_NODE_LIDAR:
      widget = new LidarWidget(device, parent);
      break;
    case WB_NODE_LIGHT_SENSOR:
      widget = new LightSensorWidget(device, parent);
      break;
    case WB_NODE_ROTATIONAL_MOTOR:
    case WB_NODE_LINEAR_MOTOR: {
      MotorWidget *motorWidget = new MotorWidget(device, parent);
      motorWidget->setupBounds();
      widget = motorWidget;
      break;
    }
    case WB_NODE_POSITION_SENSOR:
      widget = new PositionSensorWidget(device, parent);
      break;
    case WB_NODE_RADAR:
      widget = new RadarWidget(device, parent);
      break;
    case WB_NODE_RANGE_FINDER:
      widget = new RangeFinderWidget(device, parent);
      break;
    case WB_NODE_TOUCH_SENSOR:
      if (wb_touch_sensor_get_type(device->tag()) == WB_TOUCH_SENSOR_FORCE3D)
        widget = new TouchSensorVectorialWidget(device, parent);
      else
        widget = new TouchSensorScalarWidget(device, parent);
      break;
    default:
      return NULL;
  }

  return widget;
}
