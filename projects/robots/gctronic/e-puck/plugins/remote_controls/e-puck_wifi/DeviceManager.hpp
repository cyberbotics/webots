/*
 * Description:  Class allowing to create or retrieve devices
 */

#ifndef DEVICE_MANAGER_HPP
#define DEVICE_MANAGER_HPP

#include <webots/types.h>

#include <vector>

class Device;
class Camera;
class Motor;
class Led;
class SingleValueSensor;
class TripleValuesSensor;

class DeviceManager {
public:
  static DeviceManager *instance();
  static void cleanup();

  const std::vector<Device *> &devices() const { return mDevices; }
  Device *findDeviceFromTag(WbDeviceTag tag) const;

  Camera *camera() const { return mCamera; }
  Led *led(int at) const { return mLeds[at]; }
  Motor *motor(int at) const { return mMotors[at]; }
  SingleValueSensor *distanceSensor(int at) const { return mDistanceSensors[at]; }
  SingleValueSensor *lightSensor(int at) const { return mLightSensors[at]; }
  SingleValueSensor *groundSensor(int at) const { return mGroundSensors[at]; }
  SingleValueSensor *positionSensor(int at) const { return mPositionSensors[at]; }
  TripleValuesSensor *accelerometer() const { return mAccelerometer; }

  void apply(int simulationTime);

private:
  static DeviceManager *cInstance;

  DeviceManager();
  DeviceManager(const DeviceManager &);             // non constructor-copyable
  DeviceManager &operator=(const DeviceManager &);  // non copyable
  virtual ~DeviceManager();

  void clear();

  std::vector<Device *> mDevices;
  Camera *mCamera;
  Led *mLeds[10];
  Motor *mMotors[2];
  SingleValueSensor *mDistanceSensors[8];
  SingleValueSensor *mLightSensors[8];
  SingleValueSensor *mGroundSensors[3];
  SingleValueSensor *mPositionSensors[2];
  TripleValuesSensor *mAccelerometer;
};

#endif
