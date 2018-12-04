/*
 * Description:  Abstraction of a sensor returning one floating point value
 */

#ifndef SINGLE_VALUE_SENSOR_HPP
#define SINGLE_VALUE_SENSOR_HPP

#include "Sensor.hpp"

class SingleValueSensor : public Sensor {
public:
  // Device Manager is responsible to create/destroy devices
  SingleValueSensor(WbDeviceTag tag, int index) : Sensor(tag, index), mValue(0.0) {}
  virtual ~SingleValueSensor() {}

  double value() const { return mValue; }
  void setValue(double v) { mValue = v; }

private:
  double mValue;
};

#endif
