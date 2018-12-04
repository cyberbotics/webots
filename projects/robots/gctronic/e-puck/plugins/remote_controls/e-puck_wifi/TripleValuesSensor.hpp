/*
 * Description:  Abstraction of a sensor returning three floating point value
 */

#ifndef TRIPLE_VALUES_SENSOR_HPP
#define TRIPLE_VALUES_SENSOR_HPP

#include "Sensor.hpp"

class TripleValuesSensor : public Sensor {
public:
  // Device Manager is responsible to create/destroy devices
  TripleValuesSensor(WbDeviceTag tag, int index) : Sensor(tag, index) {
    mValues[0] = 0.0;
    mValues[1] = 0.0;
    mValues[2] = 0.0;
  }
  virtual ~TripleValuesSensor() {}

  const double *values() const { return mValues; }
  void setValues(double v0, double v1, double v2) {
    mValues[0] = v0;
    mValues[1] = v1;
    mValues[2] = v2;
  }

private:
  double mValues[3];
};

#endif
