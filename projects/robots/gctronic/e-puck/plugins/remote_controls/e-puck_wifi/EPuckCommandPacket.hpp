/*
 * Description:  Defines a packet sending from the remote control library to the e-puck
 */

#ifndef EPUCK_COMMAND_PACKET_HPP
#define EPUCK_COMMAND_PACKET_HPP

class Device;

class EPuckCommandPacket {
public:
  EPuckCommandPacket();
  virtual ~EPuckCommandPacket();

  virtual void clear();
  int size() const { return sizeof(mData); }
  const char *data() const { return mData; }
  int apply(int simulationTime);
  bool areDistanceSensorRequested() const { return mDistanceSensorRequested; }
  bool areGroundSensorRequested() const { return mGroundSensorRequested; }
  bool areLightSensorRequested() const { return mLightSensorRequested; }
  bool isAccelerometerRequested() const { return mAccelerometerRequested; }
  bool isEncoderRequested() const { return mEncoderRequested; }
  bool isCameraRequested() const { return ((mData[1] & 1) == 1); }

private:
  bool mDistanceSensorRequested;
  bool mGroundSensorRequested;
  bool mLightSensorRequested;
  bool mAccelerometerRequested;
  bool mEncoderRequested;
  char mData[21];
};

#endif
