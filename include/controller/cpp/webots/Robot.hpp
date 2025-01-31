// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <iostream>
#include <vector>

#include "../../c/webots/types.h"

namespace webots {
  class Accelerometer;
  class Altimeter;
  class Brake;
  class Camera;
  class Compass;
  class Connector;
  class DistanceSensor;
  class Display;
  class Device;
  class Emitter;
  class GPS;
  class Gyro;
  class InertialUnit;
  class Joystick;
  class Keyboard;
  class LED;
  class Lidar;
  class LightSensor;
  class Motor;
  class Mouse;
  class Pen;
  class PositionSensor;
  class Radar;
  class RangeFinder;
  class Receiver;
  class Skin;
  class Speaker;
  class TouchSensor;
  class VacuumGripper;

  class Robot {
  public:
    typedef enum { MODE_SIMULATION = 0, MODE_CROSS_COMPILATION, MODE_REMOTE_CONTROL } Mode;

    typedef enum {
      EVENT_QUIT = -1,
      EVENT_NO_EVENT = 0,
      EVENT_MOUSE_CLICK = 1,
      EVENT_MOUSE_MOVE = 2,
      EVENT_KEYBOARD = 4,
      EVENT_JOYSTICK_BUTTON = 8,
      EVENT_JOYSTICK_AXIS = 16,
      EVENT_JOYSTICK_POV = 32
    } UserInputEvent;

    Robot();
    virtual ~Robot();

    virtual int step(int duration);
    int stepBegin(int duration);
    int stepEnd();
    UserInputEvent waitForUserInputEvent(UserInputEvent event_type, int timeout);
    std::string getName() const;
    std::string getUrdf(std::string prefix = "") const;
    double getTime() const;
    std::string getModel() const;
    std::string getCustomData() const;
    void setCustomData(const std::string &data);
    Mode getMode() const;
    void setMode(Mode, const char *);
    bool getSupervisor() const;
    bool getSynchronization() const;
    std::string getProjectPath() const;
    std::string getWorldPath() const;
    double getBasicTimeStep() const;
    int getNumberOfDevices() const;
    Device *getDeviceByIndex(int index);
    Device *getDevice(const std::string &name);

    virtual void batterySensorEnable(int samplingPeriod);
    virtual void batterySensorDisable();
    int batterySensorGetSamplingPeriod() const;
    double batterySensorGetValue() const;

    Accelerometer *getAccelerometer(const std::string &name);
    Altimeter *getAltimeter(const std::string &name);
    Brake *getBrake(const std::string &name);
    Camera *getCamera(const std::string &name);
    Compass *getCompass(const std::string &name);
    Connector *getConnector(const std::string &name);
    Display *getDisplay(const std::string &name);
    DistanceSensor *getDistanceSensor(const std::string &name);
    Emitter *getEmitter(const std::string &name);
    GPS *getGPS(const std::string &name);
    Gyro *getGyro(const std::string &name);
    InertialUnit *getInertialUnit(const std::string &name);
    Joystick *getJoystick() { return mJoystick; }
    Keyboard *getKeyboard() { return mKeyboard; }
    LED *getLED(const std::string &name);
    Lidar *getLidar(const std::string &name);
    LightSensor *getLightSensor(const std::string &name);
    Motor *getMotor(const std::string &name);
    Mouse *getMouse() { return mMouse; }
    Pen *getPen(const std::string &name);
    PositionSensor *getPositionSensor(const std::string &name);
    Radar *getRadar(const std::string &name);
    RangeFinder *getRangeFinder(const std::string &name);
    Receiver *getReceiver(const std::string &name);
    Skin *getSkin(const std::string &name);
    Speaker *getSpeaker(const std::string &name);
    TouchSensor *getTouchSensor(const std::string &name);
    VacuumGripper *getVacuumGripper(const std::string &name);

    void *windowCustomFunction(void *arg);
    void wwiSend(const char *data, int size);
    void wwiSendText(const std::string &text);
    const char *wwiReceive(int *size);
    std::string wwiReceiveText();

    // deprecated since Webots 2018a
    std::string getData() const;
    void setData(const std::string &data);

    // internal functions
    static Device *getDeviceFromTag(int tag);
    static int getDeviceTypeFromTag(int tag);
    static std::string getDeviceNameFromTag(int tag);
    static int getDeviceTagFromIndex(int index);
    static int getDeviceTagFromName(const std::string &name);

  protected:
    static Robot *cInstance;
    virtual Accelerometer *createAccelerometer(WbDeviceTag tag) const;
    virtual Altimeter *createAltimeter(WbDeviceTag tag) const;
    virtual Brake *createBrake(WbDeviceTag tag) const;
    virtual Camera *createCamera(WbDeviceTag tag) const;
    virtual Compass *createCompass(WbDeviceTag tag) const;
    virtual Connector *createConnector(WbDeviceTag tag) const;
    virtual Display *createDisplay(WbDeviceTag tag) const;
    virtual DistanceSensor *createDistanceSensor(WbDeviceTag tag) const;
    virtual Emitter *createEmitter(WbDeviceTag tag) const;
    virtual GPS *createGPS(WbDeviceTag tag) const;
    virtual Gyro *createGyro(WbDeviceTag tag) const;
    virtual InertialUnit *createInertialUnit(WbDeviceTag tag) const;
    virtual LED *createLED(WbDeviceTag tag) const;
    virtual Lidar *createLidar(WbDeviceTag tag) const;
    virtual LightSensor *createLightSensor(WbDeviceTag tag) const;
    virtual Motor *createMotor(WbDeviceTag tag) const;
    virtual Pen *createPen(WbDeviceTag tag) const;
    virtual PositionSensor *createPositionSensor(WbDeviceTag tag) const;
    virtual Radar *createRadar(WbDeviceTag tag) const;
    virtual RangeFinder *createRangeFinder(WbDeviceTag tag) const;
    virtual Receiver *createReceiver(WbDeviceTag tag) const;
    virtual Skin *createSkin(WbDeviceTag tag) const;
    virtual Speaker *createSpeaker(WbDeviceTag tag) const;
    virtual TouchSensor *createTouchSensor(WbDeviceTag tag) const;
    virtual VacuumGripper *createVacuumGripper(WbDeviceTag tag) const;

  private:
    Keyboard *mKeyboard;
    Joystick *mJoystick;
    Mouse *mMouse;
    Device *getOrCreateDevice(int tag);
    static std::vector<Device *> deviceList;
  };
}  // namespace webots

#endif  // ROBOT_HPP
