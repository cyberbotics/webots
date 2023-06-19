// Copyright 1996-2023 Cyberbotics Ltd.
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

#define WB_ALLOW_MIXING_C_AND_CPP_API

#include <webots/device.h>
#include <webots/plugins/robot_window/robot_window.h>
#include <webots/plugins/robot_window/robot_wwi.h>
#include <webots/robot.h>
#include <webots/Device.hpp>
#include <webots/Robot.hpp>

#include <webots/Accelerometer.hpp>
#include <webots/Altimeter.hpp>
#include <webots/Brake.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/Connector.hpp>
#include <webots/Device.hpp>
#include <webots/Display.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Joystick.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Lidar.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Mouse.hpp>
#include <webots/Pen.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Radar.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Receiver.hpp>
#include <webots/Skin.hpp>
#include <webots/Speaker.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/VacuumGripper.hpp>

#include <cassert>
#include <cstdlib>
#include <list>

using namespace std;
using namespace webots;

Robot *Robot::cInstance = NULL;
std::vector<Device *> Robot::deviceList = std::vector<Device *>();

Robot::Robot() {
  if (cInstance == NULL)
    cInstance = this;
  else {
    cerr << "Only one instance of the Robot class should be created" << endl;
    wb_robot_step(0);
    wb_robot_cleanup();
    exit(-1);
  }
  wb_robot_init();
  mKeyboard = new Keyboard();
  mJoystick = new Joystick();
  mMouse = new Mouse();
}

Robot::~Robot() {
  for (size_t i = 0; i < deviceList.size(); ++i)
    delete deviceList[i];
  deviceList.clear();
  delete mKeyboard;
  delete mJoystick;
  delete mMouse;
  wb_robot_cleanup();
}

int Robot::step(int duration) {
  return wb_robot_step(duration);
}

int Robot::stepBegin(int duration) {
  return wb_robot_step_begin(duration);
}

int Robot::stepEnd() {
  return wb_robot_step_end();
}

Robot::UserInputEvent Robot::waitForUserInputEvent(UserInputEvent event_type, int timeout) {
  return UserInputEvent(wb_robot_wait_for_user_input_event(WbUserInputEvent(event_type), timeout));
}

string Robot::getName() const {
  return string(wb_robot_get_name());
}

string Robot::getUrdf(string prefix) const {
  return string(wb_robot_get_urdf(prefix.c_str()));
}

double Robot::getTime() const {
  return wb_robot_get_time();
}

string Robot::getModel() const {
  return string(wb_robot_get_model());
}

string Robot::getCustomData() const {
  return string(wb_robot_get_custom_data());
}

void Robot::setCustomData(const std::string &data) {
  wb_robot_set_custom_data(data.c_str());
}

string Robot::getData() const {
  fprintf(stderr, "Robot::getData is deprecated, please use Robot::getCustomData instead\n");
  return string(wb_robot_get_custom_data());
}

void Robot::setData(const std::string &data) {
  fprintf(stderr, "Robot::setData is deprecated, please use Robot::setCustomData instead\n");
  wb_robot_set_custom_data(data.c_str());
}

Robot::Mode Robot::getMode() const {
  return Mode(wb_robot_get_mode());
}

void Robot::setMode(Mode m, const char *arg) {
  wb_robot_set_mode(WbRobotMode(m), arg);
}

bool Robot::getSupervisor() const {
  return wb_robot_get_supervisor();
}

bool Robot::getSynchronization() const {
  return wb_robot_get_synchronization();
}

string Robot::getProjectPath() const {
  return string(wb_robot_get_project_path());
}

string Robot::getWorldPath() const {
  return string(wb_robot_get_world_path());
}

double Robot::getBasicTimeStep() const {
  return wb_robot_get_basic_time_step();
}

int Robot::getNumberOfDevices() const {
  return wb_robot_get_number_of_devices();
}

Device *Robot::getDeviceByIndex(int index) {
  return getOrCreateDevice(wb_robot_get_device_by_index(index));
}

Device *Robot::getDevice(const std::string &name) {
  return getOrCreateDevice(wb_robot_get_device(name.c_str()));
}

void Robot::batterySensorEnable(int sampling_period) {
  wb_robot_battery_sensor_enable(sampling_period);
}

void Robot::batterySensorDisable() {
  wb_robot_battery_sensor_disable();
}

int Robot::batterySensorGetSamplingPeriod() const {
  return wb_robot_battery_sensor_get_sampling_period();
}

double Robot::batterySensorGetValue() const {
  return wb_robot_battery_sensor_get_value();
}

Accelerometer *Robot::getAccelerometer(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_ACCELEROMETER))
    return NULL;
  return dynamic_cast<Accelerometer *>(getOrCreateDevice(tag));
}

Accelerometer *Robot::createAccelerometer(const string &name) const {
  return new Accelerometer(name);
}

Altimeter *Robot::getAltimeter(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_ALTIMETER))
    return NULL;
  return dynamic_cast<Altimeter *>(getOrCreateDevice(tag));
}

Altimeter *Robot::createAltimeter(const string &name) const {
  return new Altimeter(name);
}

Brake *Robot::getBrake(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_BRAKE))
    return NULL;
  return dynamic_cast<Brake *>(getOrCreateDevice(tag));
}

Brake *Robot::createBrake(const string &name) const {
  return new Brake(name);
}

Camera *Robot::getCamera(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_CAMERA))
    return NULL;
  return dynamic_cast<Camera *>(getOrCreateDevice(tag));
}

Camera *Robot::createCamera(const string &name) const {
  return new Camera(name);
}

Compass *Robot::getCompass(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_COMPASS))
    return NULL;
  return dynamic_cast<Compass *>(getOrCreateDevice(tag));
}

Compass *Robot::createCompass(const string &name) const {
  return new Compass(name);
}

Connector *Robot::getConnector(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_CONNECTOR))
    return NULL;
  return dynamic_cast<Connector *>(getOrCreateDevice(tag));
}

Connector *Robot::createConnector(const string &name) const {
  return new Connector(name);
}

Display *Robot::getDisplay(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_DISPLAY))
    return NULL;
  return dynamic_cast<Display *>(getOrCreateDevice(tag));
}

Display *Robot::createDisplay(const string &name) const {
  return new Display(name);
}

DistanceSensor *Robot::getDistanceSensor(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_DISTANCE_SENSOR))
    return NULL;
  return dynamic_cast<DistanceSensor *>(getOrCreateDevice(tag));
}

DistanceSensor *Robot::createDistanceSensor(const string &name) const {
  return new DistanceSensor(name);
}

Emitter *Robot::getEmitter(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_EMITTER))
    return NULL;
  return dynamic_cast<Emitter *>(getOrCreateDevice(tag));
}

Emitter *Robot::createEmitter(const string &name) const {
  return new Emitter(name);
}

GPS *Robot::getGPS(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_GPS))
    return NULL;
  return dynamic_cast<GPS *>(getOrCreateDevice(tag));
}

GPS *Robot::createGPS(const string &name) const {
  return new GPS(name);
}

Gyro *Robot::getGyro(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_GYRO))
    return NULL;
  return dynamic_cast<Gyro *>(getOrCreateDevice(tag));
}

Gyro *Robot::createGyro(const string &name) const {
  return new Gyro(name);
}

InertialUnit *Robot::getInertialUnit(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_INERTIAL_UNIT))
    return NULL;
  return dynamic_cast<InertialUnit *>(getOrCreateDevice(tag));
}

InertialUnit *Robot::createInertialUnit(const string &name) const {
  return new InertialUnit(name);
}

LED *Robot::getLED(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_LED))
    return NULL;
  return dynamic_cast<LED *>(getOrCreateDevice(tag));
}

LED *Robot::createLED(const string &name) const {
  return new LED(name);
}

Lidar *Robot::getLidar(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_LIDAR))
    return NULL;
  return dynamic_cast<Lidar *>(getOrCreateDevice(tag));
}

Lidar *Robot::createLidar(const string &name) const {
  return new Lidar(name);
}

LightSensor *Robot::getLightSensor(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_LIGHT_SENSOR))
    return NULL;
  return dynamic_cast<LightSensor *>(getOrCreateDevice(tag));
}

LightSensor *Robot::createLightSensor(const string &name) const {
  return new LightSensor(name);
}

Motor *Robot::getMotor(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_LINEAR_MOTOR) && !Device::hasType(tag, WB_NODE_ROTATIONAL_MOTOR))
    return NULL;
  return dynamic_cast<Motor *>(getOrCreateDevice(tag));
}

Motor *Robot::createMotor(const string &name) const {
  return new Motor(name);
}

Pen *Robot::getPen(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_PEN))
    return NULL;
  return dynamic_cast<Pen *>(getOrCreateDevice(tag));
}

Pen *Robot::createPen(const string &name) const {
  return new Pen(name);
}

PositionSensor *Robot::getPositionSensor(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_POSITION_SENSOR))
    return NULL;
  return dynamic_cast<PositionSensor *>(getOrCreateDevice(tag));
}

PositionSensor *Robot::createPositionSensor(const string &name) const {
  return new PositionSensor(name);
}

Radar *Robot::getRadar(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_RADAR))
    return NULL;
  return dynamic_cast<Radar *>(getOrCreateDevice(tag));
}

Radar *Robot::createRadar(const string &name) const {
  return new Radar(name);
}

RangeFinder *Robot::getRangeFinder(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_RANGE_FINDER))
    return NULL;
  return dynamic_cast<RangeFinder *>(getOrCreateDevice(tag));
}

RangeFinder *Robot::createRangeFinder(const string &name) const {
  return new RangeFinder(name);
}

Receiver *Robot::getReceiver(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_RECEIVER))
    return NULL;
  return dynamic_cast<Receiver *>(getOrCreateDevice(tag));
}

Receiver *Robot::createReceiver(const string &name) const {
  return new Receiver(name);
}

Skin *Robot::getSkin(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_SKIN))
    return NULL;
  return dynamic_cast<Skin *>(getOrCreateDevice(tag));
}

Skin *Robot::createSkin(const string &name) const {
  return new Skin(name);
}

Speaker *Robot::getSpeaker(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_SPEAKER))
    return NULL;
  return dynamic_cast<Speaker *>(getOrCreateDevice(tag));
}

Speaker *Robot::createSpeaker(const string &name) const {
  return new Speaker(name);
}

TouchSensor *Robot::getTouchSensor(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_TOUCH_SENSOR))
    return NULL;
  return dynamic_cast<TouchSensor *>(getOrCreateDevice(tag));
}

TouchSensor *Robot::createTouchSensor(const string &name) const {
  return new TouchSensor(name);
}

VacuumGripper *Robot::getVacuumGripper(const string &name) {
  WbDeviceTag tag = wb_robot_get_device(name.c_str());
  if (!Device::hasType(tag, WB_NODE_VACUUM_GRIPPER))
    return NULL;
  return dynamic_cast<VacuumGripper *>(getOrCreateDevice(tag));
}

VacuumGripper *Robot::createVacuumGripper(const string &name) const {
  return new VacuumGripper(name);
}

Device *Robot::getDeviceFromTag(int tag) {
  if (tag == 0)
    return NULL;

  int size = (int)deviceList.size();
  if (size == 0 || size <= tag)
    return NULL;
  return deviceList[tag];
}

Device *Robot::getOrCreateDevice(int tag) {
  if (tag == 0)
    return NULL;

  const int count = wb_robot_get_number_of_devices();
  const int size = (int)deviceList.size();
  // if new devices have been added, then count is greater than devices.length
  // deleted devices are not removed from the C API list and don't affect the number of devices
  if (size == count + 1 && size > 0 && tag < size)
    return deviceList[tag];

  // (re-)initialize deviceList
  if (tag > count)
    return NULL;
  deviceList.resize(count + 1);
  deviceList[0] = NULL;
  for (int i = 0; i < count; i++) {
    WbDeviceTag otherTag = wb_robot_get_device_by_index(i);
    const char *name = wb_device_get_name(otherTag);
    assert(otherTag <= count);
    switch (wb_device_get_node_type(otherTag)) {
      case WB_NODE_ACCELEROMETER:
        deviceList[otherTag] = createAccelerometer(name);
        break;
      case WB_NODE_ALTIMETER:
        deviceList[otherTag] = createAltimeter(name);
        break;
      case WB_NODE_BRAKE:
        deviceList[otherTag] = createBrake(name);
        break;
      case WB_NODE_CAMERA:
        deviceList[otherTag] = createCamera(name);
        break;
      case WB_NODE_COMPASS:
        deviceList[otherTag] = createCompass(name);
        break;
      case WB_NODE_CONNECTOR:
        deviceList[otherTag] = createConnector(name);
        break;
      case WB_NODE_DISPLAY:
        deviceList[otherTag] = createDisplay(name);
        break;
      case WB_NODE_DISTANCE_SENSOR:
        deviceList[otherTag] = createDistanceSensor(name);
        break;
      case WB_NODE_EMITTER:
        deviceList[otherTag] = createEmitter(name);
        break;
      case WB_NODE_GPS:
        deviceList[otherTag] = createGPS(name);
        break;
      case WB_NODE_GYRO:
        deviceList[otherTag] = createGyro(name);
        break;
      case WB_NODE_INERTIAL_UNIT:
        deviceList[otherTag] = createInertialUnit(name);
        break;
      case WB_NODE_LED:
        deviceList[otherTag] = createLED(name);
        break;
      case WB_NODE_LIDAR:
        deviceList[otherTag] = createLidar(name);
        break;
      case WB_NODE_LIGHT_SENSOR:
        deviceList[otherTag] = createLightSensor(name);
        break;
      case WB_NODE_LINEAR_MOTOR:
      case WB_NODE_ROTATIONAL_MOTOR:
        deviceList[otherTag] = createMotor(name);
        break;
      case WB_NODE_PEN:
        deviceList[otherTag] = createPen(name);
        break;
      case WB_NODE_POSITION_SENSOR:
        deviceList[otherTag] = createPositionSensor(name);
        break;
      case WB_NODE_RADAR:
        deviceList[otherTag] = createRadar(name);
        break;
      case WB_NODE_RANGE_FINDER:
        deviceList[otherTag] = createRangeFinder(name);
        break;
      case WB_NODE_RECEIVER:
        deviceList[otherTag] = createReceiver(name);
        break;
      case WB_NODE_SKIN:
        deviceList[otherTag] = createSkin(name);
        break;
      case WB_NODE_SPEAKER:
        deviceList[otherTag] = createSpeaker(name);
        break;
      case WB_NODE_TOUCH_SENSOR:
        deviceList[otherTag] = createTouchSensor(name);
        break;
      case WB_NODE_VACUUM_GRIPPER:
        deviceList[otherTag] = createVacuumGripper(name);
        break;
      default:
        deviceList[otherTag] = NULL;
        break;
    }
  }
  return deviceList[tag];
}

int Robot::getDeviceTypeFromTag(int tag) {
  return wb_device_get_node_type(tag);
}

std::string Robot::getDeviceNameFromTag(int tag) {
  return wb_device_get_name(tag);
}

int Robot::getDeviceTagFromIndex(int index) {
  return wb_robot_get_device_by_index(index);
}

int Robot::getDeviceTagFromName(const std::string &name) {
  return wb_robot_get_device(name.c_str());
}

void *Robot::windowCustomFunction(void *arg) {
  return wb_robot_window_custom_function(arg);
}

void Robot::wwiSend(const char *data, int size) {
  wb_robot_wwi_send(data, size);
}

const char *Robot::wwiReceive(int *size) {
  return wb_robot_wwi_receive(size);
}

string Robot::wwiReceiveText() {
  const char *text = wb_robot_wwi_receive_text();
  if (text)
    return string(text);
  else
    return string();
}

void Robot::wwiSendText(const string &text) {
  wb_robot_wwi_send(text.c_str(), (int)text.length() + 1);
}
