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

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Device.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Speaker.hpp>
#include <webots/utils/Motion.hpp>

#include "LinuxDARwIn.h"

#include <libgen.h>
#include <unistd.h>
#include <iostream>

using namespace std;

webots::Robot *webots::Robot::cInstance = NULL;

webots::Robot::Robot() {
  if (cInstance == NULL)
    cInstance = this;
  else {
    cerr << "Only one instance of the Robot class should be created" << endl;
    exit(-1);
  }

  initRobotisOp2();
  initDevices();
  gettimeofday(&mStart, NULL);
  mPreviousStepTime = 0.0;
  mKeyboard = new Keyboard();

  // Load TimeStep from the file "config.ini"
  minIni ini("config.ini");
  LoadINISettings(&ini, "Robot Config");
  if (mTimeStep < 16) {
    cout << "The time step selected of " << mTimeStep << "ms is very small and will probably not be respected." << endl;
    cout << "A time step of at least 16ms is recommended." << endl;
  }

  mCM730->MakeBulkReadPacketWb();  // Create the BulkReadPacket to read the actuators states in Robot::step

  // Unactive all Joints in the Motion Manager //
  std::map<const std::string, int>::iterator motorIt;
  for (motorIt = Motor::mNamesToIDs.begin(); motorIt != Motor::mNamesToIDs.end(); ++motorIt) {
    ::Robot::MotionStatus::m_CurrentJoints.SetEnable((*motorIt).second, 0);
    ::Robot::MotionStatus::m_CurrentJoints.SetValue((*motorIt).second,
                                                    (dynamic_cast<Motor *>(mDevices[(*motorIt).first]))->getGoalPosition());
  }

  // Make each motors go to the start position slowly
  const int msgLength = 5;  // id + Goal Position (L + H) + Moving speed (L + H)
  int value, changed_motors = 0, n = 0;
  int param[20 * msgLength];

  for (motorIt = Motor::mNamesToIDs.begin(); motorIt != Motor::mNamesToIDs.end(); ++motorIt) {
    Motor *motor = static_cast<Motor *>(mDevices[(*motorIt).first]);
    int motorId = (*motorIt).second;
    if (motor->getTorqueEnable() && !(::Robot::MotionStatus::m_CurrentJoints.GetEnable(motorId))) {
      param[n++] = motorId;              // id
      value = motor->getGoalPosition();  // Start position
      param[n++] = ::Robot::CM730::GetLowByte(value);
      param[n++] = ::Robot::CM730::GetHighByte(value);
      value = 100;  // small speed 100 => 11.4 rpm => 1.2 rad/s
      param[n++] = ::Robot::CM730::GetLowByte(value);
      param[n++] = ::Robot::CM730::GetHighByte(value);
      changed_motors++;
    }
  }
  mCM730->SyncWrite(::Robot::MX28::P_GOAL_POSITION_L, msgLength, changed_motors, param);
  usleep(2000000);  // wait a moment to reach start position

  // Switch LED to GREEN
  mCM730->WriteWord(::Robot::CM730::ID_CM, ::Robot::CM730::P_LED_HEAD_L, 1984, 0);
  mCM730->WriteWord(::Robot::CM730::ID_CM, ::Robot::CM730::P_LED_EYE_L, 1984, 0);
}

webots::Robot::~Robot() {
}

int webots::Robot::step(int duration) {
  // play motions if any
  Motion::playMotions();

  double actualTime = getTime() * 1000;
  int stepDuration = actualTime - mPreviousStepTime;
  std::map<const std::string, int>::iterator motorIt;
  std::map<const std::string, int>::iterator position_sensorIt;

  // -------- Update speed of each motors, according to acceleration limit if set --------  //
  for (motorIt = Motor::mNamesToIDs.begin(); motorIt != Motor::mNamesToIDs.end(); ++motorIt) {
    Motor *motor = static_cast<Motor *>(mDevices[(*motorIt).first]);
    motor->updateSpeed(stepDuration);
  }

  // -------- Bulk Read to read the actuators states (position, speed and load) and body sensors -------- //
  if (!(::Robot::MotionManager::GetInstance()->GetEnable()))  // If MotionManager is enable, no need to execute the BulkRead,
                                                              // the MotionManager has allready done it.
    mCM730->BulkRead();

  // Motors
  for (motorIt = Motor::mNamesToIDs.begin(); motorIt != Motor::mNamesToIDs.end(); ++motorIt) {
    Motor *motor = static_cast<Motor *>(mDevices[(*motorIt).first]);
    int motorId = (*motorIt).second;
    motor->setPresentSpeed(mCM730->m_BulkReadData[motorId].ReadWord(::Robot::MX28::P_PRESENT_SPEED_L));
    motor->setPresentLoad(mCM730->m_BulkReadData[motorId].ReadWord(::Robot::MX28::P_PRESENT_LOAD_L));

    int alarmShutdownControlTableValue = mCM730->m_BulkReadData[motorId].ReadWord(::Robot::MX28::P_ALARM_SHUTDOWN);
    if (alarmShutdownControlTableValue != 0) {
      cerr << "Alarm detected on id = " << motorId << " with value = " << alarmShutdownControlTableValue << endl;
      exit(EXIT_FAILURE);
    }
  }

  // Position sensors
  for (position_sensorIt = PositionSensor::mNamesToIDs.begin(); position_sensorIt != PositionSensor::mNamesToIDs.end();
       ++position_sensorIt) {
    PositionSensor *position_sensor = static_cast<PositionSensor *>(mDevices[(*position_sensorIt).first]);
    int position_sensorId = (*position_sensorIt).second;
    position_sensor->setPresentPosition(
      mCM730->m_BulkReadData[position_sensorId].ReadWord(::Robot::MX28::P_PRESENT_POSITION_L));
  }

  int values[3];

  // Gyro
  values[0] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_GYRO_X_L);
  values[1] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_GYRO_Y_L);
  values[2] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_GYRO_Z_L);
  (dynamic_cast<Gyro *>(mDevices["Gyro"]))->setValues(values);

  // Accelerometer
  values[0] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_ACCEL_X_L);
  values[1] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_ACCEL_Y_L);
  values[2] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_ACCEL_Z_L);
  (dynamic_cast<Accelerometer *>(mDevices["Accelerometer"]))->setValues(values);
  // Led states
  values[0] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_LED_HEAD_L);
  values[1] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_LED_EYE_L);
  values[2] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadByte(::Robot::CM730::P_LED_PANNEL);
  (dynamic_cast<LED *>(mDevices["HeadLed"]))->setColor(values[0]);
  (dynamic_cast<LED *>(mDevices["EyeLed"]))->setColor(values[1]);
  LED::setBackPanel(values[2]);

  // push button state (TODO: check with real robot that the masks are correct)
  // values[0] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_BUTTON) & 0x1;
  // values[1] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_BUTTON) & 0x2;
  // values[2] = mCM730->m_BulkReadData[::Robot::CM730::ID_CM].ReadWord(::Robot::CM730::P_BUTTON) & 0x4;

  // -------- Sync Write to actuators --------  //
  const int msgLength = 9;  // id + P + Empty + Goal Position (L + H) + Moving speed (L + H) + Torque Limit (L + H)

  int param[20 * msgLength];
  int n = 0;
  int changed_motors = 0;
  int value;

  for (motorIt = Motor::mNamesToIDs.begin(); motorIt != Motor::mNamesToIDs.end(); ++motorIt) {
    Motor *motor = static_cast<Motor *>(mDevices[(*motorIt).first]);
    int motorId = (*motorIt).second;
    if (motor->getTorqueEnable() && !(::Robot::MotionStatus::m_CurrentJoints.GetEnable(motorId))) {
      param[n++] = motorId;
      param[n++] = motor->getPGain();
      param[n++] = 0;  // Empty
      // TODO: controlPID should be implemented there
      value = motor->getGoalPosition();
      param[n++] = ::Robot::CM730::GetLowByte(value);
      param[n++] = ::Robot::CM730::GetHighByte(value);
      value = motor->getMovingSpeed();
      param[n++] = ::Robot::CM730::GetLowByte(value);
      param[n++] = ::Robot::CM730::GetHighByte(value);
      value = motor->getTorqueLimit();
      param[n++] = ::Robot::CM730::GetLowByte(value);
      param[n++] = ::Robot::CM730::GetHighByte(value);
      changed_motors++;
    }
  }
  mCM730->SyncWrite(::Robot::MX28::P_P_GAIN, msgLength, changed_motors, param);

  // -------- Keyboard Reset ----------- //
  mKeyboard->resetKeyboard();

  // -------- Timing management -------- //
  if (stepDuration < duration) {  // Step to short -> wait remaining time
    usleep((duration - stepDuration) * 1000);
    mPreviousStepTime = actualTime;
    return 0;
  } else {  // Step to long -> return step duration
    mPreviousStepTime = actualTime;
    return stepDuration;
  }
}

std::string webots::Robot::getName() const {
  return "robotis-op2";
}

double webots::Robot::getTime() const {
  struct timeval end;

  gettimeofday(&end, NULL);

  long seconds = end.tv_sec - mStart.tv_sec;
  long useconds = end.tv_usec - mStart.tv_usec;
  long mtime = (seconds * 1000 + useconds / 1000.0) + 0.5;

  return (double)mtime / 1000.0;
}

int webots::Robot::getMode() const {
  return 1;
}

double webots::Robot::getBasicTimeStep() const {
  return mTimeStep;
}

webots::Device *webots::Robot::getDevice(const std::string &name) const {
  std::map<const std::string, webots::Device *>::const_iterator it = mDevices.find(name);
  if (it != mDevices.end())
    return (*it).second;
  return NULL;
}

webots::Accelerometer *webots::Robot::getAccelerometer(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::Accelerometer *accelerometer = dynamic_cast<webots::Accelerometer *>(device);
    if (accelerometer)
      return accelerometer;
  }
  return NULL;
}

webots::Camera *webots::Robot::getCamera(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::Camera *camera = dynamic_cast<webots::Camera *>(device);
    if (camera)
      return camera;
  }
  return NULL;
}

webots::Gyro *webots::Robot::getGyro(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::Gyro *gyro = dynamic_cast<webots::Gyro *>(device);
    if (gyro)
      return gyro;
  }
  return NULL;
}

webots::Motor *webots::Robot::getMotor(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::Motor *motor = dynamic_cast<webots::Motor *>(device);
    if (motor)
      return motor;
  }
  return NULL;
}

webots::PositionSensor *webots::Robot::getPositionSensor(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::PositionSensor *position_sensor = dynamic_cast<webots::PositionSensor *>(device);
    if (position_sensor)
      return position_sensor;
  }
  return NULL;
}

webots::LED *webots::Robot::getLED(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::LED *led = dynamic_cast<webots::LED *>(device);
    if (led)
      return led;
  }
  return NULL;
}

webots::Speaker *webots::Robot::getSpeaker(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::Speaker *speaker = dynamic_cast<webots::Speaker *>(device);
    if (speaker)
      return speaker;
  }
  return NULL;
}

void webots::Robot::initDevices() {
  mDevices["Accelerometer"] = new webots::Accelerometer("Accelerometer");
  mDevices["Camera"] = new webots::Camera("Camera");
  mDevices["Gyro"] = new webots::Gyro("Gyro");
  mDevices["EyeLed"] = new webots::LED("EyeLed");
  mDevices["HeadLed"] = new webots::LED("HeadLed");
  mDevices["BackLedRed"] = new webots::LED("BackLedRed");
  mDevices["BackLedGreen"] = new webots::LED("BackLedGreen");
  mDevices["BackLedBlue"] = new webots::LED("BackLedBlue");
  mDevices["ShoulderR"] = new webots::Motor("ShoulderR");
  mDevices["ShoulderL"] = new webots::Motor("ShoulderL");
  mDevices["ArmUpperR"] = new webots::Motor("ArmUpperR");
  mDevices["ArmUpperL"] = new webots::Motor("ArmUpperL");
  mDevices["ArmLowerR"] = new webots::Motor("ArmLowerR");
  mDevices["ArmLowerL"] = new webots::Motor("ArmLowerL");
  mDevices["PelvYR"] = new webots::Motor("PelvYR");
  mDevices["PelvYL"] = new webots::Motor("PelvYL");
  mDevices["PelvR"] = new webots::Motor("PelvR");
  mDevices["PelvL"] = new webots::Motor("PelvL");
  mDevices["LegUpperR"] = new webots::Motor("LegUpperR");
  mDevices["LegUpperL"] = new webots::Motor("LegUpperL");
  mDevices["LegLowerR"] = new webots::Motor("LegLowerR");
  mDevices["LegLowerL"] = new webots::Motor("LegLowerL");
  mDevices["AnkleR"] = new webots::Motor("AnkleR");
  mDevices["AnkleL"] = new webots::Motor("AnkleL");
  mDevices["FootR"] = new webots::Motor("FootR");
  mDevices["FootL"] = new webots::Motor("FootL");
  mDevices["Neck"] = new webots::Motor("Neck");
  mDevices["Head"] = new webots::Motor("Head");
  mDevices["ShoulderRS"] = new webots::PositionSensor("ShoulderRS");
  mDevices["ShoulderLS"] = new webots::PositionSensor("ShoulderLS");
  mDevices["ArmUpperRS"] = new webots::PositionSensor("ArmUpperRS");
  mDevices["ArmUpperLS"] = new webots::PositionSensor("ArmUpperLS");
  mDevices["ArmLowerRS"] = new webots::PositionSensor("ArmLowerRS");
  mDevices["ArmLowerLS"] = new webots::PositionSensor("ArmLowerLS");
  mDevices["PelvYRS"] = new webots::PositionSensor("PelvYRS");
  mDevices["PelvYLS"] = new webots::PositionSensor("PelvYLS");
  mDevices["PelvRS"] = new webots::PositionSensor("PelvRS");
  mDevices["PelvLS"] = new webots::PositionSensor("PelvLS");
  mDevices["LegUpperRS"] = new webots::PositionSensor("LegUpperRS");
  mDevices["LegUpperLS"] = new webots::PositionSensor("LegUpperLS");
  mDevices["LegLowerRS"] = new webots::PositionSensor("LegLowerRS");
  mDevices["LegLowerLS"] = new webots::PositionSensor("LegLowerLS");
  mDevices["AnkleRS"] = new webots::PositionSensor("AnkleRS");
  mDevices["AnkleLS"] = new webots::PositionSensor("AnkleLS");
  mDevices["FootRS"] = new webots::PositionSensor("FootRS");
  mDevices["FootLS"] = new webots::PositionSensor("FootLS");
  mDevices["NeckS"] = new webots::PositionSensor("NeckS");
  mDevices["HeadS"] = new webots::PositionSensor("HeadS");
  mDevices["Speaker"] = new webots::Speaker("Speaker");
}

void webots::Robot::initRobotisOp2() {
  char exepath[1024] = "";
  if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1) {
    if (chdir(dirname(exepath)))
      cerr << "chdir error" << endl;
  }

  mLinuxCM730 = new ::Robot::LinuxCM730("/dev/ttyUSB0");
  mCM730 = new ::Robot::CM730(mLinuxCM730);

  if (mCM730->Connect() == false) {
    cerr << "Fail to connect CM-730" << endl;
    exit(EXIT_FAILURE);
  }

  // Read firmware version
  int firm_ver = 0;
  if (mCM730->ReadByte(::Robot::JointData::ID_HEAD_PAN, ::Robot::MX28::P_VERSION, &firm_ver, 0) != ::Robot::CM730::SUCCESS) {
    cerr << "Can't read firmware version from Dynamixel ID " << ::Robot::JointData::ID_HEAD_PAN << endl;
    exit(EXIT_FAILURE);
  }

  if (firm_ver < 27) {
    cerr << "Firmware version of Dynamixel is too old, please update them." << endl;
    cerr << "More information available in the User guide." << endl;
    exit(EXIT_FAILURE);
  }

  ::Robot::MotionManager::GetInstance()->Initialize(mCM730);

  // Switch LED to RED
  mCM730->WriteWord(::Robot::CM730::ID_CM, ::Robot::CM730::P_LED_HEAD_L, 63, 0);
  mCM730->WriteWord(::Robot::CM730::ID_CM, ::Robot::CM730::P_LED_EYE_L, 63, 0);
}

void webots::Robot::LoadINISettings(minIni *ini, const std::string &section) {
  double value = INVALID_VALUE;

  if ((value = ini->getd(section, "time_step", INVALID_VALUE)) != INVALID_VALUE)
    mTimeStep = value;
  else
    cout << "Can't read time step from 'config.ini'" << endl;

  if ((value = ini->getd(section, "camera_width", INVALID_VALUE)) != INVALID_VALUE)
    ::Robot::Camera::WIDTH = value;
  else
    cout << "Can't read camera width from 'config.ini'" << endl;

  if ((value = ini->getd(section, "camera_height", INVALID_VALUE)) != INVALID_VALUE)
    ::Robot::Camera::HEIGHT = value;
  else
    cout << "Can't read camera height from 'config.ini'" << endl;

  if (!(::webots::Camera::checkResolution(::Robot::Camera::WIDTH, ::Robot::Camera::HEIGHT))) {
    cerr << "The resolution of " << ::Robot::Camera::WIDTH << "x" << ::Robot::Camera::HEIGHT
         << " selected is not supported by the camera.\nPlease use one of the resolution recommended." << endl;
    ::Robot::Camera::WIDTH = 320;
    ::Robot::Camera::HEIGHT = 240;
    cout << "WARNING : Camera resolution reset to 320x240 pixels." << endl;
  }
}
