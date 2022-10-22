// Copyright 1996-2022 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_ROBOT_HPP
#define WB_ROBOT_HPP

#include "WbMFString.hpp"
#include "WbSFBool.hpp"
#include "WbSFString.hpp"
#include "WbSolid.hpp"
#include "WbVector3.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QList>
#include <QtCore/QVarLengthArray>
#include <QtCore/QVector>

class WbAbstractCamera;
class WbDataStream;
class WbDevice;
class WbJoystickInterface;
class WbKinematicDifferentialWheels;
class WbMFDouble;
class WbMouse;
class WbRenderingDevice;
class WbSensor;
class WbSupervisorUtilities;

class QByteArray;
class QDataStream;
class QTimer;

class WbRobot : public WbSolid {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbRobot(WbTokenizer *tokenizer = NULL);
  WbRobot(const WbRobot &other);
  explicit WbRobot(const WbNode &other);
  virtual ~WbRobot();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_ROBOT; }
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;

  // controller
  void externControllerChanged();
  void newRemoteExternController();
  void removeRemoteExternController();
  bool isControllerExtern() const { return controllerName() == "<extern>"; }
  bool isControllerStarted() const { return mControllerStarted; }
  void startController();
  void setControllerStarted(bool started) { mControllerStarted = started; }
  const QString &controllerDir();
  bool isConfigureDone() const { return !mConfigureRequest; }
  void restartController();
  void setControllerNeedRestart() { mNeedToRestartController = true; }
  bool isWaitingForUserInputEvent() const;
  bool isWaitingForWindow() const { return mWaitingForWindow; }
  void setWaitingForWindow(bool waiting);
  void addNewlyInsertedDevice(WbNode *node);
  void fixMissingResources() const override;

  // path to the project folder containing the proto model
  // returns an empty string if the robot is not a proto node
  QString protoModelProjectPath() const;

  // message dispatching
  virtual void powerOn(bool);
  bool isPowerOn() { return mPowerOn; }
  void dispatchMessage(QDataStream &);
  virtual void handleMessage(QDataStream &);
  virtual void writeAnswer(WbDataStream &);
  virtual bool hasImmediateAnswer() const;
  virtual void writeImmediateAnswer(WbDataStream &);
  void dispatchAnswer(WbDataStream &, bool includeDevices = true);
  void setConfigureRequest(bool b) { mConfigureRequest = b; }

  // device children
  int deviceCount() const { return mDevices.size(); }
  WbDevice *device(int index) const { return mDevices[index]; }
  WbDevice *findDevice(WbDeviceTag tag) const;
  void descendantNodeInserted(WbBaseNode *decendant) override;
  QList<WbRenderingDevice *> renderingDevices() { return mRenderingDevices; }

  // update sensors in case of no answer needs to be written at this step
  virtual void updateSensors();

  void renderCameras();

  // field accessors
  const QString &controllerName() const { return mController->value(); }
  const QStringList &controllerArgs() const { return mControllerArgs->value(); }
  const QString &customData() const { return mCustomData->value(); }
  const QString &window() const { return mWindow->value(); }
  bool synchronization() const { return mSynchronization->value(); }
  bool supervisor() const { return mSupervisor->value(); }
  const WbMFDouble &battery() const { return *mBattery; }
  bool selfCollision() const { return mSelfCollision->value(); }

  WbSupervisorUtilities *supervisorUtilities() const { return mSupervisorUtilities; }

  const bool isRobot() const override { return true; };

  // energy accessors and setters
  double currentEnergy() const;
  void setCurrentEnergy(double e);
  double maxEnergy() const;
  double energyUploadSpeed() const;

  // handle key events
  void keyPressed(int key, int modifiers);
  void keyReleased(int key);

  // map qt special key to webots special key, return 0 if not found
  static int mapSpecialKey(int qtKey);
  // return the absolute file name of the robot window file, if it exists
  QString windowFile(const QString &extension = "html") const;
  void showWindow();  // show the Qt-based controller robot window (to be deprecated)
  void updateControllerWindow();

  void processImmediateMessages();

  void setNeedToWriteUserInputEventAnswer() { mNeedToWriteUserInputEventAnswer = true; }

  WbKinematicDifferentialWheels *kinematicDifferentialWheels() { return mKinematicDifferentialWheels; }

public slots:
  void receiveFromJavascript(const QByteArray &message);
  void updateControllerDir();

signals:
  void startControllerRequest(WbRobot *robot);
  void immediateMessageAdded();
  void controllerChanged();
  void controllerExited();
  void windowChanged();
  void wasReset();
  void toggleRemoteMode(bool enable);
  void sendToJavascript(const QByteArray &);
  void appendMessageToConsole(const QString &message, bool useStdout);
  void userInputEventNeedUpdate();
  void keyboardChanged();
  void windowReady();

protected:
  WbRobot(const QString &modelName, WbTokenizer *tokenizer);

  // reimplemented protected functions
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  virtual void writeConfigure(WbDataStream &);

  // export
  void exportNodeFields(WbWriter &writer) const override;
  const QString urdfName() const override;

  WbKinematicDifferentialWheels *mKinematicDifferentialWheels;

private:
  // user accessible fields
  WbSFString *mController;
  WbMFString *mControllerArgs;
  WbSFString *mCustomData;
  WbSFBool *mSupervisor;
  WbSFBool *mSynchronization;
  WbMFDouble *mBattery;
  WbSFDouble *mCpuConsumption;
  WbSFBool *mSelfCollision;
  WbSFString *mWindow;
  WbSFString *mRemoteControl;

  bool mNeedToWriteUrdf;
  bool mShowWindowCalled;
  bool mShowWindowMessage;
  bool mUpdateWindowMessage;
  bool mWaitingForWindow;
  QByteArray *mMessageFromWwi;
  bool mDataNeedToWriteAnswer;
  bool mSupervisorNeedToWriteAnswer;
  bool mModelNeedToWriteAnswer;
  bool mPowerOn;
  bool mControllerStarted;
  bool mNeedToRestartController;
  bool mConfigureRequest;
  bool mSimulationModeRequested;

  QString mControllerDir;

  double mPreviousTime;

  // supervisor
  bool mSupervisorUtilitiesNeedUpdate;
  WbSupervisorUtilities *mSupervisorUtilities;

  // pin
  bool mPin;
  WbVector3 mPinTranslation;
  WbRotation mPinRotation;

  // dynamic libraries
  QString mAbsoluteWindowFilename;
  QString mAbsoluteRemoteControlFilename;

  // sensors
  WbSensor *mBatterySensor;
  WbSensor *mKeyboardSensor;
  WbSensor *mJoystickSensor;
  double mBatteryLastValue;
  QMap<QString, double> mBatteryInitialValues;
  QList<int> mKeyboardLastValue;
  struct JoyStickLastValue {
    int numberOfPressedButtons;
    QList<int> pressedButtonsIndices;
    int numberOfAxes;
    QList<int> axesValues;
    int numberOfPovs;
    QList<int> povsValues;
  };
  JoyStickLastValue *mJoyStickLastValue;
  WbMouse *mMouse;

  // if sensor refresh is needed, update value and return TRUE
  bool refreshBatterySensorIfNeeded();
  bool refreshKeyboardSensorIfNeeded();
  bool refreshJoyStickSensorIfNeeded();

  // joystick interface
  WbJoystickInterface *mJoystickInterface;
  bool mJoystickConfigureRequest;
  QTimer *mJoystickTimer;

  // user input events
  QTimer *mUserInputEventTimer;
  int mMonitoredUserInputEventTypes;
  QDateTime mUserInputEventReferenceTime;
  bool mNeedToWriteUserInputEventAnswer;
  bool mKeyboardHasChanged;

  // other variables
  QList<WbDevice *> mDevices;
  QList<WbRenderingDevice *> mRenderingDevices;
  QList<WbAbstractCamera *> mActiveCameras;
  QList<WbDevice *> mNewlyAddedDevices;
  int mNextTag;

  QList<int> mPressedKeys;

  WbRobot &operator=(const WbRobot &);  // non copyable
  WbNode *clone() const override { return new WbRobot(*this); }
  void init();
  void addDevices(WbNode *node);
  // if reset is TRUE reassign tags to devices (when device config changed)
  // if reset is FALSE, only tag of newly added devices will be assigned
  void assignDeviceTags(bool reset);
  void writeDeviceConfigure(QList<WbDevice *> devices, WbDataStream &stream) const;
  QString searchDynamicLibraryAbsolutePath(const QString &key, const QString &pluginSubdirectory);
  void updateDevicesAfterInsertion();
  void pinToStaticEnvironment(bool pin);
  double energyConsumption() const;
  void clearDevices();
  int computeSimulationMode();

private slots:
  void updateDevicesAfterDestruction();
  void updateActiveCameras(WbAbstractCamera *camera, bool isActive);
  void updateWindow();
  void updateRemoteControl();
  void updateSimulationMode();
  void updateData();
  void updateSupervisor();
  void updateModel();
  void removeRenderingDevice();
  void handleMouseChange();
  void handleJoystickChange();
};

#endif
