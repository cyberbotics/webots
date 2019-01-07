// Copyright 1996-2018 Cyberbotics Ltd.
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

#include "WbRobot.hpp"

#include "WbAbstractCamera.hpp"
#include "WbBinaryIncubator.hpp"
#include "WbControllerPlugin.hpp"
#include "WbDisplay.hpp"
#include "WbJoint.hpp"
#include "WbJoystickInterface.hpp"
#include "WbKinematicDifferentialWheels.hpp"
#include "WbLinearMotor.hpp"
#include "WbLog.hpp"
#include "WbLogicalDevice.hpp"
#include "WbMFDouble.hpp"
#include "WbMFNode.hpp"
#include "WbMouse.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbPropeller.hpp"
#include "WbProtoList.hpp"
#include "WbProtoModel.hpp"
#include "WbRenderingDevice.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSensor.hpp"
#include "WbSimulationState.hpp"
#include "WbSkin.hpp"
#include "WbSlot.hpp"
#include "WbSolidDevice.hpp"
#include "WbStandardPaths.hpp"
#include "WbSupervisorUtilities.hpp"
#include "WbTrack.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include "../../../include/controller/c/webots/keyboard.h"
#include "../../../include/controller/c/webots/robot.h"
#include "../../../include/controller/c/webots/supervisor.h"
#include "../../lib/Controller/api/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QStringList>
#include <QtCore/QTimer>

#include <limits>

static QHash<int, int> createSpecialKeys() {
  QHash<int, int> map;

  // init LUT for special keys
  map[Qt::SHIFT] = WB_KEYBOARD_SHIFT;
  map[Qt::CTRL] = WB_KEYBOARD_CONTROL;
  map[Qt::ALT] = WB_KEYBOARD_ALT;
  map[Qt::Key_Home] = WB_KEYBOARD_HOME;
  map[Qt::Key_End] = WB_KEYBOARD_END;
  map[Qt::Key_Left] = WB_KEYBOARD_LEFT;
  map[Qt::Key_Up] = WB_KEYBOARD_UP;
  map[Qt::Key_Right] = WB_KEYBOARD_RIGHT;
  map[Qt::Key_Down] = WB_KEYBOARD_DOWN;
  map[Qt::Key_PageUp] = WB_KEYBOARD_PAGEUP;
  map[Qt::Key_PageDown] = WB_KEYBOARD_PAGEDOWN;

  return map;
}

// LUT to convert Qt key values to Webots key values
static QHash<int, int> gSpecialKeys = createSpecialKeys();

enum { CURRENT_ENERGY, MAX_ENERGY, ENERGY_UPLOAD_SPEED };

int WbRobot::mapSpecialKey(int qtKey) {
  if (gSpecialKeys.contains(qtKey))
    return gSpecialKeys.value(qtKey);

  return 0;
}

void WbRobot::init() {
  mPowerOn = true;
  mBatterySensor = NULL;
  mBatteryLastValue = -1.0;
  mKeyboardSensor = NULL;
  mJoystickSensor = NULL;
  mJoystickInterface = NULL;
  mJoystickTimer = NULL;
  mUserInputEventTimer = NULL;
  mJoyStickLastValue = NULL;
  mMouse = NULL;

  mControllerStarted = false;
  mConfigureRequest = true;
  mSimulationModeRequested = false;
  mMonitoredUserInputEventTypes = -1;
  mNeedToWriteUserInputEventAnswer = false;
  mKeyboardHasChanged = false;

  mJoystickConfigureRequest = false;

  mPreviousTime = 0.0;
  mKinematicDifferentialWheels = NULL;

  mMessageFromWwi = NULL;
  mShowWindowCalled = false;
  mShowWindowMessage = false;
  mWaitingForWindow = false;
  mUpdateWindowMessage = false;
  mDataNeedToWriteAnswer = false;
  mModelNeedToWriteAnswer = false;

  mPin = false;
  mPinTranslation = WbVector3();
  mPinRotation = WbRotation();

  mSupervisor = findSFBool("supervisor");
  mSynchronization = findSFBool("synchronization");
  mController = findSFString("controller");
  mControllerArgs = findSFString("controllerArgs");
  mCustomData = findSFString("customData");
  mBattery = findMFDouble("battery");
  mCpuConsumption = findSFDouble("cpuConsumption");
  mSelfCollision = findSFBool("selfCollision");
  mShowWindow = findSFBool("showWindow");
  mWindow = findSFString("window");
  mRemoteControl = findSFString("remoteControl");

  WbSFString *data = findSFString("data");
  if (data->value() != "") {  // Introduced in Webots 2018a
    warn("Deprecated 'data' field, please use the 'customData' field instead.");
    if (mCustomData->value() == "")
      mCustomData->setValue(data->value());
    data->setValue("");
  }

  mBatteryInitialValue = (mBattery->size() > CURRENT_ENERGY) ? mBattery->item(CURRENT_ENERGY) : -1.0;
  mSupervisorUtilities = supervisor() ? new WbSupervisorUtilities(this) : NULL;
}

WbRobot::WbRobot(WbTokenizer *tokenizer) : WbSolid("Robot", tokenizer) {
  init();
}

WbRobot::WbRobot(const WbRobot &other) : WbSolid(other) {
  init();
}

WbRobot::WbRobot(const WbNode &other) : WbSolid(other) {
  init();
}

WbRobot::WbRobot(const QString &modelName, WbTokenizer *tokenizer) : WbSolid(modelName, tokenizer) {
  init();
}

WbRobot::~WbRobot() {
  clearDevices();
  delete mBatterySensor;
  delete mKeyboardSensor;
  delete mJoystickSensor;
  delete mJoystickInterface;
  delete mJoystickTimer;
  delete mUserInputEventTimer;
  delete mJoyStickLastValue;
  delete mKinematicDifferentialWheels;
  if (mMouse)
    WbMouse::destroy(mMouse);
  if (mSupervisorUtilities)
    delete mSupervisorUtilities;
  mPressedKeys.clear();
  WbWorld::instance()->removeRobotIfPresent(this);
}

void WbRobot::preFinalize() {
  WbSolid::preFinalize();
  mBatterySensor = new WbSensor();
  mKeyboardSensor = new WbSensor();
  mJoystickSensor = new WbSensor();

  updateWindow();
  updateRemoteControl();
  updateControllerDir();
  updateDevicesAfterInsertion();
  WbWorld::instance()->addRobotIfNotAlreadyPresent(this);
}

void WbRobot::postFinalize() {
  WbSolid::postFinalize();
  mKinematicDifferentialWheels = WbKinematicDifferentialWheels::createKinematicDifferentialWheelsIfNeeded(this);

  connect(mController, &WbSFString::changed, this, &WbRobot::updateControllerDir);
  connect(mWindow, &WbSFString::changed, this, &WbRobot::updateWindow);
  connect(mRemoteControl, &WbSFString::changed, this, &WbRobot::updateRemoteControl);
  connect(mCustomData, &WbSFString::changed, this, &WbRobot::updateData);
  connect(this, &WbMatter::matterModelChanged, this, &WbRobot::updateModel);
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this, &WbRobot::updateSimulationMode);

  if (absoluteScale() != WbVector3(1.0, 1.0, 1.0))
    warn(tr("This Robot node is scaled: this is discouraged as it could compromise the correct physical behavior."));
}

void WbRobot::reset() {
  WbSolid::reset();
  // restore battery level
  if (mBatteryInitialValue > 0)
    mBattery->setItem(CURRENT_ENERGY, mBatteryInitialValue);
  if (mSupervisorUtilities)
    mSupervisorUtilities->reset();
}

void WbRobot::save() {
  WbSolid::save();
  mBatteryInitialValue = (mBattery->size() > CURRENT_ENERGY) ? mBattery->item(CURRENT_ENERGY) : -1.0;
}

void WbRobot::addDevices(WbNode *node) {
  if (!node)
    return;

  if (node != this && dynamic_cast<const WbRobot *>(node))
    return;  // do not recurse through child robots, their devices are hidden

  WbGroup *group = dynamic_cast<WbGroup *>(node);
  if (group) {
    WbSolidDevice *solidDevice = dynamic_cast<WbSolidDevice *>(node);
    if (solidDevice) {
      mDevices.append(solidDevice);
      connect(static_cast<WbSolid *>(node), &WbSolid::destroyed, this, &WbRobot::updateDevicesAfterDestruction,
              Qt::UniqueConnection);

      WbRenderingDevice *renderingDevice = dynamic_cast<WbRenderingDevice *>(node);
      if (renderingDevice) {
        connect(renderingDevice, &WbBaseNode::isBeingDestroyed, this, &WbRobot::removeRenderingDevice, Qt::UniqueConnection);
        mRenderingDevices.append(renderingDevice);
      }
    }

    WbTrack *const track = dynamic_cast<WbTrack *>(group);
    if (track) {
      const QVector<WbLogicalDevice *> trackDevices = track->devices();
      for (int i = 0; i < trackDevices.size(); ++i) {
        mDevices.append(trackDevices[i]);
        connect(static_cast<WbBaseNode *>(trackDevices[i]), &WbBaseNode::destroyed, this,
                &WbRobot::updateDevicesAfterDestruction, Qt::UniqueConnection);
      }
    }

    WbMFNode::Iterator it(group->children());
    while (it.hasNext())
      addDevices(it.next());
    return;
  }

  WbSkin *const skin = dynamic_cast<WbSkin *>(node);
  if (skin) {
    mDevices.append(skin);
    connect(static_cast<WbBaseNode *>(skin), &WbBaseNode::destroyed, this, &WbRobot::updateDevicesAfterDestruction,
            Qt::UniqueConnection);
    return;
  }

  WbSlot *const slot = dynamic_cast<WbSlot *>(node);
  if (slot) {
    addDevices(slot->endPoint());
    return;
  }

  WbBasicJoint *const basicJoint = dynamic_cast<WbBasicJoint *>(node);
  if (basicJoint) {
    WbJoint *const joint = dynamic_cast<WbJoint *>(basicJoint);
    if (joint) {
      const QVector<WbLogicalDevice *> &jointDevices = joint->devices();
      foreach (WbLogicalDevice *const jointDevice, jointDevices) {
        if (jointDevice == NULL)
          continue;
        mDevices.append(jointDevice);
        connect(static_cast<WbBaseNode *>(jointDevice), &WbBaseNode::destroyed, this, &WbRobot::updateDevicesAfterDestruction,
                Qt::UniqueConnection);
      }
    }
    if (basicJoint->solidReference() == NULL)
      addDevices(basicJoint->solidEndPoint());
    return;
  }

  WbPropeller *const propeller = dynamic_cast<WbPropeller *>(node);
  if (propeller) {
    WbLogicalDevice *const propellerDevice = propeller->device();
    if (propellerDevice) {
      mDevices.append(propellerDevice);
      connect(static_cast<WbBaseNode *>(propellerDevice), &WbBaseNode::destroyed, this, &WbRobot::updateDevicesAfterDestruction,
              Qt::UniqueConnection);
    }
    return;
  }

  // check if there are duplicated names, and print a warning if necessary
  if (dynamic_cast<const WbRobot *>(node)) {  // top node
    QStringList displayedWarnings;
    foreach (WbDevice *deviceA, mDevices) {
      foreach (WbDevice *deviceB, mDevices) {
        if (deviceA != deviceB && deviceA->deviceName() == deviceB->deviceName() &&
            !displayedWarnings.contains(deviceA->deviceName())) {
          warn(tr("At least two devices are sharing the same name (\"%1\") while unique names are required.")
                 .arg(deviceA->deviceName()));
          displayedWarnings << deviceA->deviceName();
        }
      }
    }
  }
}

void WbRobot::clearDevices() {
  foreach (const WbDevice *device, mDevices)
    disconnect(dynamic_cast<const WbBaseNode *>(device), &WbBaseNode::destroyed, this, &WbRobot::updateDevicesAfterDestruction);
  foreach (const WbRenderingDevice *device, mRenderingDevices)
    disconnect(device, &WbBaseNode::isBeingDestroyed, this, &WbRobot::removeRenderingDevice);
  mDevices.clear();
  mRenderingDevices.clear();
}

void WbRobot::updateDevicesAfterDestruction() {
  clearDevices();
  addDevices(this);
}

void WbRobot::updateDevicesAfterInsertion() {
  clearDevices();
  addDevices(this);
}

void WbRobot::pinToStaticEnvironment(bool pin) {
  if (!mShowWindowCalled) {
    warn(tr("Cannot pin the robot to the static environment, because the robot window has not been shown once"));
    return;
  }

  mPin = pin;

  if (mPin) {
    mPinTranslation = translation();
    mPinRotation = rotation();
  }
}

QString WbRobot::protoModelProjectPath() const {
  if (isProtoInstance())
    return proto()->projectPath();
  return QString();
}

QString WbRobot::searchDynamicLibraryAbsolutePath(const QString &key, const QString &pluginSubdirectory) {
  QString libBasename = WbStandardPaths::dynamicLibraryPrefix() + key + WbStandardPaths::dynamicLibraryExtension();

  if (!key.isEmpty()) {
    QString regularPath =
      WbProject::current()->pluginsPath() + pluginSubdirectory + "/" + key + "/build/release/" + libBasename;
    QString buildPath = regularPath;
    regularPath.remove("build/release/");

    if (QFile::exists(buildPath) || QFile::exists(regularPath))
      return regularPath;

    if (isProtoInstance()) {
      // search in project folder associated with PROTO instance
      QString protoProjectPath = protoModelProjectPath();
      if (!protoProjectPath.isEmpty()) {
        QDir protoDir(protoProjectPath + "/plugins/" + pluginSubdirectory + "/" + key);
        if (protoDir.exists()) {
          QDir protoBuildDir(protoDir.absolutePath() + "/build/release/");
          if (protoDir.exists(libBasename) || protoBuildDir.exists(libBasename))
            return protoDir.absolutePath() + "/" + libBasename;
        }
      }
      // search in project folder associated with parent PROTO models
      WbProtoModel *protoModel = proto();
      while (protoModel) {
        if (!protoModel->projectPath().isEmpty()) {
          QDir protoDir(protoModel->projectPath() + "/plugins/" + pluginSubdirectory + "/" + key);
          if (protoDir.exists()) {
            QDir protoBuildDir(protoDir.absolutePath() + "/build/release/");
            if (protoDir.exists(libBasename) || protoBuildDir.exists(libBasename))
              return protoDir.absolutePath() + "/" + libBasename;
          }
        }
        protoModel = WbProtoList::current()->findModel(protoModel->ancestorProtoName(), "");
      }
    }

    WbControllerPlugin::Type t = WbControllerPlugin::pluginSubDirectoryToType(pluginSubdirectory);
    const QStringList &list = WbControllerPlugin::defaultList(t);

    foreach (const QString &item, list) {
      if (item.endsWith(libBasename))
        return item;
    }
  }

  return QString();
}

void WbRobot::updateWindow() {
  mAbsoluteWindowFilename = "";

  if (mConfigureRequest) {
    QString key = mWindow->value().trimmed();
    if (!key.isEmpty()) {
      const QString &absoluteFilePath = searchDynamicLibraryAbsolutePath(key, "robot_windows");
      if (absoluteFilePath.isEmpty() && windowFile().isEmpty())  // not a HTML robot window
        warn(tr("The robot window library has not been found."));
      else
        mAbsoluteWindowFilename = absoluteFilePath;
    }
  } else
    warn(tr("The robot 'window' cannot be modified after the controller is initialized."));

  if (!mWindow->value().isEmpty())
    return;  // HTML robot window without plugin case.

  if (mAbsoluteWindowFilename.isEmpty()) {
    mAbsoluteWindowFilename = WbStandardPaths::resourcesRobotWindowsPluginsPath() + "generic/" +
                              WbStandardPaths::dynamicLibraryPrefix() + "generic" + WbStandardPaths::dynamicLibraryExtension();
    if (!QFile::exists(mAbsoluteWindowFilename))
      warn(tr("The generic robot window is not found. Please check your Webots installation."));
  }

  if (!mAbsoluteWindowFilename.isEmpty())
    WbBinaryIncubator::copyBinaryAndDependencies(mAbsoluteWindowFilename);
}

void WbRobot::updateRemoteControl() {
  mAbsoluteRemoteControlFilename = "";

  if (mConfigureRequest) {
    QString key = mRemoteControl->value().trimmed();
    if (!key.isEmpty()) {
      const QString &absoluteFilePath = searchDynamicLibraryAbsolutePath(key, "remote_controls");
      if (absoluteFilePath.isEmpty())
        warn(tr("The remote control library has not been found."));
      else
        mAbsoluteRemoteControlFilename = absoluteFilePath;
    }
  } else
    warn(tr("'remoteControl' cannot be modified after the controller is initialized."));

  if (!mAbsoluteRemoteControlFilename.isEmpty())
    WbBinaryIncubator::copyBinaryAndDependencies(mAbsoluteRemoteControlFilename);
}

void WbRobot::updateControllerDir() {
  const QString &controllerName = mController->value();
  if (!controllerName.isEmpty()) {
    QStringList path;
    path << WbProject::current()->controllersPath() + controllerName + '/';
    const WbProtoModel *const protoModel = proto();
    if (protoModel)
      path << QDir::cleanPath(protoModelProjectPath() + "/controllers/" + controllerName) + '/';
    path << WbProject::defaultProject()->controllersPath() + controllerName + '/';
    path << WbProject::system()->controllersPath() + controllerName + '/';
    path.removeDuplicates();

    mControllerDir = "";
    const int size = path.size();
    for (int i = 0; i < size; ++i) {
      if (!QDir(path[i]).exists())
        continue;
      mControllerDir = path[i];
      break;
    }

    if (mControllerDir.isEmpty()) {
      QString warning(tr("The controller directory has not been found, searched the following locations:"));
      const int size = path.size();
      for (int i = 0; i < size; ++i)
        warning += "\n" + path[i];
      warn(warning);
    }
  }

  if (isPostFinalizedCalled()) {
    emit controllerChanged();
    foreach (WbRenderingDevice *device, mRenderingDevices) {
      WbAbstractCamera *ac = dynamic_cast<WbAbstractCamera *>(device);
      if (ac)
        ac->resetSharedMemory();  // shared memory automatically deleted at new controller start
    }
  }
}

const QString &WbRobot::controllerDir() {
  if (!isPreFinalizedCalled())
    updateControllerDir();

  return mControllerDir;
}

void WbRobot::restartController() {
  mControllerStarted = false;

  emit controllerChanged();

  foreach (WbDevice *device, mDevices) {
    WbAbstractCamera *ac = dynamic_cast<WbAbstractCamera *>(device);
    if (ac)
      ac->resetSharedMemory();  // shared memory automatically deleted at new controller restart
  }
}

bool WbRobot::isWaitingForUserInputEvent() const {
  return mMonitoredUserInputEventTypes != -1;
}

void WbRobot::setWaitingForWindow(bool waiting) {
  if (mWaitingForWindow == waiting)
    return;
  mWaitingForWindow = waiting;
  if (!waiting)
    emit windowReady();
}

void WbRobot::updateData() {
  mDataNeedToWriteAnswer = true;
}

void WbRobot::updateModel() {
  mModelNeedToWriteAnswer = true;
}

void WbRobot::removeRenderingDevice() {
  mRenderingDevices.removeOne(static_cast<WbRenderingDevice *>(sender()));
}

// reassign tags to devices (when device config changed)
void WbRobot::assignDeviceTags() {
  int i = 1;  // device tag 0 is reserved for the robot
  foreach (WbDevice *const device, mDevices)
    device->setTag(i++);
}

double WbRobot::currentEnergy() const {
  return (mBattery->size() > CURRENT_ENERGY) ? mBattery->item(CURRENT_ENERGY) : -1.0;
}

void WbRobot::setCurrentEnergy(double e) {
  assert(e >= 0.0);
  if (!mPowerOn && e > 0)
    powerOn(true);
  else if (e == 0 && mPowerOn)
    powerOn(false);

  if (mBattery->size() > CURRENT_ENERGY)
    mBattery->setItem(CURRENT_ENERGY, e);
}

double WbRobot::maxEnergy() const {
  return (mBattery->size() > MAX_ENERGY) ? mBattery->item(MAX_ENERGY) : -1.0;
}

double WbRobot::energyUploadSpeed() const {
  return (mBattery->size() > ENERGY_UPLOAD_SPEED) ? mBattery->item(ENERGY_UPLOAD_SPEED) : -1.0;
}

double WbRobot::energyConsumption() const {
  double e = mCpuConsumption->value();
  foreach (WbDevice *device, mDevices)  // add energy consumption for each device
    e += device->energyConsumption();
  return e;
}

void WbRobot::prePhysicsStep(double ms) {
  WbSolid::prePhysicsStep(ms);
  if (mKinematicDifferentialWheels)
    mKinematicDifferentialWheels->applyKinematicMotion(ms);
}

void WbRobot::postPhysicsStep() {
  WbSolid::postPhysicsStep();
  if (mPin) {
    setTranslation(mPinTranslation);
    setRotation(mPinRotation);
    resetPhysics();
  }
  double energy = currentEnergy();
  if (energy > 0.0) {
    double s = WbWorld::instance()->basicTimeStep() * 0.001;
    energy -= energyConsumption() * s;
    if (energy <= 0.0) {
      energy = 0.0;
      mBatterySensor->updateTimer();  // so that the battery sensor returns 0
    }
    setCurrentEnergy(energy);
  }
  if (mSupervisorUtilities)
    mSupervisorUtilities->postPhysicsStep();
}

WbDevice *WbRobot::findDevice(WbDeviceTag tag) const {
  foreach (WbDevice *const device, mDevices)
    if (device->tag() == tag)
      return device;

  return NULL;  // not found
}

void WbRobot::powerOn(bool e) {
  mPowerOn = e;
  foreach (WbDevice *const device, mDevices)
    device->powerOn(e);
}

void WbRobot::keyPressed(const QString &text, int key, int modifiers) {
  int pressedKeys = modifiers;

  if (gSpecialKeys.contains(key))
    // special key
    pressedKeys += gSpecialKeys.value(key);
  else if (!text.isEmpty())
    // normal key
    pressedKeys += key & WB_KEYBOARD_KEY;
  else
    // unknown key
    return;

  if (pressedKeys && !mPressedKeys.contains(pressedKeys)) {
    mPressedKeys.append(pressedKeys);
    mKeyboardHasChanged = true;
    emit keyboardChanged();
  }
}

void WbRobot::keyReleased(const QString &text, int key) {
  bool reset = true;
  QMutableListIterator<int> it(mPressedKeys);
  while (it.hasNext()) {
    int i = it.next();
    if ((i & 0xffff) == (gSpecialKeys.value(key) & 0xffff)) {
      // remove all sequences containing the released special key
      it.remove();
      reset = false;
    } else if (!text.isEmpty()) {
      if ((i & 0xffff) == (key & 0xffff)) {
        // remove all sequences containing the released key
        it.remove();
        reset = false;
      }
    }
  }

  if (reset)
    mPressedKeys.clear();

  mKeyboardHasChanged = true;
  emit keyboardChanged();
}

void WbRobot::writeConfigure(QDataStream &stream) {
  mBatterySensor->connectToRobotSignal(this);
  stream << (short unsigned int)0;
  stream << (unsigned char)C_CONFIGURE;
  stream << (unsigned char)(supervisor() ? 1 : 0);
  stream << (unsigned char)(synchronization() ? 1 : 0);
  stream << (short int)(1 + deviceCount());
  stream << (short int)nodeType();
  stream << (double)0.001 * WbSimulationState::instance()->time();

  QByteArray n = name().toUtf8();
  stream.writeRawData(n.constData(), n.size() + 1);

  foreach (WbDevice *const device, mDevices) {
    stream << (short int)device->deviceNodeType();
    n = device->deviceName().toUtf8();
    stream.writeRawData(n.constData(), n.size() + 1);
    WbSolidDevice *solidDevice = dynamic_cast<WbSolidDevice *>(device);
    if (solidDevice)
      n = solidDevice->model().toUtf8();
    else
      n = "";
    stream.writeRawData(n.constData(), n.size() + 1);
  }
  stream << (double)WbWorld::instance()->basicTimeStep();

  QString projectPath = WbProject::current()->path();
  // remove the last trailing space if any
  if (projectPath.endsWith('/'))
    projectPath.remove(-1, 1);
  n = projectPath.toUtf8();
  stream.writeRawData(n.constData(), n.size() + 1);

  QString worldPath = WbWorld::instance()->fileName();
  n = worldPath.toUtf8();
  stream.writeRawData(n.constData(), n.size() + 1);

  n = model().toUtf8();
  stream.writeRawData(n.constData(), n.size() + 1);
  n = mAbsoluteWindowFilename.toUtf8();
  stream.writeRawData(n.constData(), n.size() + 1);
  n = mAbsoluteRemoteControlFilename.toUtf8();
  stream.writeRawData(n.constData(), n.size() + 1);
  n = controllerName().toUtf8();
  stream.writeRawData(n.constData(), n.size() + 1);
  n = controllerArgs().toUtf8();
  stream.writeRawData(n.constData(), n.size() + 1);
  n = customData().toUtf8();
  stream.writeRawData(n.constData(), n.size() + 1);

  // show the robot window at step 0
  stream << (unsigned char)(mShowWindow->value() || mShowWindowMessage);
  stream << (unsigned char)(!windowFile().isEmpty());
  stream << (int)computeSimulationMode();

  mShowWindowMessage = false;
  mUpdateWindowMessage = false;
  mShowWindowCalled = true;
  mConfigureRequest = false;
  if (mSupervisorUtilities)
    mSupervisorUtilities->writeConfigure(stream);
}

void WbRobot::dispatchMessage(QDataStream &stream) {
  while (!stream.atEnd()) {
    WbDeviceTag tag;
    stream >> (short unsigned int &)tag;
    int size;
    stream >> (int &)size;

    if (tag == 0) {
      const int end = stream.device()->pos() + size;
      do  // handle all requests for this robot
        handleMessage(stream);
      while (stream.device()->pos() < end);
    } else {
      WbDevice *const device = findDevice(tag);
      if (device) {
        const int end = stream.device()->pos() + size;
        do  // handle all requests for this device
          device->handleMessage(stream);
        while (stream.device()->pos() < end);
      } else  // device was deleted in Webots (but the controller does not know about it)
        stream.skipRawData(size);
    }
  }
}

void WbRobot::handleMessage(QDataStream &stream) {
  QIODevice *const device = stream.device();

  char byte;
  unsigned char pin;
  device->getChar(&byte);

  switch (byte) {
    case C_CONFIGURE:
      updateDevicesAfterInsertion();
      mConfigureRequest = true;
      return;
    case C_SET_SAMPLING_PERIOD:  // for the scene tracker
      /*
      stream >> (short &)mRefreshRate;
      if (needSceneTracker==false) {
        A_SceneTracker::addNeed();
        needSceneTracker = true;
      }
      */
      return;
    case C_ROBOT_SET_BATTERY_SAMPLING_PERIOD: {
      short rate;
      stream >> (short &)rate;
      mBatterySensor->setRefreshRate(rate);
      return;
    }
    case C_ROBOT_SET_DATA: {
      short size;
      stream >> (short &)size;
      char data[size];
      stream.readRawData(data, size);
      mCustomData->setValue(data);
      return;
    }
    case C_ROBOT_SET_KEYBOARD_SAMPLING_PERIOD: {
      short rate;
      stream >> (short &)rate;
      mKeyboardSensor->setRefreshRate(rate);
      return;
    }
    case C_ROBOT_SET_JOYSTICK_SAMPLING_PERIOD: {
      short rate;
      stream >> (short &)rate;
      mJoystickSensor->setRefreshRate(rate);
      if (rate > 0) {
        if (!mJoystickInterface) {
          mJoystickInterface = new WbJoystickInterface();
          if (mJoystickInterface->isCorrectlyInitialized()) {
            mJoystickTimer = new QTimer(this);
            mJoystickTimer->setSingleShot(false);
            connect(mJoystickTimer, &QTimer::timeout, mJoystickInterface, &WbJoystickInterface::step);
            mJoystickConfigureRequest = true;
            mJoystickTimer->start(mJoystickInterface->updateRate());
          } else {
            warn(mJoystickInterface->initializationError());
            delete mJoystickInterface;
            mJoystickInterface = NULL;
          }
        }
      } else if (mJoystickInterface) {
        mJoystickTimer->stop();
        disconnect(mJoystickTimer, &QTimer::timeout, mJoystickInterface, &WbJoystickInterface::step);
        delete mJoystickTimer;
        delete mJoystickInterface;
        mJoystickTimer = NULL;
        mJoystickInterface = NULL;
        delete mJoyStickLastValue;
        mJoyStickLastValue = NULL;
      }
      return;
    }
    case C_ROBOT_SET_MOUSE_SAMPLING_PERIOD: {
      short rate;
      stream >> (short &)rate;
      if (rate <= 0 && mMouse != NULL) {
        WbMouse::destroy(mMouse);
        mMouse = NULL;
      } else if (rate > 0) {
        if (mMouse == NULL)
          mMouse = WbMouse::create();
        mMouse->setRefreshRate(rate);
      }
      return;
    }
    case C_ROBOT_MOUSE_ENABLE_3D_POSITION: {
      unsigned char enable;
      stream >> (unsigned char &)enable;
      if (mMouse)
        mMouse->set3dPositionEnabled(enable);
      return;
    }
    case C_ROBOT_SET_JOYSTICK_FORCE_FEEDBACK: {
      short level;
      stream >> (short &)level;
      if (mJoystickInterface && mJoystickInterface->isCorrectlyInitialized())
        mJoystickInterface->addForce(level);
      return;
    }
    case C_ROBOT_SET_JOYSTICK_FORCE_FEEDBACK_DURATION: {
      double duration;
      stream >> (double &)duration;
      if (mJoystickInterface && mJoystickInterface->isCorrectlyInitialized())
        mJoystickInterface->setConstantForceDuration(duration);
      return;
    }
    case C_ROBOT_SET_JOYSTICK_AUTO_CENTERING_GAIN: {
      double gain;
      stream >> (double &)gain;
      if (mJoystickInterface && mJoystickInterface->isCorrectlyInitialized())
        mJoystickInterface->setAutoCenteringGain(gain);
      return;
    }
    case C_ROBOT_SET_JOYSTICK_RESISTANCE_GAIN: {
      double gain;
      stream >> (double &)gain;
      if (mJoystickInterface && mJoystickInterface->isCorrectlyInitialized())
        mJoystickInterface->setResistanceGain(gain);
      return;
    }
    case C_ROBOT_SET_JOYSTICK_FORCE_AXIS: {
      int axis;
      stream >> (int &)axis;
      if (mJoystickInterface && mJoystickInterface->isCorrectlyInitialized())
        mJoystickInterface->setForceAxis(axis);
    }
    case C_ROBOT_CLIENT_EXIT_NOTIFY:
      /*
      C_Controller::displayControllerProcesses();
      A_Application::addRobotConsolePrint(name->getValue(), _("controller has terminated.\n"), 1);
      A_Application::setControllerRequest(this, NULL);
      */
      return;
    case C_ROBOT_REMOTE_ON:
      emit toggleRemoteMode(true);
      return;
    case C_ROBOT_REMOTE_OFF:
      emit toggleRemoteMode(false);
      return;
    case C_ROBOT_PIN:
      stream >> (unsigned char &)pin;
      pinToStaticEnvironment((bool)pin);
      return;
    case C_CONSOLE_MESSAGE: {
      unsigned char streamChannel = 0;
      stream >> (unsigned char &)streamChannel;
      unsigned int size;
      stream >> (unsigned int &)size;
      char nativeMessage[size];
      stream.readRawData(nativeMessage, size);
      QString message(nativeMessage);
      if (!message.endsWith('\n'))
        message += '\n';
      // cppcheck-suppress redundantCondition
      emit appendMessageToConsole(message, streamChannel == 0);
      return;
    }
    case C_ROBOT_WWI_MESSAGE: {
      int size;
      stream >> size;
      QByteArray message;
      message.resize(size - 1);  // an extra character is always added for the final '\0'
      stream.readRawData(message.data(), size);
      emit sendToJavascript(message);
      return;
    }
    case C_ROBOT_WAIT_FOR_USER_INPUT_EVENT: {
      int timeout;
      stream >> (int &)mMonitoredUserInputEventTypes;
      stream >> (int &)timeout;
      mNeedToWriteUserInputEventAnswer = false;
      if (mMonitoredUserInputEventTypes >= 0) {
        if (!mUserInputEventTimer) {
          mUserInputEventTimer = new QTimer(this);
          mUserInputEventTimer->setSingleShot(true);
          connect(mUserInputEventTimer, &QTimer::timeout, this, &WbRobot::userInputEventNeedUpdate);
        }
        if (((WB_EVENT_MOUSE_CLICK | WB_EVENT_MOUSE_MOVE) & mMonitoredUserInputEventTypes) && mMouse) {
          mMouse->setHasMoved(false);
          mMouse->setHasClicked(false);
          mMouse->setTracked(true);
          connect(mMouse, &WbMouse::changed, this, &WbRobot::handleMouseChange);
        }
        if ((WB_EVENT_KEYBOARD & mMonitoredUserInputEventTypes) && mKeyboardSensor->isEnabled()) {
          mKeyboardHasChanged = false;
          connect(this, &WbRobot::keyboardChanged, this, &WbRobot::userInputEventNeedUpdate);
        }
        if (((WB_EVENT_JOYSTICK_BUTTON | WB_EVENT_JOYSTICK_AXIS | WB_EVENT_JOYSTICK_POV) & mMonitoredUserInputEventTypes) &&
            mJoystickInterface && mJoystickInterface->isCorrectlyInitialized()) {
          mJoystickInterface->resetButtonHasChanged();
          mJoystickInterface->resetPovHasChanged();
          mJoystickInterface->resetAxisHasChanged();
          connect(mJoystickInterface, &WbJoystickInterface::changed, this, &WbRobot::handleJoystickChange);
        }
        mUserInputEventTimer->start(timeout);
        mUserInputEventReferenceTime = QDateTime::currentDateTime();
      } else if (mUserInputEventTimer)
        mUserInputEventTimer->stop();
      return;
    }
    default:
      // if it was not catched, then this message is apparently for a subclass of WbRobot
      // we must rewind 1 byte so the DifferentialWheels or Supervisor can read the command
      device->ungetChar(byte);
  }
  if (mSupervisorUtilities)
    mSupervisorUtilities->handleMessage(stream);
}

void WbRobot::dispatchAnswer(QDataStream &stream, bool includeDevices) {
  if (mConfigureRequest) {
    assignDeviceTags();
    writeConfigure(stream);
    if (includeDevices) {
      foreach (WbDevice *const device, mDevices) {
        if (device->hasTag())
          device->writeConfigure(stream);
      }
    }
  } else {
    if (includeDevices) {
      foreach (WbDevice *const device, mDevices)
        if (device->hasTag())
          device->writeAnswer(stream);
    }
    writeAnswer(stream);
  }
}

void WbRobot::writeAnswer(QDataStream &stream) {
  const double time = 0.001 * WbSimulationState::instance()->time();
  if (time != mPreviousTime) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_TIME;
    stream << (double)time;
    mPreviousTime = time;
  }

  if (mSimulationModeRequested) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_SIMULATION_CHANGE_MODE;
    stream << (int)computeSimulationMode();
    mSimulationModeRequested = false;
  }

  if (refreshBatterySensorIfNeeded() || mBatterySensor->hasPendingValue()) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_BATTERY_VALUE;
    stream << (double)mBattery->item(CURRENT_ENERGY);
    mBatterySensor->resetPendingValue();
  }

  if (mDataNeedToWriteAnswer) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_DATA;
    QByteArray n = customData().toUtf8();
    stream.writeRawData(n.constData(), n.size() + 1);

    mDataNeedToWriteAnswer = false;
  }

  if (mModelNeedToWriteAnswer) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_MODEL;
    QByteArray n = model().toUtf8();
    stream.writeRawData(n.constData(), n.size() + 1);

    mModelNeedToWriteAnswer = false;
  }

  int userInputEvents = 0;
  if (mMonitoredUserInputEventTypes >= 0 && mNeedToWriteUserInputEventAnswer) {
    mUserInputEventTimer->stop();
    if (mMouse && (mMonitoredUserInputEventTypes & (WB_EVENT_MOUSE_CLICK | WB_EVENT_MOUSE_MOVE))) {
      if (mMouse->hasClicked() && (mMonitoredUserInputEventTypes & WB_EVENT_MOUSE_CLICK))
        userInputEvents = userInputEvents | WB_EVENT_MOUSE_CLICK;
      if (mMouse->hasMoved() && (mMonitoredUserInputEventTypes & WB_EVENT_MOUSE_MOVE))
        userInputEvents = userInputEvents | WB_EVENT_MOUSE_MOVE;
      mMouse->setTracked(false);
      disconnect(mMouse, &WbMouse::changed, this, &WbRobot::handleMouseChange);
    }
    if (mMonitoredUserInputEventTypes & WB_EVENT_KEYBOARD) {
      if (mKeyboardHasChanged)
        userInputEvents = userInputEvents | WB_EVENT_KEYBOARD;
      disconnect(this, &WbRobot::keyboardChanged, this, &WbRobot::userInputEventNeedUpdate);
    }
    if (mJoystickInterface &&
        (mMonitoredUserInputEventTypes & (WB_EVENT_JOYSTICK_BUTTON | WB_EVENT_JOYSTICK_AXIS | WB_EVENT_JOYSTICK_POV))) {
      if (mJoystickInterface->buttonHasChanged() && (mMonitoredUserInputEventTypes & WB_EVENT_JOYSTICK_BUTTON))
        userInputEvents = userInputEvents | WB_EVENT_JOYSTICK_BUTTON;
      if (mJoystickInterface->povHasChanged() && (mMonitoredUserInputEventTypes & WB_EVENT_JOYSTICK_POV))
        userInputEvents = userInputEvents | WB_EVENT_JOYSTICK_POV;
      if (mJoystickInterface->axisHasChanged() && (mMonitoredUserInputEventTypes & WB_EVENT_JOYSTICK_AXIS))
        userInputEvents = userInputEvents | WB_EVENT_JOYSTICK_AXIS;
      disconnect(mJoystickInterface, &WbJoystickInterface::changed, this, &WbRobot::handleJoystickChange);
    }
  }

  if (refreshKeyboardSensorIfNeeded() || mKeyboardSensor->hasPendingValue()) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_KEYBOARD_VALUE;
    stream << (unsigned char)mKeyboardLastValue.size();

    foreach (int key, mKeyboardLastValue)
      stream << (int)key;

    mKeyboardSensor->resetPendingValue();
  }

  if (mJoystickConfigureRequest && mJoystickInterface && mJoystickInterface->isCorrectlyInitialized()) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_JOYSTICK_CONFIG;
    stream << (int)mJoystickInterface->numberOfAxes();
    stream << (int)mJoystickInterface->numberOfButtons();
    stream << (int)mJoystickInterface->numberOfPovs();
    QByteArray n = mJoystickInterface->model().toUtf8();
    stream.writeRawData(n.constData(), n.size() + 1);
  } else if (mJoystickConfigureRequest) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_JOYSTICK_CONFIG;
    stream << (int)-1;
    stream << (int)-1;
    stream << (int)-1;
  }

  if (refreshJoyStickSensorIfNeeded() || mJoystickSensor->hasPendingValue()) {
    assert(mJoyStickLastValue);
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_JOYSTICK_VALUE;
    stream << (int)mJoyStickLastValue->numberOfPressedButtons;
    for (int i = 0; i < mJoyStickLastValue->pressedButtonsIndices.size(); ++i)
      stream << (int)mJoyStickLastValue->pressedButtonsIndices[i];
    stream << (int)mJoyStickLastValue->numberOfAxes;
    for (int i = 0; i < mJoyStickLastValue->axesValues.size(); ++i)
      stream << (int)mJoyStickLastValue->axesValues[i];
    stream << (int)mJoyStickLastValue->numberOfPovs;
    for (int i = 0; i < mJoyStickLastValue->povsValues.size(); ++i)
      stream << (int)mJoyStickLastValue->povsValues[i];
    mJoystickSensor->resetPendingValue();
  }

  if ((userInputEvents & (WB_EVENT_MOUSE_CLICK | WB_EVENT_MOUSE_MOVE)) ||
      (mMouse != NULL && mPowerOn && mMouse->hasPendingValue())) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_MOUSE_VALUE;
    stream << (unsigned char)mMouse->left();
    stream << (unsigned char)mMouse->middle();
    stream << (unsigned char)mMouse->right();
    stream << (double)mMouse->u();
    stream << (double)mMouse->v();
    stream << (double)mMouse->x();
    stream << (double)mMouse->y();
    stream << (double)mMouse->z();
    mMouse->reset();
  }

  if (mMonitoredUserInputEventTypes >= 0 && mNeedToWriteUserInputEventAnswer) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_WAIT_FOR_USER_INPUT_EVENT;
    stream << (int)userInputEvents;
    mMonitoredUserInputEventTypes = -1;
    mNeedToWriteUserInputEventAnswer = false;
  }

  if (mSupervisorUtilities)
    mSupervisorUtilities->writeAnswer(stream);
}

bool WbRobot::hasImmediateAnswer() const {
  if (mConfigureRequest)
    return false;
  return mShowWindowMessage || mUpdateWindowMessage || mMessageFromWwi;
}

void WbRobot::writeImmediateAnswer(QDataStream &stream) {
  if (mConfigureRequest)
    return;
  if (mShowWindowMessage) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_WINDOW_SHOW;
    mShowWindowMessage = false;
    mShowWindowCalled = true;
  }
  if (mUpdateWindowMessage) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_WINDOW_UPDATE;
    mUpdateWindowMessage = false;
  }
  if (mMessageFromWwi) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_ROBOT_WWI_MESSAGE;
    int length = mMessageFromWwi->length() + 1;
    stream << length;
    stream.writeRawData(mMessageFromWwi->constData(), length);
    delete mMessageFromWwi;
    mMessageFromWwi = NULL;
  }
}

void WbRobot::handleMouseChange() {
  assert(mMouse);
  assert(mUserInputEventTimer);

  if ((mMonitoredUserInputEventTypes & WB_EVENT_MOUSE_CLICK) && mMouse->hasClicked()) {
    mUserInputEventTimer->stop();
    emit userInputEventNeedUpdate();
    return;
  }

  if ((mMonitoredUserInputEventTypes & WB_EVENT_MOUSE_MOVE) && mMouse->hasMoved()) {
    const int elapsedTime = mUserInputEventReferenceTime.msecsTo(QDateTime::currentDateTime());
    const int difference = elapsedTime - mMouse->refreshRate();
    if (difference > 0) {
      mUserInputEventTimer->stop();
      emit userInputEventNeedUpdate();
      return;
    } else
      mUserInputEventTimer->start(-difference);
  }
}

void WbRobot::handleJoystickChange() {
  assert(mJoystickInterface);
  assert(mUserInputEventTimer);

  if (((mMonitoredUserInputEventTypes & WB_EVENT_JOYSTICK_BUTTON) && mJoystickInterface->buttonHasChanged()) ||
      ((mMonitoredUserInputEventTypes & WB_EVENT_JOYSTICK_POV) && mJoystickInterface->povHasChanged())) {
    mUserInputEventTimer->stop();
    emit userInputEventNeedUpdate();
    return;
  }

  if ((mMonitoredUserInputEventTypes & WB_EVENT_JOYSTICK_AXIS) && mJoystickInterface->axisHasChanged()) {
    const int elapsedTime = mUserInputEventReferenceTime.msecsTo(QDateTime::currentDateTime());
    const int difference = elapsedTime - mJoystickSensor->refreshRate();
    if (difference > 0) {
      mUserInputEventTimer->stop();
      emit userInputEventNeedUpdate();
      return;
    } else
      mUserInputEventTimer->start(-difference);
  }
}

QString WbRobot::windowFile(const QString &extension) {
  if (window().isEmpty())
    return WbStandardPaths::resourcesRobotWindowsPluginsPath() + "generic/generic." + extension;

  const QString fileName = window() + "/" + window() + "." + extension;
  QString path = WbProject::current()->robotWindowPluginsPath() + fileName;
  QFileInfo file(path);
  if (file.exists() && file.isFile() && file.isReadable())
    return path;
  if (isProtoInstance()) {
    // search in project folder associated with PROTO instance
    path = protoModelProjectPath() + "/plugins/robot_windows/" + fileName;
    file = QFileInfo(path);
    if (file.exists() && file.isFile() && file.isReadable())
      return path;
    // search in project folder associated with parent PROTO models
    WbProtoModel *protoModel = proto();
    while (protoModel) {
      if (!protoModel->projectPath().isEmpty()) {
        path = protoModel->projectPath() + "/plugins/robot_windows/" + fileName;
        file = QFileInfo(path);
        if (file.exists() && file.isFile() && file.isReadable())
          return path;
      }
      protoModel = WbProtoList::current()->findModel(protoModel->ancestorProtoName(), "");
    }
  }

  return "";
}

void WbRobot::showWindow() {  // show the Qt-based controller robot window (to be deprecated)
  mShowWindowMessage = true;
  if (!isControllerStarted())
    startController();
  emit immediateMessageAdded();
}

void WbRobot::updateControllerWindow() {  // run a html window step with duration 0 ms
  mUpdateWindowMessage = true;
  emit immediateMessageAdded();
}

void WbRobot::processImmediateMessages() {
  if (mSupervisorUtilities)
    mSupervisorUtilities->processImmediateMessages();
}

void WbRobot::startController() {
  emit startControllerRequest(this);
}

void WbRobot::receiveFromJavascript(const QByteArray &message) {
  delete mMessageFromWwi;
  mMessageFromWwi = new QByteArray(message);
  emit immediateMessageAdded();
}

void WbRobot::updateSimulationMode() {
  mSimulationModeRequested = true;
  emit immediateMessageAdded();
}

void WbRobot::descendantNodeInserted(WbBaseNode *decendant) {
  if (isPreFinalizedCalled())
    updateDevicesAfterInsertion();
}

void WbRobot::updateSensors() {
  refreshBatterySensorIfNeeded();
  refreshKeyboardSensorIfNeeded();
  refreshJoyStickSensorIfNeeded();

  if (mDevices.isEmpty())
    return;

  foreach (WbDevice *const device, mDevices)
    device->refreshSensorIfNeeded();
}

bool WbRobot::refreshBatterySensorIfNeeded() {
  if (mPowerOn && mBatterySensor->needToRefresh()) {
    mBatteryLastValue = currentEnergy();
    mBatterySensor->updateTimer();
    return true;
  }
  return false;
}

bool WbRobot::refreshKeyboardSensorIfNeeded() {
  if ((mPowerOn && mKeyboardSensor->needToRefresh()) ||
      ((mMonitoredUserInputEventTypes & WB_EVENT_KEYBOARD) && mKeyboardHasChanged)) {
    mKeyboardLastValue = mPressedKeys;
    mKeyboardSensor->updateTimer();
    return true;
  }
  return false;
}

bool WbRobot::refreshJoyStickSensorIfNeeded() {
  if ((mJoystickInterface && mJoystickInterface->isCorrectlyInitialized()) &&
      ((mPowerOn && mJoystickSensor->needToRefresh()) ||
       ((WB_EVENT_JOYSTICK_BUTTON & mMonitoredUserInputEventTypes) && mJoystickInterface->buttonHasChanged()) ||
       ((WB_EVENT_JOYSTICK_POV & mMonitoredUserInputEventTypes) && mJoystickInterface->povHasChanged()) ||
       ((WB_EVENT_JOYSTICK_AXIS & mMonitoredUserInputEventTypes) && mJoystickInterface->axisHasChanged()))) {
    if (mJoyStickLastValue == NULL)
      mJoyStickLastValue = new JoyStickLastValue;
    if (WB_EVENT_JOYSTICK_BUTTON & mMonitoredUserInputEventTypes)
      mJoystickInterface->releaseButtons();
    mJoyStickLastValue->pressedButtonsIndices.clear();
    mJoyStickLastValue->axesValues.clear();
    mJoyStickLastValue->povsValues.clear();
    mJoystickInterface->capture();
    int numberOfButttons = mJoystickInterface->numberOfButtons();
    mJoyStickLastValue->numberOfPressedButtons = mJoystickInterface->numberOfPressedButtons();
    for (int i = 0; i < numberOfButttons; ++i) {
      if (mJoystickInterface->isButtonPressed(i))
        mJoyStickLastValue->pressedButtonsIndices.append(i);
    }
    mJoyStickLastValue->numberOfAxes = mJoystickInterface->numberOfAxes();
    for (int i = 0; i < mJoyStickLastValue->numberOfAxes; ++i)
      mJoyStickLastValue->axesValues.append((int)mJoystickInterface->axisValue(i));
    mJoyStickLastValue->numberOfPovs = mJoystickInterface->numberOfPovs();
    for (int i = 0; i < mJoyStickLastValue->numberOfPovs; ++i)
      mJoyStickLastValue->povsValues.append((int)mJoystickInterface->povValue(i));
    mJoystickInterface->releaseButtons();
    mJoystickSensor->updateTimer();
    return true;
  }
  return false;
}

void WbRobot::exportNodeFields(WbVrmlWriter &writer) const {
  WbSolid::exportNodeFields(writer);
  if (writer.isX3d()) {
    if (findField("controller") && !controllerName().isEmpty())
      writer << " controller='" << controllerName() << "'";
    if (findField("window") && !window().isEmpty())
      writer << " window='" << window() << "'";
  }
}

int WbRobot::computeSimulationMode() {
  WbSimulationState *state = WbSimulationState::instance();
  switch (state->mode()) {
    case WbSimulationState::REALTIME:
      return WB_SUPERVISOR_SIMULATION_MODE_REAL_TIME;
    case WbSimulationState::RUN:
      return WB_SUPERVISOR_SIMULATION_MODE_RUN;
    case WbSimulationState::FAST:
      return WB_SUPERVISOR_SIMULATION_MODE_FAST;
    default:
      return WB_SUPERVISOR_SIMULATION_MODE_PAUSE;
  }
}
