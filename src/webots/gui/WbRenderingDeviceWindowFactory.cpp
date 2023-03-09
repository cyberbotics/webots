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

#include "WbRenderingDeviceWindowFactory.hpp"

#include "WbPerspective.hpp"
#include "WbRenderingDevice.hpp"
#include "WbRenderingDeviceWindow.hpp"

#include <cassert>

WbRenderingDeviceWindowFactory *WbRenderingDeviceWindowFactory::cInstance = NULL;

WbRenderingDeviceWindowFactory *WbRenderingDeviceWindowFactory::instance() {
  if (!cInstance)
    cInstance = new WbRenderingDeviceWindowFactory();

  return cInstance;
}

void WbRenderingDeviceWindowFactory::deleteInstance() {
  delete cInstance;
  cInstance = NULL;
}

void WbRenderingDeviceWindowFactory::reset() {
  delete cInstance;
  cInstance = new WbRenderingDeviceWindowFactory();
}

void WbRenderingDeviceWindowFactory::storeOpenGLContext(QOpenGLContext *context) {
  WbRenderingDeviceWindow::storeOpenGLContext(context);
}

WbRenderingDeviceWindowFactory::WbRenderingDeviceWindowFactory() {
  QList<WbRenderingDevice *> devices = WbRenderingDevice::renderingDevices();
  for (int i = 0; i < devices.size(); ++i)
    connect(devices[i], &WbRenderingDevice::restoreWindowPerspective, this,
            &WbRenderingDeviceWindowFactory::restoreWindowPerspective, Qt::UniqueConnection);
}

WbRenderingDeviceWindowFactory::~WbRenderingDeviceWindowFactory() {
  for (int i = 0; i < mWindowsList.size(); ++i) {
    mWindowsList[i]->setVisible(false);
    delete mWindowsList[i];
  }
  mWindowsList.clear();
  mActiveWindowsList.clear();
}

void WbRenderingDeviceWindowFactory::listenToRenderingDevice(const WbRenderingDevice *device) const {
  connect(device, &WbRenderingDevice::restoreWindowPerspective, this, &WbRenderingDeviceWindowFactory::restoreWindowPerspective,
          Qt::UniqueConnection);
}

void WbRenderingDeviceWindowFactory::showWindowForDevice(WbRenderingDevice *device) {
  WbRenderingDeviceWindow *window = getWindowForDevice(device, true);
  window->show();
}

void WbRenderingDeviceWindowFactory::saveWindowsPerspective(WbPerspective &perspective) {
  for (int i = 0; i < mWindowsList.size(); ++i) {
    if (mWindowsList[i]->isVisible()) {
      WbRenderingDeviceWindow *window = mWindowsList[i];
      QStringList devicePerspective(window->device()->perspective());
      devicePerspective << window->perspective();
      perspective.setRenderingDevicePerspective(window->device()->computeShortUniqueName(), devicePerspective);
    }
  }
}

QStringList WbRenderingDeviceWindowFactory::windowPerspective(const WbRenderingDevice *device) {
  const WbRenderingDeviceWindow *window = getWindowForDevice(const_cast<WbRenderingDevice *>(device), false);
  if (window) {
    QStringList perspective(window->device()->perspective());
    perspective << window->perspective();
    return perspective;
  }
  return QStringList();
}

void WbRenderingDeviceWindowFactory::restoreWindowPerspective(const WbRenderingDevice *device, const QStringList &perspective) {
  if (perspective.size() < 5)
    // invalid window perspective
    return;

  bool valid = false;
  for (int i = 0; i < 5; i++)
    valid |= perspective.at(i) != "0";
  if (!valid)
    return;  // external window disabled

  WbRenderingDeviceWindow *window = getWindowForDevice(const_cast<WbRenderingDevice *>(device), true);
  window->restorePerspective(perspective);
  window->show();
}

WbRenderingDeviceWindow *WbRenderingDeviceWindowFactory::getWindowForDevice(WbRenderingDevice *device, bool createIfNeeded) {
  for (int i = 0; i < mWindowsList.size(); ++i) {
    if (mWindowsList[i]->deviceId() == device->uniqueId())
      return mWindowsList[i];
  }

  if (!createIfNeeded)
    return NULL;

  WbRenderingDeviceWindow *window = new WbRenderingDeviceWindow(device);
  connect(device, &WbRenderingDevice::isBeingDestroyed, this, &WbRenderingDeviceWindowFactory::deleteWindow);
  mWindowsList.append(window);
  return window;
}

void WbRenderingDeviceWindowFactory::setWindowsEnabled(bool enabled) {
  if (enabled == mActiveWindowsList.isEmpty())
    return;

  if (enabled) {
    for (int i = 0; i < mActiveWindowsList.size(); ++i)
      mActiveWindowsList[i]->show();
    mActiveWindowsList.clear();
  } else {
    for (int i = 0; i < mWindowsList.size(); ++i) {
      if (!mWindowsList[i]->isVisible())
        continue;
      mActiveWindowsList.append(mWindowsList[i]);
      mWindowsList[i]->close();
    }
  }
}

void WbRenderingDeviceWindowFactory::deleteWindow() {
  WbBaseNode *node = dynamic_cast<WbBaseNode *>(sender());
  assert(node);
  for (int i = 0; i < mWindowsList.size(); ++i) {
    if (mWindowsList[i]->deviceId() == node->uniqueId()) {
      WbRenderingDeviceWindow *window = mWindowsList[i];
      mActiveWindowsList.removeOne(window);
      mWindowsList.removeOne(window);
      delete window;
      return;
    }
  }
}
