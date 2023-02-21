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

#ifndef WB_RENDERING_WINDOW_FACTORY_HPP
#define WB_RENDERING_WINDOW_FACTORY_HPP

#include <QtCore/QObject>

class QOpenGLContext;

class WbPerspective;
class WbRenderingDevice;
class WbRenderingDeviceWindow;

class WbRenderingDeviceWindowFactory : public QObject {
  Q_OBJECT

public:
  static WbRenderingDeviceWindowFactory *instance();
  static void deleteInstance();
  static void reset();
  static void storeOpenGLContext(QOpenGLContext *context);

  void showWindowForDevice(WbRenderingDevice *device);

  // connect signal to restore perspective
  void listenToRenderingDevice(const WbRenderingDevice *device) const;
  QStringList windowPerspective(const WbRenderingDevice *device);
  void saveWindowsPerspective(WbPerspective &perspective);

  // set if windows are enabled (for example for FAST mode)
  // if false then hide windows and restore them next time this flag is set to true
  void setWindowsEnabled(bool enabled);

private:
  WbRenderingDeviceWindowFactory();
  ~WbRenderingDeviceWindowFactory();

  WbRenderingDeviceWindow *getWindowForDevice(WbRenderingDevice *device, bool createIfNeeded);

  QList<WbRenderingDeviceWindow *> mWindowsList;
  QList<WbRenderingDeviceWindow *> mActiveWindowsList;
  static WbRenderingDeviceWindowFactory *cInstance;

private slots:
  void restoreWindowPerspective(const WbRenderingDevice *device, const QStringList &perspective);
  void deleteWindow();
};

#endif
