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

#ifndef WB_RENDERING_DEVICE_HPP
#define WB_RENDERING_DEVICE_HPP

#include <QtCore/QObject>

#include "WbSolidDevice.hpp"

class WbWrenTextureOverlay;
class WbSFDouble;
class WbSFInt;
class WbSFVector2;

class WbRenderingDevice : public WbSolidDevice {
  Q_OBJECT

public:
  virtual ~WbRenderingDevice();

  // reimplemented public functions
  void preFinalize() override;
  void postFinalize() override;

  // overlay related functions
  void toggleOverlayVisibility(bool enabled, bool emitSignal = false);
  void moveWindow(int dx, int dy);
  void setPixelSize(double pixelSize);

  QStringList perspective() const;
  void restorePerspective(QStringList &perspective);
  bool isOverlayEnabled() const;
  bool isWindowActive() const { return mIsExternalWindowEnabled; }
  double pixelSize() const;

  // external window
  virtual int textureGLId() const;
  virtual int backgroundTextureGLId() const;
  virtual int maskTextureGLId() const;
  virtual int foregroundTextureGLId() const;
  virtual void enableExternalWindow(bool enabled);

  // getters
  virtual int width() const;
  virtual int height() const;
  WbWrenTextureOverlay *overlay() const { return mOverlay; }
  virtual QString pixelInfo(int x, int y) const = 0;

  bool hasBeenSetup() const { return mHasBeenSetup; }

  // static functions
  static WbRenderingDevice *fromMousePosition(int x, int y);
  static QList<WbRenderingDevice *> renderingDevices() { return cRenderingDevices; }

  enum TextureRole { BACKGROUND_TEXTURE = 0, MAIN_TEXTURE, MASK_TEXTURE, FOREGROUND_TEXTURE };

signals:
  void overlayVisibilityChanged(bool visible);
  void overlayStatusChanged(bool enabled);
  void textureUpdated();
  void textureIdUpdated(int textureGLID, TextureRole role);
  void restoreWindowPerspective(const WbRenderingDevice *device, const QStringList &perspective);
  void closeWindow();

protected:
  // all constructors are reserved for derived classes only
  WbRenderingDevice(const QString &modelName, WbTokenizer *tokenizer);
  WbRenderingDevice(const WbRenderingDevice &other);
  WbRenderingDevice(const WbNode &other);

  // WREN Data
  WbWrenTextureOverlay *mOverlay;

  // setup functions
  virtual void setup();

  virtual void createWrenOverlay() = 0;  // not very useful: this function is not called in a polymorphical way

  bool areOverlaysEnabled() const;  // global preferences value

protected slots:
  virtual void updateWidth();
  virtual void updateHeight();

private:
  WbRenderingDevice &operator=(const WbRenderingDevice &);  // non copyable
  void init();

  // user accessible fields
  WbSFInt *mWidth;
  WbSFInt *mHeight;

  // values just after the setup
  int mSetupWidth;
  int mSetupHeight;

  // private stuff
  bool mHasBeenSetup;
  bool mIsExternalWindowEnabled;

  // static variables
  static QList<WbRenderingDevice *> cRenderingDevices;  // list of the current devices rendering in the 3D-view; used by
                                                        // WbView3D after each resize event to reset texture overlays positions
};

#endif
