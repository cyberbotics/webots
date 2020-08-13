// Copyright 1996-2020 Cyberbotics Ltd.
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

#ifndef WB_RENDERING_DEVICE_WINDOW_HPP
#define WB_RENDERING_DEVICE_WINDOW_HPP

#include <QtGui/QOpenGLFunctions_3_3_Core>
#include <QtGui/QWindow>
#ifdef _WIN32
#include <windows.h>
#endif

class QOpenGLContext;
class QOpenGLShaderProgram;

class WbRenderingDevice;
class WbAbstractCamera;

class WbRenderingDeviceWindow : public QWindow {
  Q_OBJECT

public:
  explicit WbRenderingDeviceWindow(WbRenderingDevice *device);
  ~WbRenderingDeviceWindow();

  WbRenderingDevice *device() const { return mDevice; }
  int deviceId() const;

  QStringList perspective() const;
  void restorePerspective(const QStringList &perspective);

  static void storeOpenGLContext(QOpenGLContext *context);

protected:
  bool event(QEvent *event) override;

private:
  static QOpenGLContext *cMainOpenGLContext;

  QOpenGLContext *mContext;
  QOpenGLShaderProgram *mProgram;
  WbRenderingDevice *mDevice;
  WbAbstractCamera *mAbstractCamera;
  GLuint mTextureGLId;
  GLuint mBackgroundTextureGLId;
  GLuint mForegroundTextureGLId;
  GLuint mMaxRangeUniform;
  GLuint mImageUniform;
  GLuint mBackgroundTextureUniform;
  GLuint mForegroundTextureUniform;
  GLuint mVaoId;
  GLuint mVboId[2];
  bool mInitialized;
  float mXFactor;
  float mYFactor;
  QRect mPreviousGeometry;

  bool mUpdateRequested;
  bool mShowOverlayOnClose;

  void initialize();
  void render();

private slots:
  void renderNow();
  void requestUpdate();
  void updateTextureGLId(int id);
  void updateBackgroundTextureGLId(int id);
  void updateForegroundTextureGLId(int id);
  void closeFromMainWindow();
};

#endif
