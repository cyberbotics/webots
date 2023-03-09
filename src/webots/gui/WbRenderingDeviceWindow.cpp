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

#include "WbRenderingDeviceWindow.hpp"

#include "WbAbstractCamera.hpp"
#include "WbCamera.hpp"
#include "WbDisplay.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPerformanceLog.hpp"
#include "WbRenderingDevice.hpp"
#include "WbRobot.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenTextureOverlay.hpp"

#include <QtGui/QOpenGLContext>
#include <QtOpenGL/QOpenGLShaderProgram>
#include <QtOpenGL/QOpenGLVersionFunctionsFactory>

static const char *gVertexShaderSource = "#version 330\n"
                                         "layout (location = 0) in vec4 posAttr;\n"
                                         "layout (location = 1) in vec2 uvAttr;\n"
                                         "out vec2 uv0;\n"
                                         "void main() {\n"
                                         "  uv0 = uvAttr;\n"
                                         "  gl_Position = posAttr;\n"
                                         "}\n";

static const char *gStdFragmentShaderSource = "#version 330\n"
                                              "uniform sampler2D image;\n"
                                              "in vec2 uv0;\n"
                                              "out vec4 fragColor;\n"
                                              "void main() {\n"
                                              "  vec2 deviceUv0 = uv0;\n"
                                              "  fragColor = texture(image, deviceUv0);\n"
                                              "}\n";

static const char *gBackgroundFragmentShaderSource = "#version 330\n"
                                                     "uniform sampler2D image;\n"
                                                     "uniform sampler2D background;\n"
                                                     "in vec2 uv0;\n"
                                                     "out vec4 fragColor;\n"
                                                     "void main() {\n"
                                                     "  vec4 backgroundColor = texture(background, uv0);\n"
                                                     "  vec2 deviceUv0 = uv0;\n"
                                                     "  vec4 color = texture(image, deviceUv0);\n"
                                                     "  fragColor = mix(backgroundColor, color, color.a);\n"
                                                     "}\n";

static const char *gForegroundFragmentShaderSource = "#version 330\n"
                                                     "uniform sampler2D image;\n"
                                                     "uniform sampler2D mask;\n"
                                                     "uniform sampler2D foreground;\n"
                                                     "uniform int activeTextures;\n"
                                                     "in vec2 uv0;\n"
                                                     "out vec4 fragColor;\n"
                                                     "void main() {\n"
                                                     "  fragColor = texture(image, uv0);\n"
                                                     "  if ((activeTextures & 0x01) != 0) {\n"  // mask is active
                                                     "    vec4 maskColor = texture(mask, uv0);\n"
                                                     "    if (maskColor.x > 0.01 || maskColor.y > 0.01 || maskColor.z > 0.01)\n"
                                                     "      fragColor = mix(fragColor, maskColor, 0.8);\n"
                                                     "    else\n"
                                                     "      fragColor = mix(fragColor, vec4(1.0, 1.0, 1.0, 1.0), 0.4);\n"
                                                     "  }\n"
                                                     "  if ((activeTextures & 0x10) != 0) {\n"  // foreground is active
                                                     "    vec4 foregroundColor = texture(foreground, uv0);\n"
                                                     "    fragColor = mix(fragColor, foregroundColor, foregroundColor.a);\n"
                                                     "  }\n"
                                                     "}\n";

static const char *gDepthFragmentShaderSource = "#version 330\n"
                                                "uniform sampler2D image;\n"
                                                "uniform float maxRange;\n"
                                                "in vec2 uv0;\n"
                                                "out vec4 fragColor;\n"
                                                "void main() {\n"
                                                "   float normalizedDepth = texture(image, uv0).x / maxRange;\n"
                                                "   fragColor = vec4(normalizedDepth, normalizedDepth, normalizedDepth, 1.0);\n"
                                                "}\n";

QOpenGLContext *WbRenderingDeviceWindow::cMainOpenGLContext = NULL;

void WbRenderingDeviceWindow::storeOpenGLContext(QOpenGLContext *context) {
  cMainOpenGLContext = context;
}

WbRenderingDeviceWindow::WbRenderingDeviceWindow(WbRenderingDevice *device) :
  QWindow(),
  mContext(NULL),
  mProgram(NULL),
  mDevice(device),
  mTextureGLId(device->textureGLId()),
  mBackgroundTextureGLId(device->backgroundTextureGLId()),
  mMaskTextureGLId(device->maskTextureGLId()),
  mForegroundTextureGLId(device->foregroundTextureGLId()),
  mVboId(NULL),
  mInitialized(false),
  mXFactor(0.0f),
  mYFactor(0.0f),
  mUpdateRequested(true),
  mShowOverlayOnClose(true) {
  setSurfaceType(QWindow::OpenGLSurface);
  QSurfaceFormat surfaceFormat = format();
  surfaceFormat.setMajorVersion(3);
  surfaceFormat.setMinorVersion(3);
  setFormat(surfaceFormat);

  mAbstractCamera = dynamic_cast<WbAbstractCamera *>(mDevice);
  connect(mDevice, &WbRenderingDevice::textureUpdated, this, &WbRenderingDeviceWindow::requestUpdate);
  connect(mDevice, &WbRenderingDevice::textureIdUpdated, this, &WbRenderingDeviceWindow::updateTextureGLId);
  connect(mDevice, &WbRenderingDevice::closeWindow, this, &WbRenderingDeviceWindow::closeFromMainWindow);
  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::mainRenderingEnded, this,
          &WbRenderingDeviceWindow::renderNow);
  const WbDisplay *display = dynamic_cast<WbDisplay *>(mDevice);
  if (display) {
    connect(display, &WbDisplay::attachedCameraChanged, this, &WbRenderingDeviceWindow::listenToBackgroundImageChanges);
    listenToBackgroundImageChanges(NULL, display->attachedCamera());
  }

  // set initial size
  double pixelSize = mDevice->pixelSize();
  const double textureWidth = mDevice->width();
  const double textureHeight = mDevice->height();
  double windowWidth = textureWidth * pixelSize;
  double windowHeight = textureHeight * pixelSize;
  QSize minSize = minimumSize();
  if (windowWidth < minSize.width()) {
    double newPixelSize = windowWidth / textureWidth;
    windowWidth = textureWidth * newPixelSize;
    windowHeight = textureHeight * newPixelSize;
  }

  if (windowHeight < minSize.height()) {
    double newPixelSize = windowHeight / textureHeight;
    windowWidth = textureWidth * newPixelSize;
    windowHeight = textureHeight * newPixelSize;
  }

  const WbRobot *const robotNode = WbNodeUtilities::findRobotAncestor(mDevice);
  assert(robotNode);
  setTitle(robotNode->name() + ": " + mDevice->name());
  resize(windowWidth, windowHeight);
}

WbRenderingDeviceWindow::~WbRenderingDeviceWindow() {
  if (!mContext)
    return;

  if (!isVisible())
    show();  // if the window is not exposed mContext->makeCurrent() doesn't work

  const bool success = mContext->makeCurrent(this);
  assert(success);
  if (!success)
    return;
  QOpenGLFunctions_3_3_Core *f = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(mContext);
  f->glDeleteVertexArrays(1, &mVaoId);
  f->glDeleteBuffers(2, (GLuint *)&mVboId);
  mContext->doneCurrent();
  delete mVboId;
}

void WbRenderingDeviceWindow::initialize() {
  if (mProgram == NULL)
    mProgram = new QOpenGLShaderProgram(this);
  else
    mProgram->removeAllShaders();
  mProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, gVertexShaderSource);
  if (mAbstractCamera && mAbstractCamera->isRangeFinder())
    mProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, gDepthFragmentShaderSource);
  else if (mBackgroundTextureGLId)
    mProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, gBackgroundFragmentShaderSource);
  else if (mForegroundTextureGLId || mMaskTextureGLId)
    mProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, gForegroundFragmentShaderSource);
  else
    mProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, gStdFragmentShaderSource);
  mProgram->link();
  mMaxRangeUniform = mProgram->uniformLocation("maxRange");
  mImageUniform = mProgram->uniformLocation("image");
  mBackgroundTextureUniform = mProgram->uniformLocation("background");
  mForegroundTextureUniform = mProgram->uniformLocation("foreground");
  mMaskTextureUniform = mProgram->uniformLocation("mask");
  mActiveTexturesUniform = mProgram->uniformLocation("activeTextures");

  if (!mDevice->hasBeenSetup())
    return;

  QOpenGLFunctions_3_3_Core *f = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(mContext);
  if (mAbstractCamera == NULL) {
    GLint textureWidth = 0;
    GLint textureHeight = 0;
    f->glBindTexture(GL_TEXTURE_2D, mTextureGLId);
    f->glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &textureWidth);
    f->glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &textureHeight);
    mXFactor = ((float)mDevice->width()) / textureWidth;
    mYFactor = ((float)mDevice->height()) / textureHeight;
  } else {
    mXFactor = 1.0f;
    mYFactor = 1.0f;
  }

  static const GLfloat vertices[] = {-1.0f, -1.0f, 1.0f, 1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f, 1.0f};

  static GLfloat const texCoords[] = {0.0f, mYFactor, mXFactor, 0.0,      0.0f,     0.0,
                                      0.0f, mYFactor, mXFactor, mYFactor, mXFactor, 0.0};

  if (!mVboId) {
    mVboId = new GLuint[2];
    f->glGenVertexArrays(1, &mVaoId);
    f->glGenBuffers(2, mVboId);
  }
  f->glBindVertexArray(mVaoId);
  f->glBindBuffer(GL_ARRAY_BUFFER, mVboId[0]);
  f->glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  f->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
  f->glEnableVertexAttribArray(0);
  f->glBindBuffer(GL_ARRAY_BUFFER, mVboId[1]);
  f->glBufferData(GL_ARRAY_BUFFER, sizeof(texCoords), texCoords, GL_STATIC_DRAW);
  f->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
  f->glEnableVertexAttribArray(1);

  mInitialized = true;
}

void WbRenderingDeviceWindow::render() {
  QOpenGLFunctions_3_3_Core *f = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(mContext);

  const int ratio = (int)devicePixelRatio();
  f->glViewport(0, 0, width() * ratio, height() * ratio);

  f->glClear(GL_COLOR_BUFFER_BIT);

  mProgram->bind();

  f->glBindVertexArray(mVaoId);

  if (mAbstractCamera && mAbstractCamera->isRangeFinder())
    mProgram->setUniformValue(mMaxRangeUniform, static_cast<float>(mAbstractCamera->maxRange()));

  if (mBackgroundTextureGLId) {
    mProgram->setUniformValue(mImageUniform, 0);
    mProgram->setUniformValue(mBackgroundTextureUniform, 1);
  }

  if (mForegroundTextureGLId || mMaskTextureGLId) {
    mProgram->setUniformValue(mImageUniform, 0);
    mProgram->setUniformValue(mMaskTextureUniform, 1);
    mProgram->setUniformValue(mForegroundTextureUniform, 2);
    mProgram->setUniformValue(mActiveTexturesUniform,
                              (mForegroundTextureUniform ? 0x10 : 0x0) | (mMaskTextureGLId ? 0x1 : 0x0));
  }

  f->glActiveTexture(GL_TEXTURE0);
  f->glBindTexture(GL_TEXTURE_2D, mTextureGLId);
  f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  if (mBackgroundTextureGLId) {
    f->glActiveTexture(GL_TEXTURE1);
    f->glBindTexture(GL_TEXTURE_2D, mBackgroundTextureGLId);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  }

  if (mMaskTextureGLId) {
    f->glActiveTexture(GL_TEXTURE1);
    f->glBindTexture(GL_TEXTURE_2D, mMaskTextureGLId);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  }

  if (mForegroundTextureGLId) {
    f->glActiveTexture(GL_TEXTURE2);
    f->glBindTexture(GL_TEXTURE_2D, mForegroundTextureGLId);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  }

  f->glDrawArrays(GL_TRIANGLES, 0, 6);

  mProgram->release();
}

void WbRenderingDeviceWindow::renderNow() {
  if (!mUpdateRequested || !isExposed())
    return;

  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->startMeasure(WbPerformanceLog::DEVICE_WINDOW_RENDERING, mDevice->deviceName() + " window");

  if (!mContext) {
    mContext = new QOpenGLContext(this);
    mContext->setFormat(requestedFormat());
    mContext->setShareContext(cMainOpenGLContext);
    mContext->create();
  }

#ifndef NDEBUG
  const bool success =
#endif  // NDEBUG
    mContext->makeCurrent(this);
  assert(success);

  if (!mInitialized)
    initialize();

  render();

  mContext->swapBuffers(this);

  mContext->doneCurrent();

  mUpdateRequested = false;

  if (log)
    log->stopMeasure(WbPerformanceLog::DEVICE_WINDOW_RENDERING, mDevice->deviceName() + " window");
}

bool WbRenderingDeviceWindow::event(QEvent *event) {
  switch (event->type()) {
    case QEvent::UpdateRequest:
    case QEvent::Expose:
      mUpdateRequested = true;
      renderNow();
      return true;
    case QEvent::Show:
      mShowOverlayOnClose = true;
      mDevice->enableExternalWindow(true);
      if (!mPreviousGeometry.isNull())
        setGeometry(mPreviousGeometry);
      return QWindow::event(event);
    case QEvent::Close:
      mPreviousGeometry = geometry();
      if (mShowOverlayOnClose)
        mDevice->enableExternalWindow(false);
    default:
      return QWindow::event(event);
  }
}

void WbRenderingDeviceWindow::closeFromMainWindow() {
  mShowOverlayOnClose = false;
  close();
}

void WbRenderingDeviceWindow::requestUpdate() {
  mUpdateRequested = true;
}

int WbRenderingDeviceWindow::deviceId() const {
  return mDevice->uniqueId();
}

void WbRenderingDeviceWindow::updateTextureGLId(int id, WbRenderingDevice::TextureRole role) {
  switch (role) {
    case WbRenderingDevice::BACKGROUND_TEXTURE:
      mBackgroundTextureGLId = id;
      break;
    case WbRenderingDevice::MAIN_TEXTURE:
      mTextureGLId = id;
      break;
    case WbRenderingDevice::MASK_TEXTURE:
      mMaskTextureGLId = id;
      break;
    case WbRenderingDevice::FOREGROUND_TEXTURE:
      mForegroundTextureGLId = id;
      break;
    default:
      assert(false);
  }
  mUpdateRequested = true;
  mInitialized = false;
}

void WbRenderingDeviceWindow::listenToBackgroundImageChanges(const WbRenderingDevice *previousAttachedDevice,
                                                             const WbRenderingDevice *newAttachedDevice) {
  if (previousAttachedDevice)
    disconnect(previousAttachedDevice, &WbRenderingDevice::textureUpdated, this, &WbRenderingDeviceWindow::requestUpdate);
  if (newAttachedDevice)
    connect(newAttachedDevice, &WbRenderingDevice::textureUpdated, this, &WbRenderingDeviceWindow::requestUpdate);
}

QStringList WbRenderingDeviceWindow::perspective() const {
  QStringList windowPerspective;
  QRect windowGeometry = geometry();
  windowPerspective << QString::number(windowGeometry.x());
  windowPerspective << QString::number(windowGeometry.y());
  windowPerspective << QString::number(windowGeometry.width());
  windowPerspective << QString::number(windowGeometry.height());

  int state = Qt::WindowNoState;
  if (windowState() & Qt::WindowMaximized)
    state &= Qt::WindowMaximized;
  else if (windowState() & Qt::WindowFullScreen)
    state &= Qt::WindowFullScreen;
  windowPerspective << QString::number(state);
  return windowPerspective;
}

void WbRenderingDeviceWindow::restorePerspective(const QStringList &perspective) {
  assert(perspective.size() >= 5);

  int x = perspective[0].toInt();
  int y = perspective[1].toInt();
  int width = perspective[2].toInt();
  int height = perspective[3].toInt();
  QRect windowGeometry(x, y, width, height);

  setGeometry(windowGeometry);
  setWindowState((Qt::WindowState)perspective[4].toInt());
}
