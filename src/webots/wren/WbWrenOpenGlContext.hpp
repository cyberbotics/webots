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

#ifndef WB_OPEN_GL_CONTEXT_HPP
#define WB_OPEN_GL_CONTEXT_HPP

#include <QtGui/QOpenGLContext>

#include <cassert>

//
// Description: Wrapper for QOpenGLContext to keep the WREN context current
// An instance of WbWrenOpenGlContext is created during initialization of the main window,
// and allows modules that don't have access to the GUI module to make the main context current.
//
class WbWrenOpenGlContext : public QOpenGLContext {
  Q_OBJECT
public:
  explicit WbWrenOpenGlContext(QObject *parent) : QOpenGLContext(parent) {}

  // nop: bypass the make/doneCurrent() called by Qt
  // in order to keep always the Wren OpenGLContext as
  // the main context
  bool makeCurrent(QSurface *surface) { return true; }
  void doneCurrent() {}
  bool forceMakeCurrent(QSurface *surface) { return QOpenGLContext::makeCurrent(surface); }

  static WbWrenOpenGlContext *instance() {
    assert(mWrenContext);
    return mWrenContext;
  }

  static void init(QObject *parent, QSurface *wrenSurface, QSurfaceFormat &format) {
    mWrenContext = new WbWrenOpenGlContext(parent);
    mWrenContext->setFormat(format);
    mWrenContext->create();

    mWrenSurface = wrenSurface;
  }

  static void destroy();

  // Makes WREN's OpenGL context current and marks it as active.
  // WREN will immediately apply changes to OpenGL state until the context has been marked as inactive.
  static bool makeWrenCurrent();

  // Marks WREN's OpenGL context as inactive, preventing WREN from making OpenGL calls until it is marked as active again.
  // Any call to makeWrenCurrent should be followed by a call to doneWren before any possible OpenGL context changes.
  static void doneWren();

  static bool isCurrent();

private:
  static bool mIsCurrent;
  static QStack<bool> mPreviousState;

  static WbWrenOpenGlContext *mWrenContext;
  static QSurface *mWrenSurface;
};

#endif
