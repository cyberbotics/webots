#include "MainApplication.hpp"
#include "StandardPaths.hpp"

#include <QtCore/QAbstractEventDispatcher>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QSocketNotifier>

#include <QtGui/QIcon>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMessageBox>

#ifdef _WIN32
#include <windows.h>
#elif defined(__APPLE__)
#include <sched.h>
#endif

using namespace webotsQtUtils;

MainApplication::MainApplication() {
  static int c = 1;
  static const char *v[1] = {"qt_utils"};

  // add the qt plugins library before instantiating QApplication.
  // this is required for loading libqcocoa.dylib, libqxcb.so and qwindows.dll
  QApplication::addLibraryPath(StandardPaths::getWebotsHomePath() + "lib/webots/qt/plugins");

  // set icon paths
  QDir::addSearchPath("icons", StandardPaths::getCurrentLibraryPath() + "icons");
  QDir::addSearchPath("icons", StandardPaths::getWebotsHomePath() + "resources/icons/dark");

  // init application
  mMainApplicationPrivate = new MainApplicationPrivate(c, const_cast<char **>(v));
  QApplication *app = static_cast<QApplication *>(QApplication::instance());
  app->setWindowIcon(QIcon("icons:webots.png"));
}

MainApplication::~MainApplication() {
  if (mMainApplicationPrivate) {
    delete mMainApplicationPrivate;
    mMainApplicationPrivate = NULL;
  }
}

void MainApplication::preUpdateGui() {
  if (mMainApplicationPrivate && mMainApplicationPrivate->isInitialized())
    mMainApplicationPrivate->preUpdateGui();
}

void MainApplication::updateGui() {
  if (mMainApplicationPrivate && mMainApplicationPrivate->isInitialized())
    mMainApplicationPrivate->updateGui();
}

bool MainApplication::isInitialized() const {
  if (mMainApplicationPrivate)
    return mMainApplicationPrivate->isInitialized();
  return false;
}

MainApplicationPrivate::MainApplicationPrivate(int &argc, char **argv) : QApplication(argc, argv), mIsInitialized(false) {
  mPipeInHandler = QString(qgetenv("WEBOTS_PIPE_IN")).toInt();

  if (mPipeInHandler == 0) {
    QMessageBox::critical(NULL, tr("Robo.window"), tr("the pipe handle is not well defined"));
    return;
  }

  mIsLeaving = true;

#ifndef _WIN32
  mPipeInFile = new QFile();
  mPipeInFile->open(mPipeInHandler, QIODevice::ReadOnly);

  mSocketNotifier = new QSocketNotifier(mPipeInFile->handle(), QSocketNotifier::Read);
  mSocketNotifier->setEnabled(false);
  connect(mSocketNotifier, SIGNAL(activated(int)), this, SLOT(dataReadyOnPipeIn()));
#endif

  // the application has been initialized without problems
  mIsInitialized = true;
}

MainApplicationPrivate::~MainApplicationPrivate() {
  if (isInitialized()) {
#ifndef _WIN32
    delete mSocketNotifier;
    delete mPipeInFile;
#endif
  }
}

void MainApplicationPrivate::preUpdateGui() {
  mIsLeaving = false;
#ifndef _WIN32
  mSocketNotifier->setEnabled(true);
#endif
}

#ifdef _WIN32
bool MainApplicationPrivate::isMainWindowVisible() const {
  foreach (QWidget *widget, QApplication::allWidgets()) {
    QMainWindow *window = dynamic_cast<QMainWindow *>(widget);
    if (window)
      return window->isVisible();
  }
  return false;
}
#endif

void MainApplicationPrivate::updateGui() {
  if (!mIsInitialized)
    return;

#ifdef _WIN32
  // don't process the events if the GUI is hidden
  // otherwise the performence loss is significative
  // the events are processed just after the hidding
  if (!isMainWindowVisible())
    return;
#endif

// process events until there is something on the pipe
#ifdef _WIN32
  DWORD bytesAvailable = 0;
  while (bytesAvailable == 0) {
    processEvents();
    PeekNamedPipe((HANDLE)mPipeInHandler, NULL, 0, NULL, &bytesAvailable, NULL);
    if (bytesAvailable == 0)
      Sleep(1);
  }
#else
  while (!mIsLeaving)
    processEvents(QEventLoop::WaitForMoreEvents);

#ifdef __APPLE__
  // in some rare cases, the process can saturate the processor
  // if it is not yield.
  // Especially, a customer was not able to run 2 e-pucks in remote-control
  // smoothly on Mac due to this. But similar issues were observed in some
  // other robot windows.
  sched_yield();
#endif

#endif
}

#ifndef _WIN32
// if this signal is called, there is something on the pipe
void MainApplicationPrivate::dataReadyOnPipeIn() {
  mIsLeaving = true;

  mSocketNotifier->setEnabled(false);

#ifdef __APPLE__
  // on Mac, processEvents isn't interrupt automatically after this function call
  QAbstractEventDispatcher *disp = QAbstractEventDispatcher::instance();
  disp->interrupt();
#endif
}
#endif
