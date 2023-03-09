#include "MainWindow.hpp"

#include <QtCore/QRandomGenerator>
#include <QtCore/QTime>

#include <QtGui/QCloseEvent>
#include <QtGui/QScreen>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>

#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <X11/Xlib.h>
#endif

using namespace webotsQtUtils;

MainWindow::MainWindow() : QMainWindow() {
  setUnifiedTitleAndToolBarOnMac(true);

  // hidden until show is called by Webots
  hide();
}

MainWindow::~MainWindow() {
}

void MainWindow::closeEvent(QCloseEvent *event) {
  // the close event is ignored, the robot window is only hidden
  hide();
  event->ignore();
}

void MainWindow::showWindow() {
  static bool firstShow = true;

  // compromise on each platform to show and
  // activate the window after this function call
#ifdef _WIN32
  if (!isVisible())
    show();
  else if (isMinimized())
    showNormal();

  HWND wnd = (HWND)winId();
  ::SetWindowPos(wnd,
                 HWND_TOPMOST,  // this call brings the window on top
                 0, 0, 0, 0, SWP_SHOWWINDOW | SWP_NOSIZE | SWP_NOMOVE);
  ::SetWindowPos(wnd,
                 HWND_NOTOPMOST,  // remove this flag to remove the stay on top effect. This solved issues with dialogs focus
                                  // (i.e. PoseEditor)
                 0, 0, 0, 0, SWP_SHOWWINDOW | SWP_NOSIZE | SWP_NOMOVE);

  setWindowState(windowState() | Qt::WindowActive);
#elif defined(__APPLE__)
  show();
  raise();
#else  // __linux__
  if (isMinimized()) {
    Display *display = XOpenDisplay(NULL);
    XMapWindow(display, winId());
    XSync(display, false);

  } else {
    if (isVisible())
      setVisible(false);
    show();
    raise();
  }

  activateWindow();
#endif

  // center the window
  if (firstShow) {
    const int MAX_OFFSET = 50;
    const QRect &desktopRect = QGuiApplication::primaryScreen()->geometry();
    const QSize &windowSize = size();
    const QPoint offset(QRandomGenerator::global()->bounded(MAX_OFFSET) - MAX_OFFSET / 2,
                        QRandomGenerator::global()->bounded(MAX_OFFSET) - MAX_OFFSET / 2);

    move(desktopRect.x() + desktopRect.width() / 2 - windowSize.width() / 2 + offset.x(),
         desktopRect.y() + desktopRect.height() / 2 - windowSize.height() / 2 + offset.y());
    firstShow = false;
  }

#ifdef _WIN32
  // to not freeze the GUI because
  // the events are not processed if the
  // GUI is hidden
  processApplicationEvents();
#endif
}

#ifdef _WIN32
void MainWindow::hideEvent(QHideEvent *event) {
  QMainWindow::hideEvent(event);
  processApplicationEvents();
}

void MainWindow::showEvent(QShowEvent *event) {
  QMainWindow::showEvent(event);
  processApplicationEvents();
}

void MainWindow::processApplicationEvents() {
  QCoreApplication::instance()->processEvents();
}
#endif
