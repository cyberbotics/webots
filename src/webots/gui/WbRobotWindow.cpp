// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "WbRobotWindow.hpp"

#include "WbApplicationInfo.hpp"
#include "WbLog.hpp"
#include "WbProject.hpp"
#include "WbRobot.hpp"
#include "WbRobotWindowTransportLayer.hpp"
#include "WbStandardPaths.hpp"
#include "WbVersion.hpp"
#include "WbWebPage.hpp"

#include <QtCore/qdebug.h>
#include <QtCore/QCoreApplication>
#include <QtCore/QFile>
#include <QtCore/QRegularExpression>
#include <QtCore/QTextStream>
#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>
#include <QtWebSockets/QWebSocket>

#ifdef _WIN32
#include <QtWebKitWidgets/QWebFrame>
#include <QtWebKitWidgets/QWebView>
#else
#include <QtWebChannel/QWebChannel>
#include <QtWebEngineWidgets/QWebEngineView>
#endif

// Debug code: uncomment to show a web inspector for QtWebKit.
// #include <QtWebKitWidgets/QWebInspector>
// #include <QtWidgets/QDialog>
// #include <QtWidgets/QVBoxLayout>

WbRobotWindow::WbRobotWindow(WbRobot *robot) : mRobot(robot), mResetCount(0) {
  QString title = "Robot: " + robot->name();

  const QString &windowFileName = robot->windowFile("html");
  if (windowFileName.isEmpty()) {
    robot->parsingWarn(QString("No dockable HTML robot window is set in the 'window' field."));
    return;
  }

  connect(robot, &WbRobot::sendToJavascript, this, &WbRobotWindow::sendToJavascript);
}

void WbRobotWindow::setupPage() {
  QString windowFileName = mRobot->windowFile("html");
  if (windowFileName.isEmpty()) {
    mRobot->parsingWarn(QString("No dockable HTML robot window is set in the 'window' field."));
    return;
  }
  mResetCount++;
  windowFileName = windowFileName.mid(windowFileName.indexOf("/robot_windows"));  // remove content before robot_windows

  qDebug() << "windowFileName:"
           << "http://localhost:1234" + windowFileName;
  QDesktopServices::openUrl(QUrl("http://localhost:1234" + windowFileName));

  QWebChannel *channel = new QWebChannel();
  // mWebView->page()->setWebChannel(channel);
  mTransportLayer = new WbRobotWindowTransportLayer();
  channel->registerObject("_webots", mTransportLayer);
  connect(mTransportLayer, &WbRobotWindowTransportLayer::ackReceived, this, &WbRobotWindow::notifyAckReceived);  // TODO
  connect(mTransportLayer, &WbRobotWindowTransportLayer::javascriptReceived, mRobot, &WbRobot::receiveFromJavascript);
}
#ifndef _WIN32
void WbRobotWindow::notifyLoadCompleted() {
  mLoaded = true;
  if (!mWaitingSentMessages.isEmpty()) {
    foreach (const QString &message, mWaitingSentMessages)
      runJavaScript(message);
    mWaitingSentMessages.clear();
  }
}

void WbRobotWindow::runJavaScript(const QString &message) {  // TODO: send this message to robot_window.
  QString jsMessage = "window.robot_window.receive('" + message + "', '" + escapeString(robot()->name()) + "')";
  //qDebug() << "runJavaScript:" << jsMessage;
  mTransportLayer->requestAck();
  //WbStreamingServer->sendTextMessage(jsMessage);
  // webots->view->runJavaScript("webots.Window.receive('" + message + "', '" + escapeString(robot()->name()) + "')");
}
#endif

void WbRobotWindow::notifyAckReceived() {
  mRobot->setWaitingForWindow(false);
}

QString WbRobotWindow::escapeString(const QString &text) {
  QString escaped(text);
  escaped.replace("\\", "\\\\\\\\");
  escaped.replace("'", "\\'");
  return escaped;
}

void WbRobotWindow::sendToJavascript(const QByteArray &string) {
  const QString &message(escapeString(string));
#ifdef _WIN32
  mFrame->evaluateJavaScript("webots.Window.receive('" + message + "', '" + escapeString(robot()->name()) + "')");
#else
  // mRobot->setWaitingForWindow(true); //TODO
  if (mLoaded)
    runJavaScript(message);
  else  // message will be sent once the robot window loading is completed
    mWaitingSentMessages << message;
#endif
}

#ifdef _WIN32
void WbRobotWindow::receiveFromJavascript(const QByteArray &message) {
  mRobot->receiveFromJavascript(message);
}
#endif
