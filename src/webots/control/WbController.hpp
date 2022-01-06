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

#ifndef WB_CONTROLLER_HPP
#define WB_CONTROLLER_HPP

#include <QtCore/QObject>
#include <QtCore/QProcess>
#include <QtCore/QTextDecoder>
#include "WbFileUtil.hpp"
#include "WbRobot.hpp"

class QLocalSocket;
class QProcessEnvironment;

class WbController : public QObject {
  Q_OBJECT

public:
  // constructor & destructor
  // name: controller name as in Robot.controller, e.g. "void"
  // arguments: controller arguments as in Robot.controllerArgs
  explicit WbController(WbRobot *robot);
  virtual ~WbController();

  // start the controller
  // it never fails: the void controller is started as a fallback
  void start();

  void setSocket(QLocalSocket *socket);
  void writeAnswer(bool immediateAnswer = false);
  void writePendingImmediateAnswer() {
    if (mHasPendingImmediateAnswer)
      writeImmediateAnswer();
  }
  void flushBuffers() {
    if (mStderrNeedsFlush)
      flushBuffer(&mStderrBuffer);
    if (mStdoutNeedsFlush)
      flushBuffer(&mStdoutBuffer);
  }
  WbRobot *robot() const { return mRobot; }
  int robotId() const;
  const QString &name() const;
  bool synchronization() const { return mRobot->synchronization(); }
  double requestTime() const { return mRequestTime; }
  void resetRequestTime();
  bool isIncompleteRequest() const { return mIncompleteRequest; }
  unsigned int deltaTimeRequested() const { return mDeltaTimeRequested; }
  bool isRequestPending() const { return mRequestPending; }
  bool isRunning() const;
  bool isProcessingRequest() const { return mProcessingRequest; }

signals:
  void hasTerminatedByItself(WbController *);
  void requestReceived();

public slots:
  void readRequest();
  void appendMessageToConsole(const QString &message, bool useStdout);
  void writeUserInputEventAnswer();
  void handleControllerExit();

private:
  WbRobot *mRobot;
  WbFileUtil::FileType mType;
  QString mControllerPath;  // path where the controller file is located
  QString mName;            // controller name, e.g. "void"
  QString mCommand;         // command to be executed, e.g. "java"
  QStringList mArguments;   // command arguments
  QString mJavaCommand;
  QString mJavaOptions;
  QString mPythonCommand;
  QString mPythonShortVersion;
  QString mPythonOptions;
  QString mMatlabCommand;
  QString mMatlabOptions;
  QProcess *mProcess;
  QLocalSocket *mSocket;
  QByteArray mRequest;
  QTextDecoder mStdout;
  QTextDecoder mStderr;
  double mRequestTime;
  bool mHasBeenTerminatedByItself;
  bool mIncompleteRequest;
  unsigned int mDeltaTimeRequested;
  double mDeltaTimeMeasured;
  bool mRequestPending;
  bool mProcessingRequest;
  bool mHasPendingImmediateAnswer;

  QString mStdoutBuffer;
  QString mStderrBuffer;
  bool mStdoutNeedsFlush;
  bool mStderrNeedsFlush;

  void addToPathEnvironmentVariable(QProcessEnvironment &env, const QString &key, const QString &value, bool override,
                                    bool shouldPrepend = false);
  bool removeFromPathEnvironmentVariable(QProcessEnvironment &env, const QString &key, const QString &value);
  void setProcessEnvironment();
  void updateName(const QString &name);

  WbFileUtil::FileType findType(const QString &controllerPath);
  void startExecutable();
  void startVoidExecutable();
  void startJava(bool jar = false);
  void startPython();
  void startMatlab();
  void startBotstudio();
  void copyBinaryAndDependencies(const QString &filename);
  void appendMessageToBuffer(const QString &message, QString *buffer);
  void flushBuffer(QString *buffer);
  QString commandLine() const;

private slots:
  void readStdout();
  void readStderr();
  void info(const QString &message);
  void warn(const QString &message);
  void error(const QString &message);
  void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void processError(QProcess::ProcessError error);
  void reportControllerNotFound();
  void reportFailedStart();
  void reportMissingCommand(const QString &command);
  void robotDestroyed();
  void writeImmediateAnswer();
};

#endif
