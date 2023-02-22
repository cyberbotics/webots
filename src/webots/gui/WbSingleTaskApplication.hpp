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

#ifndef WB_SINGLE_TASK_APPLICATION_HPP
#define WB_SINGLE_TASK_APPLICATION_HPP

//
// Description: object executing the task defined within the start options
//              and requesting the Webots application to exit immediately
//

#include "WbGuiApplication.hpp"

#include <QtCore/QString>

class WbSingleTaskApplication : public QObject {
  Q_OBJECT

public:
  explicit WbSingleTaskApplication(WbGuiApplication::Task task, const QStringList &taskArgument = QStringList(),
                                   QObject *parent = 0, const QString &startupPath = QString()) :
    QObject(parent),
    mTask(task),
    mTaskArguments(taskArgument),
    mStartupPath(startupPath) {}

public slots:
  void run();

signals:
  void finished(int returnCode);

private:
  WbGuiApplication::Task mTask;
  QStringList mTaskArguments;
  QString mStartupPath;

  void convertProto() const;
  void showHelp() const;
  void showSysInfo() const;
};

#endif
