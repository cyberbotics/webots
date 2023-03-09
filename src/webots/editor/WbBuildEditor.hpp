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

#ifndef WB_BUILD_EDITOR_HPP
#define WB_BUILD_EDITOR_HPP

//
// Description: Text editor with external make process
//

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QProcess>
#include <QtCore/QStringList>
#include "WbTextEditor.hpp"

class QAction;

class WbBuildEditor : public WbTextEditor {
  Q_OBJECT

public:
  // singleton
  static WbBuildEditor *instance();

  explicit WbBuildEditor(QWidget *parent, const QString &toolBarAlign);
  virtual ~WbBuildEditor();

  // open specified file, show line if specified, show exact word if column specified
  void jumpToError(QString fileName, int line = -1, int column = -1);

  // unmark error line/word
  void unmarkError();

  QAction *buildAction() const { return mBuildAction; }
  QAction *cleanAction() const { return mCleanAction; }
  QAction *makeJarAction() const { return mMakeJarAction; }
  QAction *crossCompileAction() const { return mCrossCompileAction; }
  QAction *cleanCrossCompilationAction() const { return mCleanCrossCompilationAction; }

  void updateGui() override;

signals:
  void reloadRequested();
  void resetRequested();

private:
  QAction *mBuildAction, *mCleanAction, *mMakeJarAction;
  QAction *mCrossCompileAction, *mCleanCrossCompilationAction;
  QProcess *mProcess;
  QString mCrossCompileMakefile;
  QDateTime mTargetModificationTimeBeforeMake;
  QString mTargetFile;
  bool mIsCleaning;

  void make(const QString &target);
  void cleanupProcess();
  bool makefileExists(const QString &compilePath);
  void addMakefileIfNecessary(const QString &compilePath);
  void createActions();
  void updateBuildButtons();
  void computeTargetFile();
  void updateTargetModificationTime();
  void reloadMessageBoxIfNeeded();
  const QDir compileDir() const;
  QStringList getJavaCommandLine(const QString &target) const;

private slots:
  void readStdout();
  void readStderr();
  void build();
  void clean();
  void makeJar();
  void crossCompile();
  void cleanCrossCompilation();
  void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
};

#endif
