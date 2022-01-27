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

#include "WbProjectRelocationDialog.hpp"

#include "WbFileUtil.hpp"
#include "WbLanguage.hpp"
#include "WbLineEdit.hpp"
#include "WbMessageBox.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbProtoModel.hpp"
#include "WbRobot.hpp"
#include "WbSimulationState.hpp"
#include "WbStandardPaths.hpp"
#include "WbWorld.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>

QString WbProjectRelocationDialog::mExternalProtoProjectPath = QString();

WbProjectRelocationDialog::WbProjectRelocationDialog(WbProject *project, const QString &relativeFilename,
                                                     const QString &absoluteFilePath, QWidget *parent) :
  QDialog(parent),
  mProject(project),
  mRelativeFilename(relativeFilename),
  mAbsoluteFilePath(absoluteFilePath),
  mSourceEdit(NULL),
  mTargetEdit(NULL),
  mProtoCheckBox(NULL),
  mPluginsCheckBox(NULL),
  mFilesLabel(NULL),
  mConclusionLabel(NULL),
  mStatusEdit(NULL),
  mSelectButton(NULL),
  mCancelButton(NULL),
  mCopyButton(NULL),
  mButtonBox(NULL) {
  setWindowTitle(tr("Project relocation"));

  mIsProtoModified = WbLanguage::findByFileName(mRelativeFilename)->code() == WbLanguage::PROTO;
  mProtoCheckBox = new QCheckBox(tr("Include all PROTO files"), this);
  mProtoCheckBox->setChecked(mIsProtoModified);
  mPluginsCheckBox = new QCheckBox(tr("Include all plugins files"), this);
  const QString &absoluteFilename = mAbsoluteFilePath + mRelativeFilename;
  const bool isPluginModified = WbFileUtil::isLocatedInDirectory(absoluteFilename, mProject->path() + "plugins") ||
                                (!mExternalProtoProjectPath.isEmpty() &&
                                 WbFileUtil::isLocatedInDirectory(absoluteFilename, mExternalProtoProjectPath + "plugins"));
  mPluginsCheckBox->setChecked(mIsProtoModified || isPluginModified);

  mButtonBox = new QDialogButtonBox(this);
  mCancelButton = mButtonBox->addButton(QDialogButtonBox::Cancel);
  mCopyButton = mButtonBox->addButton(tr("Copy"), QDialogButtonBox::AcceptRole);
  connect(mCancelButton, &QPushButton::pressed, this, &WbProjectRelocationDialog::reject);
  connect(mCopyButton, &QPushButton::pressed, this, &WbProjectRelocationDialog::copy);

  mStatusEdit = new QPlainTextEdit(this);
  mStatusEdit->setReadOnly(true);

  mIsCompleteRelocation = mProject->isReadOnly();

  if (mIsCompleteRelocation)
    initCompleteRelocation();
  else
    initProtoSourceRelocation();
}

void WbProjectRelocationDialog::initCompleteRelocation() {
  QLabel *title = new QLabel(tr("<b>Copy necessary files from source to target directory?</b>"), this);

  const QString &sourcePath = QDir::toNativeSeparators(mProject->path());
  mTargetPath =
    QDir::toNativeSeparators(WbPreferences::instance()->value("Directories/projects").toString() + mProject->dirName());

  mSourceEdit = new WbLineEdit(sourcePath, this);
  mSourceEdit->setReadOnly(true);
  mSourceEdit->setMinimumWidth(sourcePath.length() * 8);

  mTargetEdit = new WbLineEdit(mTargetPath, this);
  connect(mTargetEdit, &WbLineEdit::textEdited, this, &WbProjectRelocationDialog::targetEdited);

  mSelectButton = new QPushButton(tr("Select"), this);
  connect(mSelectButton, &QPushButton::pressed, this, &WbProjectRelocationDialog::selectDirectory);

  QHBoxLayout *layout = new QHBoxLayout();
  layout->addWidget(mTargetEdit);
  layout->addWidget(mSelectButton);

  QFormLayout *formLayout = new QFormLayout();
  formLayout->addRow(tr("Source directory: "), mSourceEdit);
  formLayout->addRow(tr("Target directory: "), layout);

  setStatus(tr("Please select a target directory then push the [Copy] button."));

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(title);
  mainLayout->addLayout(formLayout);
  if (mProtoCheckBox)
    mainLayout->addWidget(mProtoCheckBox);
  if (mPluginsCheckBox)
    mainLayout->addWidget(mPluginsCheckBox);
  mainLayout->addWidget(mStatusEdit);
  mainLayout->addWidget(mButtonBox);
}

void WbProjectRelocationDialog::initProtoSourceRelocation() {
  QLabel *title = new QLabel(tr("<b>Copy necessary files from source to current project directory?</b>"), this);

  mTargetPath = mProject->path();

  mSourceEdit = new WbLineEdit(QDir::toNativeSeparators(mAbsoluteFilePath), this);
  mSourceEdit->setReadOnly(true);
  mSourceEdit->setMinimumWidth(mAbsoluteFilePath.length() * 8);

  mTargetEdit = new WbLineEdit(QDir::toNativeSeparators(mTargetPath), this);
  mTargetEdit->setReadOnly(true);
  mTargetEdit->setMinimumWidth(mTargetPath.length() * 8);

  QFormLayout *formLayout = new QFormLayout();
  formLayout->addRow(tr("Source directory: "), mSourceEdit);
  formLayout->addRow(tr("Target directory: "), mTargetEdit);

  setStatus(tr("Please select the copy options then push the [Copy] button."));

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(title);
  mainLayout->addLayout(formLayout);
  if (mProtoCheckBox)
    mainLayout->addWidget(mProtoCheckBox);
  if (mPluginsCheckBox)
    mainLayout->addWidget(mPluginsCheckBox);
  mainLayout->addWidget(mStatusEdit);
  mainLayout->addWidget(mButtonBox);
}

WbProjectRelocationDialog::~WbProjectRelocationDialog() {
}

void WbProjectRelocationDialog::setStatus(const QString &text, bool ok) {
  QTextCharFormat format;

  if (ok)
    format.setForeground(QBrush(Qt::black));
  else
    format.setForeground(QBrush(Qt::red));

  mStatusEdit->setCurrentCharFormat(format);
  mStatusEdit->setPlainText(text);

#ifdef __APPLE__
  // Qt bug on macOS: the QDialog::exec() doesn't deal correctly the repaint events as it should.
  repaint();
#endif
}

void WbProjectRelocationDialog::targetEdited(const QString &text) {
  mTargetPath = QDir::fromNativeSeparators(text);
  setStatus(tr("Push the [Copy] button to copy the project."));
}

void WbProjectRelocationDialog::copy() {
  const QString &home = WbStandardPaths::webotsHomePath();
  if (WbFileUtil::isLocatedInDirectory(mTargetPath, home)) {
    setStatus(tr("Target directory is located in the Webots installation directory.") + "\n" +
                tr("Please select another target directory."),
              false);
    return;
  }

  QDir dir(mTargetPath);
  if (mIsCompleteRelocation) {
    if (dir.exists()) {
      if (dir.count() > 2) {  // 2 because special cases: "." and ".." are counted
        setStatus(tr("Target directory is not empty.") + "\n" + tr("Please select another directory."), false);
        return;
      }
    } else {
      if (!QDir::root().mkpath(mTargetPath)) {
        setStatus(
          tr("Could not create target directory.") + "\n" + tr("Make sure that you have write access at this location."),
          false);
        return;
      }
    }
  }

  int copiedFilesCount = 0;
  if (mIsCompleteRelocation) {
    copiedFilesCount += copyWorldFiles();
    QString currentProjectPath(QDir(mProject->path()).absolutePath());
    if (!currentProjectPath.endsWith("/"))
      currentProjectPath += "/";
    copiedFilesCount += copyProject(currentProjectPath, false);
  }
  if (!mExternalProtoProjectPath.isEmpty())
    copiedFilesCount += copyProject(mExternalProtoProjectPath, mIsProtoModified);

  if (copiedFilesCount == 0) {
    setStatus(tr("Project relocation failed.") + "\n" + tr("Some files or directories could not be copied."));
    return;
  }

  // change buttons
  if (mSelectButton)
    mSelectButton->setEnabled(false);
  mSourceEdit->setEnabled(false);
  mTargetEdit->setEnabled(false);
  if (mProtoCheckBox)
    mProtoCheckBox->setEnabled(false);
  if (mPluginsCheckBox)
    mPluginsCheckBox->setEnabled(false);
  mButtonBox->removeButton(mCancelButton);
  mButtonBox->removeButton(mCopyButton);
  QPushButton *closeButton = mButtonBox->addButton(QDialogButtonBox::Close);
  closeButton->setFocus();
  connect(closeButton, &QPushButton::pressed, this, &WbProjectRelocationDialog::accept);

  mProject->setPath(dir.path());

  // store the accepted project directory in the preferences
  dir.cdUp();  // store the upper level, probably the path where the directories are stored
  WbPreferences::instance()->setValue("Directories/projects", dir.absolutePath() + "/");

  // good news
  setStatus(tr("Project successfully relocated.") + "\n" + tr("%1 file(s) copied.").arg(copiedFilesCount));
}

int WbProjectRelocationDialog::copyProject(const QString &projectPath, bool copyProtoProject) {
  // Copy modified project resources:
  // current project folder or project folder of an external PROTO model used in the current project.
  // When modifying any resource (world file, PROTO file, controller, plugins, etc.) all the other resources stored in
  // the current project path have to also be copied. Only non-modified PROTO related files stored in the standard paths can be
  // skipped.
  //
  // Here we should copy only the files that may be edited / changed by the user, that is:
  // 1) textures, meshes, and skins in `protos` folders if PROTO file was modified
  // 2) all the controllers (folders) and related libraries that was modified or used by this world and which are not located in
  //    the current project folder. Skip controllers used by PROTO models that have not been modified.
  // 3) possibly the current controller file (even if corresponding to a PROTO robot)
  // 4) project libraries if a controller or plugin is copied
  // 5) project motions folder if a controller is copied

  int result = 0;

  if (copyProtoProject) {
    // copy all PROTO textures
    result += WbFileUtil::copyDir(projectPath + "protos/textures", mTargetPath + "/protos/textures", true, true, true);
    result += WbFileUtil::copyDir(projectPath + "protos/meshes", mTargetPath + "/protos/meshes", true, true, true);
    result += WbFileUtil::copyDir(projectPath + "protos/skins", mTargetPath + "/protos/skins", true, true, true);
  }

  bool copyLibraries = false;
  bool projectLibrariesCopied = false;

  // copy only the needed robot controllers
  QStringList copiedControllers;
  const QList<WbRobot *> &robots = WbWorld::instance()->robots();
  foreach (WbRobot *robot, robots) {
    const WbProtoModel *proto = robot->proto();
    if (copyProtoProject && !proto)
      continue;

    const QString &controllerName = robot->controllerName();
    const QString &controllerPath = robot->controllerDir();
    const QString &projectControllerPath = projectPath + "controllers/" + controllerName + "/";
    bool isThisProtoModified = false;
    QDir protoProjectDir;
    if (proto) {
      protoProjectDir.setPath(QFileInfo(proto->fileName()).path());
      protoProjectDir.cdUp();
      const QString &protoControllerPath = protoProjectDir.path() + "/controllers/" + controllerName + "/";
      if (mIsProtoModified && proto->fileName() == (projectPath + mRelativeFilename))
        isThisProtoModified = true;
      else if ((mProtoCheckBox == NULL || !mProtoCheckBox->isChecked()) && protoControllerPath == projectControllerPath)
        // this controller is associated with a PROTO file that has not been modified
        // no need to copy it
        continue;
    }
    if (!isThisProtoModified && controllerPath != projectControllerPath)
      // the controller is not included in the project
      // maybe a default controller or a PROTO controller
      continue;

    if (copyProtoProject || !copiedControllers.contains(controllerName)) {
      result += WbFileUtil::copyDir(controllerPath, mTargetPath + "/controllers/" + controllerName, true, false, true);
      result += WbFileUtil::copyDir(controllerPath + "../motions", mTargetPath + "/motions", true, true, true);
      copiedControllers << controllerName;
    }
    if (proto) {
      result += WbFileUtil::copyDir(protoProjectDir.path() + "/libraries/", mTargetPath + "/libraries/", true, false, true);
      projectLibrariesCopied = projectLibrariesCopied || protoProjectDir.path() + '/' == projectPath;
    } else
      copyLibraries = true;
  }

  // copy the current source folder
  QString relativeDirPath = mRelativeFilename;
  if (QFileInfo(projectPath + relativeDirPath).isFile())
    // if it's not a directory, but a file, get the containing controller directory from after the "controllers/", "libraries/",
    // "protos/", etc. part of the string
    relativeDirPath = mRelativeFilename.left(mRelativeFilename.indexOf("/", mRelativeFilename.indexOf("/") + 1));
  const QString dstPath = mTargetPath + "/" + relativeDirPath;
  if (!QDir(dstPath).exists()) {
    result += WbFileUtil::copyDir(projectPath + relativeDirPath, dstPath, true, true, true);
    if (relativeDirPath.startsWith("controllers"))
      result += WbFileUtil::copyDir(projectPath + "motions", mTargetPath + "/motions", true, true, true);
  }

  if (mProtoCheckBox && mProtoCheckBox->isChecked())
    result += WbFileUtil::copyDir(projectPath + "protos", mTargetPath + "/protos", true, true, true);
  if (mPluginsCheckBox && mPluginsCheckBox->isChecked()) {
    result += WbFileUtil::copyDir(projectPath + "plugins", mTargetPath + "/plugins", true, false, true);
    copyLibraries = true;
  }

  if (copyLibraries && !projectLibrariesCopied)
    result += WbFileUtil::copyDir(projectPath + "libraries", mTargetPath + "/libraries", true, false, true);

  return result;
}

int WbProjectRelocationDialog::copyWorldFiles() {
  // Files to be copied:
  // 1) the current world file and associated wbproj file
  // 2) all the textures used explicitly by this world file and present in the local worlds
  //    folder (but not the ones used implicitly by PROTOs).

  const WbWorld *world = WbWorld::instance();
  const QString &worldFileBaseName = QFileInfo(world->fileName()).baseName();

  int result = 0;

  // copy the .wbt file, the .wbproj file
  QDir targetPathDir(mTargetPath + "/worlds");
  targetPathDir.mkpath(".");
  const QString &targetWorld(mTargetPath + "/worlds/" + worldFileBaseName);
  if (QFile::copy(mProject->path() + "worlds/" + worldFileBaseName + ".wbt", targetWorld + ".wbt")) {
    QFile::setPermissions(targetWorld + ".wbt",
                          QFile::permissions(targetWorld + ".wbt") | QFile::WriteOwner | QFile::WriteUser);
    result++;
  }
  const QString &targetProjectFile(mTargetPath + "/worlds/." + worldFileBaseName + ".wbproj");
  if (QFile::copy(mProject->path() + "worlds/." + worldFileBaseName + ".wbproj", targetProjectFile)) {
    QFile::setPermissions(targetProjectFile, QFile::permissions(targetProjectFile) | QFile::WriteOwner | QFile::WriteUser);
    result++;
  }

  // copy only the needed texture files
  const QStringList &textureList = world->listTextureFiles();
  foreach (const QString &textureFile, textureList) {
    const QFileInfo &fi(textureFile);
    targetPathDir.mkpath(fi.path());
    if (QFile::copy(mProject->path() + "worlds/" + textureFile, mTargetPath + "/worlds/" + textureFile))
      result++;
  }

  return result;
}

void WbProjectRelocationDialog::selectDirectory() {
  const QString &path = QFileDialog::getExistingDirectory(this, "Select a directory", mTargetEdit->text());

  if (path.isEmpty())
    return;

  mTargetPath = path;
  mTargetEdit->setText(QDir::toNativeSeparators(mTargetPath));
  setStatus(tr("Push the [Copy] button to copy the necessary project files."));
}

bool WbProjectRelocationDialog::validateLocation(QWidget *parent, QString &filename, bool isImportingVrml) {
  mExternalProtoProjectPath.clear();

  // if file is not in installation directory: it's ok
  if (!WbFileUtil::isLocatedInInstallationDirectory(filename))
    return true;

  WbSimulationState *simulationState = WbSimulationState::instance();
  simulationState->pauseSimulation();

  // use native separators in user dialog
  const QString &nativeFilename = QDir::toNativeSeparators(filename);

  WbProject *current = WbProject::current();
  if (!WbFileUtil::isLocatedInDirectory(filename, current->path()) ||
      WbFileUtil::isLocatedInDirectory(filename, WbStandardPaths::resourcesPath())) {
    const QList<WbRobot *> &robots = WbWorld::instance()->robots();
    foreach (WbRobot *robot, robots) {
      const WbProtoModel *proto = robot->proto();
      if (!proto)
        continue;

      QDir protoProjectDir(QFileInfo(proto->fileName()).path());
      protoProjectDir.cdUp();
      if (WbFileUtil::isLocatedInDirectory(filename, protoProjectDir.absolutePath())) {
        mExternalProtoProjectPath = protoProjectDir.absolutePath();
        if (!mExternalProtoProjectPath.endsWith("/"))
          mExternalProtoProjectPath = mExternalProtoProjectPath + "/";
        break;
      }
    }

    if (mExternalProtoProjectPath.isEmpty()) {
      // file is not in current project
      WbMessageBox::warning(tr("You are trying to modify a file located in Webots installation directory:") + "\n\n'" +
                              nativeFilename + "'\n\n" + tr("This operation is not permitted."),
                            parent);
      simulationState->resumeSimulation();
      return false;
    }
  }

  const QString &modificationDescription =
    isImportingVrml ? tr("You are trying to import VRML97 nodes to a world located in the Webots installation directory:") :
                      tr("You are trying to modify a file located in the Webots installation directory:");

  // file is in current project
  if (WbMessageBox::question(
        modificationDescription + "\n\n'" + nativeFilename + "'\n\n" +
          tr("This operation is not permitted: would you like to copy the necessary files to another location?"),
        parent) == QMessageBox::Cancel) {
    simulationState->resumeSimulation();
    return false;
  }

  // change filename parameter: get relative filename with respect to previous project path
  QString absolutePath;
  if (mExternalProtoProjectPath.isEmpty())
    absolutePath = current->path();
  else
    absolutePath = mExternalProtoProjectPath;
  filename = QDir(absolutePath).relativeFilePath(filename);

  // relocate dialog
  WbProjectRelocationDialog dialog(current, filename, absolutePath, parent);
  dialog.exec();
  if (dialog.result() == QDialog::Rejected) {
    simulationState->resumeSimulation();
    return false;
  }

  // change filename parameter: set absolute filename with respect to the new project path
  filename = QDir(dialog.targetPath()).absoluteFilePath(filename);

  simulationState->resumeSimulation();
  return true;
}
