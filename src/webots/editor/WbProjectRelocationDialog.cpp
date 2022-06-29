// Copyright 1996-2022 Cyberbotics Ltd.
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
#include "WbNetwork.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbProtoModel.hpp"
#include "WbRobot.hpp"
#include "WbSimulationState.hpp"
#include "WbStandardPaths.hpp"
#include "WbWorld.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>

#include <QtCore/QRegularExpression>
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
  mFilesLabel(NULL),
  mConclusionLabel(NULL),
  mStatusEdit(NULL),
  mSelectButton(NULL),
  mCancelButton(NULL),
  mCopyButton(NULL),
  mButtonBox(NULL) {
  setWindowTitle(tr("Project relocation"));

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
    copiedFilesCount += copyProject(currentProjectPath);
  }
  if (!mExternalProtoProjectPath.isEmpty())
    copiedFilesCount += copyProject(mExternalProtoProjectPath);

  if (copiedFilesCount == 0) {
    setStatus(tr("Project relocation failed.") + "\n" + tr("Some files or directories could not be copied."));
    return;
  }

  // change buttons
  if (mSelectButton)
    mSelectButton->setEnabled(false);
  mSourceEdit->setEnabled(false);
  mTargetEdit->setEnabled(false);
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

int WbProjectRelocationDialog::copyProject(const QString &projectPath) {
  int result = 0;

  // copy all local resources
  result += WbFileUtil::copyDir(projectPath + "protos", mTargetPath + "/protos", true, false, true);
  result += WbFileUtil::copyDir(projectPath + "motions", mTargetPath + "/motions", true, false, true);
  result += WbFileUtil::copyDir(projectPath + "plugins", mTargetPath + "/plugins", true, false, true);
  result += WbFileUtil::copyDir(projectPath + "libraries", mTargetPath + "/libraries", true, false, true);

  // copy only the needed robot controllers
  QStringList copiedControllers;
  const QList<WbRobot *> &robots = WbWorld::instance()->robots();
  foreach (WbRobot *robot, robots) {
    const QString &controllerName = robot->controllerName();
    if (controllerName.isEmpty())
      continue;
    if (controllerName.front() == '<' && controllerName.back() == '>')  // <none>, <generic> or <extern>
      continue;
    if (!copiedControllers.contains(controllerName)) {
      const QString &controllerPath = robot->controllerDir();
      result += WbFileUtil::copyDir(controllerPath, mTargetPath + "/controllers/" + controllerName, true, false, true);
      copiedControllers << controllerName;
    }
  }

  // copy the current source folder
  QString relativeDirPath = mRelativeFilename;
  if (QFileInfo(projectPath + relativeDirPath).isFile())
    // if it's not a directory, but a file, get the containing controller directory from after the "controllers/", "libraries/",
    // "protos/", etc. part of the string
    relativeDirPath = mRelativeFilename.left(mRelativeFilename.indexOf("/", mRelativeFilename.indexOf("/") + 1));
  const QString destinationPath = mTargetPath + "/" + relativeDirPath;
  if (!QDir(destinationPath).exists())
    result += WbFileUtil::copyDir(projectPath + relativeDirPath, destinationPath, true, true, true);

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
    const QFileInfo fi(textureFile);
    targetPathDir.mkpath(fi.path());
    if (QFile::copy(mProject->path() + "worlds/" + textureFile, mTargetPath + "/worlds/" + textureFile))
      result++;
  }

  // copy forests if the world files references any
  QFile file(world->fileName());
  if (file.open(QIODevice::ReadOnly)) {
    QRegularExpression re("\"([^\\.\"]+\\.forest)\"");
    QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());

    QStringList forests;
    while (it.hasNext()) {
      QRegularExpressionMatch match = it.next();
      if (match.hasMatch())
        forests << match.captured(1);
    }
    file.close();

    foreach (const QString &forest, forests) {
      const QFileInfo absolutePath = QFileInfo(QDir(WbProject::current()->worldsPath()).filePath(forest));
      const QFileInfo targetPath = QFileInfo(QDir(mTargetPath + "/worlds/").filePath(forest));
      QDir().mkpath(targetPath.absolutePath());  // create any necessary directories prior to copying the file
      if (QFile::copy(absolutePath.absoluteFilePath(), targetPath.absoluteFilePath()))
        result++;
      else
        setStatus(
          tr("Impossible to copy file '%1' to '%2'.").arg(absolutePath.absoluteFilePath()).arg(targetPath.absoluteFilePath()));
    }
  } else
    setStatus(tr("Impossible to read file '%1'").arg(world->fileName()));

  // copy SUMO net directory if any
  QString fileName = world->fileName();
  const QString netDir = fileName.replace(".wbt", "_net");
  if (QDir().exists(netDir))
    result += WbFileUtil::copyDir(netDir, mTargetPath + "/worlds/" + QFileInfo(netDir).baseName(), true, false, true);

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

  if (WbFileUtil::isLocatedInDirectory(filename, WbNetwork::instance()->cacheDirectory())) {
    WbMessageBox::warning(tr("You are trying to modify a remote file.") + "\n\n'" + tr("This operation is not permitted."),
                          parent);
    return false;
  }

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
