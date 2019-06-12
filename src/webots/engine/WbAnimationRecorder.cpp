// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbAnimationRecorder.hpp"

#include "WbField.hpp"
#include "WbGroup.hpp"
#include "WbLog.hpp"
#include "WbSFRotation.hpp"
#include "WbSimulationState.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QMutableListIterator>

// this function is used to round the transform position coordinates
#define ROUND(x, precision) (roundf((x) / precision) * precision)

WbAnimationCommand::WbAnimationCommand(WbNode *n, QStringList fields, bool saveInitialValue) :
  mNode(n),
  mChangedFromStart(false) {
  QString state;
  for (int i = 0; i < fields.size(); ++i) {
    WbField *field = mNode->findField(fields[i], true);
    if (field) {
      connect(field, &WbField::valueChanged, this, &WbAnimationCommand::updateValue);
      connect(field, &WbField::valueChangedByOde, this, &WbAnimationCommand::updateValue);
      connect(field, &WbField::valueChangedByWebots, this, &WbAnimationCommand::updateValue);
      mFields.append(field);

      if (saveInitialValue) {
        const WbSFVector3 *sfVector3 = dynamic_cast<WbSFVector3 *>(field->value());
        const WbSFRotation *sfRotation = dynamic_cast<WbSFRotation *>(field->value());
        const QString fieldName = field->name();
        if (!state.isEmpty())
          state += ",";
        state += "\"" + fieldName + "\":\"";
        if (sfVector3 && fieldName.compare("translation") == 0) {
          // special translation case
          state += QString("%1 %2 %3")
                     .arg(ROUND(sfVector3->x(), 0.0001))
                     .arg(ROUND(sfVector3->y(), 0.0001))
                     .arg(ROUND(sfVector3->z(), 0.0001));
        } else if (sfRotation && fieldName.compare("rotation") == 0) {
          // special rotation case
          state += QString("%1 %2 %3 %4")
                     .arg(ROUND(sfRotation->x(), 0.0001))
                     .arg(ROUND(sfRotation->y(), 0.0001))
                     .arg(ROUND(sfRotation->z(), 0.0001))
                     .arg(ROUND(sfRotation->angle(), 0.0001));
        } else  // generic case
          state += field->value()->toString(WbPrecision::FLOAT_MAX);
        state += fieldName + "\"";
      }
    }
  }

  if (!state.isEmpty()) {
    mInitialState = QString("{\"id\":\"%1\",%2}").arg(n->uniqueId()).arg(state);
  }
}

void WbAnimationCommand::resetChanges() {
  mChangedValues.clear();
}

void WbAnimationCommand::addArtificialFieldChange(const QString &fieldName, const QString &value) {
  mChangedValues[fieldName] = value;
}

void WbAnimationCommand::updateValue() {
  WbField *field = dynamic_cast<WbField *>(sender());
  if (field)
    updateFieldValue(field);
}

void WbAnimationCommand::updateAllFieldValues() {
  for (int i = 0; i < mFields.size(); ++i) {
    WbField *field = mFields.at(i);
    if (!mChangedValues.contains(field->name()))
      updateFieldValue(field);
  }
}

void WbAnimationCommand::updateFieldValue(WbField *field) {
  WbSFVector3 *sfVector3 = dynamic_cast<WbSFVector3 *>(field->value());
  WbSFRotation *sfRotation = dynamic_cast<WbSFRotation *>(field->value());
  if (sfVector3 && field->name().compare("translation") == 0) {
    // special translation case
    WbVector3 translationRounded =
      WbVector3(ROUND(sfVector3->x(), 0.001), ROUND(sfVector3->y(), 0.001), ROUND(sfVector3->z(), 0.001));
    if (translationRounded != mLastTranslation) {
      mChangedValues["translation"] = QString("%1 %2 %3")
                                        .arg(ROUND(sfVector3->x(), 0.0001))
                                        .arg(ROUND(sfVector3->y(), 0.0001))
                                        .arg(ROUND(sfVector3->z(), 0.0001));
      mLastTranslation = translationRounded;
      mChangedFromStart = true;
      emit changed(this);
    }
  } else if (sfRotation && field->name().compare("rotation") == 0) {
    // special rotation case
    WbRotation rotationRounded = WbRotation(ROUND(sfRotation->x(), 0.001), ROUND(sfRotation->y(), 0.001),
                                            ROUND(sfRotation->z(), 0.001), ROUND(sfRotation->angle(), 0.001));
    if (rotationRounded != mLastRotation) {
      mChangedValues["rotation"] = QString("%1 %2 %3 %4")
                                     .arg(ROUND(sfRotation->x(), 0.0001))
                                     .arg(ROUND(sfRotation->y(), 0.0001))
                                     .arg(ROUND(sfRotation->z(), 0.0001))
                                     .arg(ROUND(sfRotation->angle(), 0.0001));
      mLastRotation = rotationRounded;
      mChangedFromStart = true;
      emit changed(this);
    }
  } else {
    // generic case
    mChangedValues[field->name()] = field->value()->toString(WbPrecision::FLOAT_MAX);
    mChangedFromStart = true;
    emit changed(this);
  }
}

WbAnimationRecorder *WbAnimationRecorder::cInstance = NULL;

WbAnimationRecorder *WbAnimationRecorder::instance() {
  if (cInstance == NULL) {
    cInstance = new WbAnimationRecorder;
    qAddPostRoutine(WbAnimationRecorder::cleanup);
  }
  return cInstance;
}

void WbAnimationRecorder::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

WbAnimationRecorder::WbAnimationRecorder() :
  mIsRecording(false),
  mStartedFromGui(false),
  mLastUpdateTime(0.0),
  mFile(NULL),
  mFirstFrame(true),
  mStreamingServer(false) {
}

WbAnimationRecorder::~WbAnimationRecorder() {
  try {
    stopRecording();
  } catch (const QString &e) {
    WbLog::warning(tr("Error when stopping the HTML5 animation recording: '%1'").arg(e), true);
  }
}

void WbAnimationRecorder::initFromStreamingServer() {
  if (mFile)
    throw tr("HTML5 animation recorder is enabled.");

  if (mStreamingServer)
    throw tr("Streaming server already initialized.");

  emit initalizedFromStreamingServer();

  populateCommands();

  mStreamingServer = true;
}

void WbAnimationRecorder::cleanupFromStreamingServer() {
  emit cleanedUpFromStreamingServer();

  cleanCommands();

  mStreamingServer = false;
}

void WbAnimationRecorder::propagateNodeAddition(WbNode *node) {
  if (!mStreamingServer)
    return;

  populateCommands();
}

void WbAnimationRecorder::populateCommands() {
  cleanCommands();

  WbWorld *world = WbWorld::instance();
  if (world) {
    QList<WbNode *> nodes = WbWorld::instance()->root()->subNodes(true);
    for (int i = 0; i < nodes.size(); ++i) {
      WbNode *node = nodes.at(i);
      if (node->isUseNode())
        // skip updates for USE nodes
        // DEF/USE mechanism is handled in webots.min.js
        continue;
      QStringList fields = node->fieldsToSynchronizeWithX3D();
      if (fields.size() > 0) {
        WbAnimationCommand *command = new WbAnimationCommand(node, fields, !mStreamingServer);
        mCommands << command;
      }
    }
  }

  foreach (WbAnimationCommand *command, mCommands) {
    connect(command, &WbAnimationCommand::changed, this, &WbAnimationRecorder::addChangedCommandToList);
    // support node deletions
    connect(command->node(), &WbNode::destroyed, this, &WbAnimationRecorder::updateCommandsAfterNodeDeletion);
  }

  connect(WbWorld::instance()->viewpoint(), &WbViewpoint::nodeVisibilityChanged, this,
          &WbAnimationRecorder::handleNodeVisibilityChange);
}

void WbAnimationRecorder::cleanCommands() {
  foreach (WbAnimationCommand *command, mCommands) {
    disconnect(command, &WbAnimationCommand::changed, this, &WbAnimationRecorder::addChangedCommandToList);
    disconnect(command->node(), &WbNode::destroyed, this, &WbAnimationRecorder::updateCommandsAfterNodeDeletion);
    delete command;
  }
  mCommands.clear();
  mChangedCommands.clear();
  foreach (WbAnimationCommand *command, mArtificialCommands)
    delete command;
  mArtificialCommands.clear();
}

void WbAnimationRecorder::addChangedCommandToList(WbAnimationCommand *command) {
  if (!mChangedCommands.contains(command))
    mChangedCommands.append(command);
}

void WbAnimationRecorder::handleNodeVisibilityChange(WbNode *node, bool visibility) {
  WbAnimationCommand *newCommand = NULL;
  foreach (WbAnimationCommand *command, mArtificialCommands) {
    if (command->node() == node) {
      newCommand = command;
      break;
    }
  }
  if (!newCommand) {
    newCommand = new WbAnimationCommand(node, QStringList(), !mStreamingServer);
    mArtificialCommands << newCommand;
  }
  if (visibility)
    newCommand->addArtificialFieldChange("render", "true");
  else
    newCommand->addArtificialFieldChange("render", "false");
}

void WbAnimationRecorder::updateCommandsAfterNodeDeletion(QObject *node) {
  QMutableListIterator<WbAnimationCommand *> it(mCommands);
  while (it.hasNext()) {
    WbAnimationCommand *command = it.next();
    if (command->node() == node) {
      it.remove();
      mChangedCommands.removeAll(command);
      disconnect(command, &WbAnimationCommand::changed, this, &WbAnimationRecorder::addChangedCommandToList);
      disconnect(command->node(), &WbNode::destroyed, this, &WbAnimationRecorder::updateCommandsAfterNodeDeletion);
      delete command;
    }
  }
}

void WbAnimationRecorder::update() {
  double currentTime = WbSimulationState::instance()->time();
  if (mLastUpdateTime < 0.0 || currentTime - mLastUpdateTime >= 1000.0 / WbWorld::instance()->worldInfo()->fps()) {
    QString data = computeUpdateData();
    if (data.isEmpty())
      return;

    QTextStream out(mFile);

    if (!mFirstFrame)
      out << ",\n";

    out << data;

    mFirstFrame = false;
    mLastUpdateTime = currentTime;
  }
}

QString WbAnimationRecorder::computeUpdateData(bool force) {
  // Note: the copy to json is done in 2 passes in order
  //       to be able to manage correctly the trailing ',' characters
  if (force) {
    foreach (WbAnimationCommand *command, mChangedCommands)
      command->updateAllFieldValues();
  }
  QString result;
  QTextStream out(&result);
  double time = WbSimulationState::instance()->time();
  out << "{\"time\":" << QString::number(time);
  QList<WbAnimationCommand *> commands = mChangedCommands + mArtificialCommands;
  if (commands.size() == 0) {
    out << "}";
    return result;
  }
  out << ",\"poses\":[";
  foreach (WbAnimationCommand *command, commands) {
    const QList<QString> keys = command->fields();
    if (keys.isEmpty())
      continue;
    out << "{";
    out << QString("\"id\":%1").arg(command->node()->uniqueId());
    foreach (const QString &fieldName, keys)
      out << QString(",\"%1\":\"%2\"").arg(fieldName).arg(command->fieldValue(fieldName));
    if (command == commands.last())
      out << "}";
    else
      out << "},";
    command->resetChanges();
  }
  out << "]}";
  mChangedCommands.clear();
  foreach (WbAnimationCommand *command, mArtificialCommands)
    delete command;
  mArtificialCommands.clear();
  return result;
}

void WbAnimationRecorder::startRecording(const QString &targetFile) {
  // clear previous data
  mFileHeader.clear();

  mFile = new QFile(targetFile);
  if (!mFile->open(QIODevice::WriteOnly))
    throw tr("Cannot open HTML5 animation file '%1'").arg(mFile->fileName());

  populateCommands();

  // save data to be written at the beginning of the file at the end of the animation recording
  QTextStream out(&mFileHeader);
  out << "{\n";
  WbWorldInfo *const worldInfo = WbWorld::instance()->worldInfo();
  double step = worldInfo->basicTimeStep() * ceil((1000.0 / worldInfo->fps()) / worldInfo->basicTimeStep());
  out << QString(" \"basicTimeStep\":%1,\n").arg(step);
  out << " \"ids\":\"";
  foreach (WbAnimationCommand *command, mCommands) {
    out << command->node()->uniqueId();
    if (command != mCommands.last())
      out << ";";
  }
  out << "\",\n";
  out << " \"frames\":[\n";

  connect(WbSimulationState::instance(), &WbSimulationState::physicsStepEnded, this, &WbAnimationRecorder::update);

  mLastUpdateTime = -1;
  mIsRecording = true;
  mFirstFrame = true;

  WbLog::info(tr("Start HTML5 animation export\n"));
}

void WbAnimationRecorder::stop() {
  try {
    stopRecording();
  } catch (const QString &e) {
    WbLog::warning(tr("Error when stopping the HTML5 animation recording: '%1'").arg(e), true);
    emit animationStopStatusChanged(false);
  }
}

void WbAnimationRecorder::start(const QString &fileName) {
  WbWorld *world = WbWorld::instance();
  connect(world, &WbWorld::destroyed, this, &WbAnimationRecorder::stop);

  mAnimationFilename = fileName;
  mAnimationFilename.replace(QRegExp(".html$", Qt::CaseInsensitive), ".json");

  try {
    bool success = world->exportAsHtml(fileName, true);
    if (!success)
      throw tr("HTML5 export failed");

    QString animationFilename(mAnimationFilename);

    startRecording(animationFilename);

    emit animationStartStatusChanged(true);
  } catch (const QString &e) {
    WbLog::error(tr("Error when starting the HTML5 animation recording: '%1'").arg(e), true);
    emit animationStartStatusChanged(false);
  }
}

void WbAnimationRecorder::stopRecording() {
  disconnect(WbSimulationState::instance(), &WbSimulationState::physicsStepEnded, this, &WbAnimationRecorder::update);
  mIsRecording = false;

  if (!mFile)
    return;

  // Prepend header and initial state to the file containing updates.
  mFile->close();
  mFile->open(QFile::ReadOnly | QFile::Text);
  const QByteArray updates = mFile->readAll();
  mFile->close();
  mFile->open(QFile::WriteOnly | QFile::Text);
  QTextStream out(mFile);
  out << mFileHeader;

  // write initial state
  out << "{\"time\":0,\"poses\":[";
  bool firstState = true;
  foreach (WbAnimationCommand *command, mCommands) {
    if (command->isChangedFromStart()) {
      if (!firstState)
        out << ",";
      else
        firstState = false;
      out << command->initialState();
    }
  }
  cleanCommands();
  out << "]}";
  if (!updates.isEmpty()) {
    out << ",\n";
    out << updates;
  }
  out << "\n ]\n}\n";
  mFile->close();

  const QFileInfo fi(mFile->fileName());
  const QString fileName = fi.absolutePath() + "/" + fi.baseName() + ".html";
  WbLog::info(tr("HTML5 animation successfully exported in '%1'\n").arg(fileName));

  if (mStartedFromGui && !mStreamingServer)
    emit requestOpenUrl(fileName,
                        tr("The animation has been created:\n%1\n\nDo you want to view it locally now?\n\nNote: Animations can "
                           "not be viewed locally on Google Chrome.")
                          .arg(fileName),
                        tr("Make HTML5 Animation"));

  delete mFile;
  mFile = NULL;
}
