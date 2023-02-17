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

#ifndef WB_ACTION_MANAGER_HPP
#define WB_ACTION_MANAGER_HPP

//
// Description: Singleton class creating and storing the Qt actions
//

#include "WbAction.hpp"

#include <QtCore/QHash>
#include <QtCore/QObject>

class QAction;

class WbActionManager : public QObject {
  Q_OBJECT

public:
  static WbActionManager *instance();

  QAction *action(WbAction::WbActionKind kind);

  void setEnabled(WbAction::WbActionKind kind, bool enabled);

  void resetApplicationActionsState();
  void enableTextEditActions(bool enabled, bool isReadOnly);
  QObject *focusObject() const { return mFocusObject; }
  void setFocusObject(QObject *object) { mFocusObject = object; }

  static void setActionEnabledSilently(QAction *action, bool enabled);
  static const QString mapControlKey();

  void updateRenderingButton();

signals:
  void userConsoleEditCommandReceived(WbAction::WbActionKind action);
  void userDocumentationEditCommandReceived(WbAction::WbActionKind action);
  void userTextEditCommandReceived(WbAction::WbActionKind action);
  void userWorldEditCommandReceived(WbAction::WbActionKind action);
  void transformRequested(QString newModelName);

public slots:
  void forwardTransformToActionToSceneTree();

private slots:
  void updateEnabled();
  void dispatchUserCommand();

private:
  static void cleanup();

  WbActionManager();
  virtual ~WbActionManager();

  void populateActions();
  void connectActions();

  static WbActionManager *cInstance;

  QHash<WbAction::WbActionKind, QAction *> mActions;
  QObject *mFocusObject;
};

#endif
