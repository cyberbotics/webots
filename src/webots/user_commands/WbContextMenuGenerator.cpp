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

#include "WbContextMenuGenerator.hpp"

#include "WbActionManager.hpp"
#include "WbNodeModel.hpp"
#include "WbNodeUtilities.hpp"
#include "WbRobot.hpp"
#include "WbViewpoint.hpp"
#include "WbWorldInfo.hpp"

#include <QtCore/QObject>
#include <QtWidgets/QMenu>

namespace WbContextMenuGenerator {
  static bool gAreNodeActionsEnabled = false;
  static bool gAreRobotActionsEnabled = false;
  static bool gAreProtoActionsEnabled = false;
  static bool gAreExternProtoActionsEnabled = false;
  static QMenu *gOverlaysMenu = NULL;

  void enableNodeActions(bool enabled) {
    gAreNodeActionsEnabled = enabled;
  }
  void enableRobotActions(bool enabled) {
    gAreRobotActionsEnabled = enabled;
  }
  void enableProtoActions(bool enabled) {
    gAreProtoActionsEnabled = enabled;
  }
  void enableExternProtoActions(bool enabled) {
    gAreExternProtoActionsEnabled = enabled;
  }
  void setOverlaysMenu(QMenu *menu) {
    gOverlaysMenu = menu;
  }

  const QStringList fillTransformToItems(const WbNode *selectedNode) {
    // populate transform combo box
    QStringList suitableModels;

    if (selectedNode && !selectedNode->isUseNode() && (selectedNode->useCount() == 0) && !selectedNode->isProtoInstance() &&
        (dynamic_cast<const WbGroup *>(selectedNode))) {
      // find all basic nodes
      QStringList basicModels = WbNodeModel::baseModelNames();

      // cache intensive searches results
      int hasDeviceChildren = -1;
      // find all nodes suitable for transform
      foreach (const QString &modelName, basicModels) {
        const WbNodeUtilities::Answer answer =
          WbNodeUtilities::isSuitableForTransform(selectedNode, modelName, &hasDeviceChildren);
        if (answer != WbNodeUtilities::UNSUITABLE)
          suitableModels << modelName;
      }
    }
    return suitableModels;
  }

  void generateContextMenu(const QPoint &position, const WbNode *selectedNode, QWidget *parentWidget) {
    QMenu *contextMenu = new QMenu(parentWidget);
    contextMenu->setObjectName("ContextMenu");
    contextMenu->addAction(WbActionManager::instance()->action(WbAction::CUT));
    contextMenu->addAction(WbActionManager::instance()->action(WbAction::COPY));
    contextMenu->addAction(WbActionManager::instance()->action(WbAction::PASTE));
    contextMenu->addAction(WbActionManager::instance()->action(WbAction::RESET_VALUE));
    contextMenu->addAction(WbActionManager::instance()->action(WbAction::EDIT_FIELD));
    contextMenu->addSeparator();
    contextMenu->addAction(WbActionManager::instance()->action(WbAction::ADD_NEW));
    contextMenu->addAction(WbActionManager::instance()->action(WbAction::DEL));
    contextMenu->addSeparator();
    contextMenu->addAction(WbActionManager::instance()->action(WbAction::MOVE_VIEWPOINT_TO_OBJECT));
    QMenu *viewMenu = contextMenu->addMenu(QObject::tr("Ali&gn View to Object"));
    viewMenu->addAction(WbActionManager::instance()->action(WbAction::OBJECT_FRONT_VIEW));
    viewMenu->addAction(WbActionManager::instance()->action(WbAction::OBJECT_BACK_VIEW));
    viewMenu->addAction(WbActionManager::instance()->action(WbAction::OBJECT_LEFT_VIEW));
    viewMenu->addAction(WbActionManager::instance()->action(WbAction::OBJECT_RIGHT_VIEW));
    viewMenu->addAction(WbActionManager::instance()->action(WbAction::OBJECT_TOP_VIEW));
    viewMenu->addAction(WbActionManager::instance()->action(WbAction::OBJECT_BOTTOM_VIEW));
    contextMenu->addSeparator();

    // selection-dependent actions
    if (selectedNode) {
      // actions for robots
      if (gAreRobotActionsEnabled) {
        contextMenu->addAction(WbActionManager::instance()->action(WbAction::EDIT_CONTROLLER));
        contextMenu->addAction(WbActionManager::instance()->action(WbAction::SHOW_ROBOT_WINDOW));

        assert(gOverlaysMenu);
        QMenu *subMenu = contextMenu->addMenu(QObject::tr("Overlays"));
        subMenu->setEnabled(false);
        QListIterator<QAction *> actionIt(gOverlaysMenu->actions());
        while (actionIt.hasNext()) {
          const QAction *action = actionIt.next();
          const QMenu *robotMenu = action->menu();
          if (robotMenu && robotMenu->property("robot").value<void *>() == selectedNode) {
            if (!robotMenu->isEnabled())
              break;
            assert(!robotMenu->actions().isEmpty());
            QListIterator<QAction *> menuIt(robotMenu->actions());
            bool enabled = true;
            while (menuIt.hasNext()) {
              QMenu *deviceMenu = menuIt.next()->menu();
              enabled = enabled || deviceMenu->isEnabled();
              subMenu->addMenu(deviceMenu);
            }
            subMenu->setEnabled(enabled);
          }
        }

        contextMenu->addSeparator();
      }

      // actions for nodes in general
      if (gAreNodeActionsEnabled) {
        QMenu *subMenu = contextMenu->addMenu(QObject::tr("Follow Object"));
        subMenu->addAction(WbActionManager::instance()->action(WbAction::FOLLOW_NONE));
        subMenu->addAction(WbActionManager::instance()->action(WbAction::FOLLOW_TRACKING));
        subMenu->addAction(WbActionManager::instance()->action(WbAction::FOLLOW_MOUNTED));
        subMenu->addAction(WbActionManager::instance()->action(WbAction::FOLLOW_PAN_AND_TILT));

        subMenu = contextMenu->addMenu(QObject::tr("Optional Rendering"));
        subMenu->addAction(WbActionManager::instance()->action(WbAction::CENTER_OF_MASS));
        subMenu->addAction(WbActionManager::instance()->action(WbAction::CENTER_OF_BUOYANCY));
        subMenu->addAction(WbActionManager::instance()->action(WbAction::SUPPORT_POLYGON));

        contextMenu->addSeparator();

        const WbBaseNode *selectedBaseNode = static_cast<const WbBaseNode *>(selectedNode);
        if (selectedBaseNode->nodeType() == WB_NODE_ROBOT)
          contextMenu->addAction(WbActionManager::instance()->action(WbAction::EXPORT_URDF));

        if (!gAreProtoActionsEnabled) {
          subMenu = contextMenu->addMenu(QObject::tr("Transform To..."));
          const QStringList suitableTransformToModels = fillTransformToItems(selectedNode);

          if (!suitableTransformToModels.isEmpty()) {
            foreach (const QString &model, suitableTransformToModels) {
              QAction *action = subMenu->addAction(model);
              QObject::connect(action, &QAction::triggered, WbActionManager::instance(),
                               &WbActionManager::forwardTransformToActionToSceneTree);
            }
          } else
            subMenu->setEnabled(false);
        }
      }

      // actions for PROTO nodes
      if (gAreProtoActionsEnabled) {
        QAction *editProtoAction(WbActionManager::instance()->action(WbAction::EDIT_PROTO_SOURCE));
        contextMenu->addAction(editProtoAction);
        if (gAreExternProtoActionsEnabled) {
          editProtoAction->setStatusTip(QObject::tr("Copy and edit the PROTO file in Text Editor."));
          contextMenu->addAction(WbActionManager::instance()->action(WbAction::SHOW_PROTO_SOURCE));
        } else
          editProtoAction->setStatusTip(QObject::tr("Edit the PROTO file in Text Editor."));
        editProtoAction->setToolTip(editProtoAction->statusTip());

        if (selectedNode->isTemplate())
          contextMenu->addAction(WbActionManager::instance()->action(WbAction::SHOW_PROTO_RESULT));

        contextMenu->addAction(WbActionManager::instance()->action(WbAction::CONVERT_TO_BASE_NODES));
        contextMenu->addAction(WbActionManager::instance()->action(WbAction::CONVERT_ROOT_TO_BASE_NODES));
      }
      contextMenu->addSeparator();
    }
    contextMenu->addAction(WbActionManager::instance()->action(WbAction::OPEN_HELP));

    QObject *focusObject = WbActionManager::instance()->focusObject();
    WbActionManager::instance()->setFocusObject(contextMenu);
    contextMenu->exec(position);
    WbActionManager::instance()->setFocusObject(focusObject);
  }
}  // namespace WbContextMenuGenerator
