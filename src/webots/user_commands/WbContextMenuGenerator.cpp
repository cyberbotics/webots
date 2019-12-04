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
  static QMenu *gRobotCameraMenu = NULL;
  static QMenu *gRobotRangeFinderMenu = NULL;
  static QMenu *gRobotDisplayMenu = NULL;

  void enableNodeActions(bool enabled) { gAreNodeActionsEnabled = enabled; }
  void enableRobotActions(bool enabled) { gAreRobotActionsEnabled = enabled; }
  void enableProtoActions(bool enabled) { gAreProtoActionsEnabled = enabled; }
  void setRobotCameraMenu(QMenu *menu) { gRobotCameraMenu = menu; }
  void setRobotRangeFinderMenu(QMenu *menu) { gRobotRangeFinderMenu = menu; }
  void setRobotDisplayMenu(QMenu *menu) { gRobotDisplayMenu = menu; }

  const QStringList fillTransformToItems(const WbBaseNode *selectedNode) {
    // populate transform combo box
    QStringList suitableModels;

    if (selectedNode && !selectedNode->isUseNode() && (selectedNode->useCount() == 0) && !selectedNode->isProtoInstance() &&
        (dynamic_cast<const WbGroup *>(selectedNode))) {
      // find all basic nodes
      QStringList basicModels = WbNodeModel::baseModelNames();

      // find all nodes suitable for transform
      foreach (const QString &modelName, basicModels) {
        WbNodeUtilities::Answer answer = WbNodeUtilities::isSuitableForTransform(selectedNode, modelName);
        if (answer != WbNodeUtilities::UNSUITABLE) {
          suitableModels << modelName;
        }
      }
    }
    return suitableModels;
  }

#ifdef __linux__
  void renameRobotOverlayActions(QMenu *menu, bool doubleUnderscoresRequired) {
    foreach (QAction *action, menu->actions()) {
      if (action->isSeparator())
        continue;
      else if (action->menu())
        renameRobotOverlayActions(action->menu(), doubleUnderscoresRequired);
      else {
        if (doubleUnderscoresRequired)
          action->setText(action->text().replace("_", "__"));
        else
          action->setText(action->text().replace("__", "_"));
      }
    }
  }
#endif

  void generateContextMenu(const QPoint &position, const WbBaseNode *selectedNode) {
    QMenu contextMenu;
    contextMenu.setObjectName("ContextMenu");
    contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::CUT));
    contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::COPY));
    contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::PASTE));
    contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::RESET_VALUE));
    contextMenu.addSeparator();
    contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::ADD_NEW));
    contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::DEL));
    contextMenu.addSeparator();
    contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::MOVE_VIEWPOINT_TO_OBJECT));
    contextMenu.addSeparator();

    // selection-dependent actions
    if (selectedNode) {
      // actions for robots
      if (gAreRobotActionsEnabled) {
#ifdef __linux__
        // fix for https://github.com/omichel/webots-dev/issues/7443, the context menu doesn't need the double underscore
        // fix for menubars on Unity desktops (Ubuntu 16.04), so undo the workaround before opening the menu and redo it on menu
        // close
        if (qgetenv("XDG_CURRENT_DESKTOP") == "Unity") {
          renameRobotOverlayActions(gRobotCameraMenu, false);
          renameRobotOverlayActions(gRobotRangeFinderMenu, false);
          renameRobotOverlayActions(gRobotDisplayMenu, false);
        }
#endif
        contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::EDIT_CONTROLLER));
        contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::SHOW_ROBOT_WINDOW));
        QMenu *subMenu = contextMenu.addMenu(QObject::tr("Overlays"));
        subMenu->addMenu(gRobotCameraMenu);
        subMenu->addMenu(gRobotRangeFinderMenu);
        subMenu->addMenu(gRobotDisplayMenu);
        contextMenu.addSeparator();
      }

      // actions for nodes in general
      if (gAreNodeActionsEnabled) {
        QMenu *subMenu = contextMenu.addMenu(QObject::tr("Follow Object"));
        subMenu->addAction(WbActionManager::instance()->action(WbActionManager::FOLLOW_NONE));
        subMenu->addAction(WbActionManager::instance()->action(WbActionManager::FOLLOW_TRACKING));
        subMenu->addAction(WbActionManager::instance()->action(WbActionManager::FOLLOW_MOUNTED));
        subMenu->addAction(WbActionManager::instance()->action(WbActionManager::FOLLOW_PAN_AND_TILT));

        subMenu = contextMenu.addMenu(QObject::tr("Optional Rendering"));
        subMenu->addAction(WbActionManager::instance()->action(WbActionManager::CENTER_OF_MASS));
        subMenu->addAction(WbActionManager::instance()->action(WbActionManager::CENTER_OF_BUOYANCY));
        subMenu->addAction(WbActionManager::instance()->action(WbActionManager::SUPPORT_POLYGON));

        contextMenu.addSeparator();

        if (!(selectedNode->nodeType() == WB_NODE_WORLD_INFO || selectedNode->nodeType() == WB_NODE_VIEWPOINT))
          contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::EXPORT_NODE));

        if (!gAreProtoActionsEnabled) {
          subMenu = contextMenu.addMenu(QObject::tr("Transform To..."));
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
        contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::SHOW_PROTO_SOURCE));

        if (selectedNode->isTemplate())
          contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::SHOW_PROTO_RESULT));

        contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::CONVERT_TO_BASE_NODES));
      }
      contextMenu.addSeparator();
    }
    contextMenu.addAction(WbActionManager::instance()->action(WbActionManager::OPEN_HELP));

    QObject *focusObject = WbActionManager::instance()->focusObject();
    WbActionManager::instance()->setFocusObject(&contextMenu);
    contextMenu.exec(position);
    WbActionManager::instance()->setFocusObject(focusObject);

#ifdef __linux__
    // see above comment, rename menu items again so everything works in the Overlays menu
    if (qgetenv("XDG_CURRENT_DESKTOP") == "Unity" && gAreRobotActionsEnabled) {
      renameRobotOverlayActions(gRobotCameraMenu, true);
      renameRobotOverlayActions(gRobotRangeFinderMenu, true);
      renameRobotOverlayActions(gRobotDisplayMenu, true);
    }
#endif
  }
}  // namespace WbContextMenuGenerator
