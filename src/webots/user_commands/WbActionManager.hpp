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

#ifndef WB_ACTION_MANAGER_HPP
#define WB_ACTION_MANAGER_HPP

//
// Description: Singleton class creating and storing the Qt actions
//

#include <QtCore/QHash>
#include <QtCore/QObject>

class QAction;

class WbActionManager : public QObject {
  Q_OBJECT

public:
  enum WbActionKind {
    // world and simulation actions
    NEW_WORLD,
    OPEN_WORLD,
    OPEN_SAMPLE_WORLD,
    SAVE_WORLD,
    SAVE_WORLD_AS,
    RELOAD_WORLD,
    RESET_SIMULATION,
    REAL_TIME,
    PAUSE,
    STEP,
    RUN,
    FAST,
    SOUND_UNMUTE,
    SOUND_MUTE,
    ANIMATION,
    TAKE_SCREENSHOT,
    // 3D scene
    COORDINATE_SYSTEM,
    BOUNDING_OBJECT,
    CONTACT_POINTS,
    CONNECTOR_AXES,
    JOINT_AXES,
    RANGE_FINDER_FRUSTUMS,
    LIDAR_RAYS_PATH,
    LIDAR_POINT_CLOUD,
    CAMERA_FRUSTUM,
    DISTANCE_SENSOR_RAYS,
    LIGHT_SENSOR_RAYS,
    LIGHT_POSITIONS,
    CENTER_OF_BUOYANCY,
    PEN_PAINTING_RAYS,
    CENTER_OF_MASS,
    SUPPORT_POLYGON,
    SKIN_SKELETON,
    RADAR_FRUSTUMS,
    PHYSICS_CLUSTERS,
    BOUNDING_SPHERE,
    ORTHOGRAPHIC_PROJECTION,
    PERSPECTIVE_PROJECTION,
    PLAIN_RENDERING,
    WIREFRAME_RENDERING,
    DISABLE_SELECTION,
    LOCK_VIEWPOINT,
    // application actions
    CUT,
    COPY,
    PASTE,
    SELECT_ALL,
    UNDO,
    REDO,
    ADD_NEW,
    DEL,  // 'DELETE' identifier causes compilation errors on Windows
    // text edit actions
    NEW_FILE,
    OPEN_FILE,
    SAVE_FILE,
    SAVE_FILE_AS,
    SAVE_ALL_FILES,
    REVERT_FILE,
    FIND,
    FIND_NEXT,
    FIND_PREVIOUS,
    REPLACE,
    GO_TO_LINE,
    TOGGLE_LINE_COMMENT,
    DUPLICATE_SELECTION,
    TRANSPOSE_LINE,
    PRINT,
    PRINT_PREVIEW,
    // console actions
    CLEAR_CONSOLE,
    // viewpoint actions
    FOLLOW_NONE,
    FOLLOW_TRACKING,
    FOLLOW_MOUNTED,
    FOLLOW_PAN_AND_TILT,
    MOVE_VIEWPOINT_TO_OBJECT,
    RESTORE_VIEWPOINT,
    VIEW_MENU,
    FRONT_VIEW,
    BACK_VIEW,
    LEFT_VIEW,
    RIGHT_VIEW,
    TOP_VIEW,
    BOTTOM_VIEW,
    // overlays
    HIDE_ALL_CAMERA_OVERLAYS,
    HIDE_ALL_DISPLAY_OVERLAYS,
    HIDE_ALL_RANGE_FINDER_OVERLAYS,
    // virtual reality headset
    VIRTUAL_REALITY_HEADSET_ENABLE,
    VIRTUAL_REALITY_HEADSET_POSITION,
    VIRTUAL_REALITY_HEADSET_ORIENTATION,
    VIRTUAL_REALITY_HEADSET_LEFT_EYE,
    VIRTUAL_REALITY_HEADSET_RIGHT_EYE,
    VIRTUAL_REALITY_HEADSET_NO_EYE,
    VIRTUAL_REALITY_HEADSET_ANTI_ALIASING,
    // robot actions
    EDIT_CONTROLLER,
    SHOW_ROBOT_WINDOW,
    // node/field actions
    OPEN_HELP,
    RESET_VALUE,
    EXPORT_NODE,
    // PROTO actions
    SHOW_PROTO_SOURCE,
    SHOW_PROTO_RESULT,
    CONVERT_TO_BASE_NODES,
    // keep track of numher of actions
    NACTIONS
  };

  static WbActionManager *instance();

  QAction *action(WbActionKind kind);

  void setEnabled(WbActionKind kind, bool enabled);

  void resetApplicationActionsState();
  void enableTextEditActions(bool enabled);
  QObject *focusObject() const { return mFocusObject; }
  void setFocusObject(QObject *object) { mFocusObject = object; }

  static void setActionEnabledSilently(QAction *action, bool enabled);

signals:
  void userConsoleEditCommandReceived(WbActionKind action);
  void userDocumentationEditCommandReceived(WbActionKind action);
  void userTextEditCommandReceived(WbActionKind action);
  void userWorldEditCommandReceived(WbActionKind action);
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
  QString mapControlKey();

  static WbActionManager *cInstance;

  QHash<WbActionKind, QAction *> mActions;
  QObject *mFocusObject;
};

#endif
