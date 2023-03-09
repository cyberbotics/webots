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

#ifndef WB_ACTION_HPP
#define WB_ACTION_HPP

//
// Description: List of actions available from any Webots module
//

#include <QtCore/QString>

namespace WbAction {
  enum WbActionKind {
    // world and simulation actions
    OPEN_WORLD,
    OPEN_SAMPLE_WORLD,
    SAVE_WORLD,
    SAVE_WORLD_AS,
    RELOAD_WORLD,
    RESET_SIMULATION,
    REAL_TIME,
    PAUSE,
    STEP,
    FAST,
    RENDERING,
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
    NORMALS,
    ORTHOGRAPHIC_PROJECTION,
    PERSPECTIVE_PROJECTION,
    PLAIN_RENDERING,
    WIREFRAME_RENDERING,
    DISABLE_SELECTION,
    DISABLE_3D_VIEW_CONTEXT_MENU,
    LOCK_VIEWPOINT,
    DISABLE_OBJECT_MOVE,
    DISABLE_FORCE_AND_TORQUE,
    DISABLE_RENDERING,
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
    PRINT,
    PRINT_PREVIEW,
    // console actions
    CLEAR_CONSOLE,
    NEW_CONSOLE,
    // viewpoint actions
    FOLLOW_NONE,
    FOLLOW_TRACKING,
    FOLLOW_MOUNTED,
    FOLLOW_PAN_AND_TILT,
    MOVE_VIEWPOINT_TO_OBJECT,
    OBJECT_FRONT_VIEW,
    OBJECT_BACK_VIEW,
    OBJECT_LEFT_VIEW,
    OBJECT_RIGHT_VIEW,
    OBJECT_TOP_VIEW,
    OBJECT_BOTTOM_VIEW,
    RESTORE_VIEWPOINT,
    VIEW_MENU,
    SOUTH_VIEW,
    NORTH_VIEW,
    WEST_VIEW,
    EAST_VIEW,
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
    EDIT_FIELD,
    EXPORT_URDF,
    // PROTO actions
    EDIT_PROTO_SOURCE,
    SHOW_PROTO_SOURCE,
    SHOW_PROTO_RESULT,
    CONVERT_TO_BASE_NODES,
    CONVERT_ROOT_TO_BASE_NODES,
    // keep track of numher of actions
    NACTIONS
  };
};  // namespace WbAction

#endif
