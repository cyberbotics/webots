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

#include "WbActionManager.hpp"

#include "WbSimulationState.hpp"
#include "WbUndoStack.hpp"

#ifdef __linux__
#include <QtCore/QCoreApplication>
#endif
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>

#include <cassert>

using namespace WbAction;

WbActionManager *WbActionManager::cInstance = NULL;

WbActionManager *WbActionManager::instance() {
  if (cInstance == NULL) {
    cInstance = new WbActionManager;
    qAddPostRoutine(WbActionManager::cleanup);
  }
  return cInstance;
}

void WbActionManager::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

WbActionManager::WbActionManager() : QObject(), mFocusObject(NULL) {
  populateActions();
  connectActions();
}

WbActionManager::~WbActionManager() {
}

QAction *WbActionManager::action(WbActionKind kind) {
  return mActions[kind];
}

QString WbActionManager::mapControlKey() {
#ifdef __APPLE__
  return "⌘";
#else
  return "Ctrl";
#endif
}

void WbActionManager::populateActions() {
  QAction *action;
  QIcon icon;

  /* WORLD and SIMULATION ACTIONS */
  action = new QAction(this);
  action->setText(tr("&New World"));
  action->setStatusTip(tr("Create a new simulation world. (%1+Shift+N)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::CTRL + Qt::Key_N);
  mActions[NEW_WORLD] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:open_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:open_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Open World..."));
  action->setStatusTip(tr("Open an existing world file. (%1+Shift+O)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::CTRL + Qt::Key_O);
  action->setIcon(icon);
  mActions[OPEN_WORLD] = action;

  action = new QAction(this);
  action->setText(tr("&Open Sample World..."));
  action->setStatusTip(tr("Open a sample world."));
  action->setToolTip(action->statusTip());
  mActions[OPEN_SAMPLE_WORLD] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:save_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:save_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Save World"));
  action->setStatusTip(tr("Save the current world file. (%1+Shift+S)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::CTRL + Qt::Key_S);
  action->setIcon(icon);
  mActions[SAVE_WORLD] = action;

  action = new QAction(this);
  action->setText(tr("Save World &As..."));
  action->setStatusTip(tr("Save the current world file with a new name."));
  action->setToolTip(action->statusTip());
  mActions[SAVE_WORLD_AS] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:reload_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:reload_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Reload World"));
  action->setStatusTip(
    tr("Reload World.\nReload the current world file and restart the simulation. (%1+Shift+R)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
  action->setShortcut(Qt::SHIFT + Qt::CTRL + Qt::Key_R);
  mActions[RELOAD_WORLD] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:reset_simulation_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:reset_simulation_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("Reset Simulation"));
  action->setStatusTip(tr("Reset Simulation.\nRestore initial state of the simulation. (%1+Shift+T)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::CTRL + Qt::Key_T);
  action->setIcon(icon);
  mActions[RESET_SIMULATION] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:real_time_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:real_time_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("Real-&time"));
  action->setStatusTip(tr("Run the simulation in real-time. (%1+2)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_2);
  action->setIcon(icon);
  mActions[REAL_TIME] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:pause_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:pause_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Pause"));
  action->setStatusTip(tr("Pause the simulation. (%1+0)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_0);
  action->setIcon(icon);
  mActions[PAUSE] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:step_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:step_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("St&ep"));
  action->setStatusTip(tr("Execute one simulation step. (%1+1)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_1);
  action->setIcon(icon);
  mActions[STEP] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:fast_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:fast_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Fast"));
  action->setStatusTip(tr("Run the simulation as fast as possible. (%1+3)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_3);
  action->setIcon(icon);
  mActions[FAST] = action;

  action = new QAction(this);
  action->setCheckable(true);
  action->setShortcut(Qt::CTRL + Qt::Key_4);
  action->setText(tr("&Rendering"));
  mActions[RENDERING] = action;

  action = new QAction(this);
  action->setText(tr("&Unmute sound"));
  action->setStatusTip(tr("Unmute the sound on the main audio device of the computer."));
  action->setToolTip(action->statusTip());
  // the icon is inverted there to respect the other applications behavior
  action->setIcon(QIcon("enabledIcons:sound_mute_button.png"));
  mActions[SOUND_UNMUTE] = action;

  action = new QAction(this);
  action->setText(tr("&Mute sound"));
  action->setStatusTip(tr("Mute the sound on the main audio device of the computer."));
  action->setToolTip(action->statusTip());
  // the icon is inverted there to respect the other applications behavior
  action->setIcon(QIcon("enabledIcons:sound_unmute_button.png"));
  mActions[SOUND_MUTE] = action;

  action = new QAction(this);
  mActions[ANIMATION] = action;

  action = new QAction(this);
  action->setText(tr("&Take Screenshot..."));
  action->setStatusTip(tr("Save the current image of the simulation. (%1 + SHIFT + P)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::SHIFT + Qt::Key_P);
  action->setIcon(QIcon("enabledIcons:screenshot_button.png"));
  mActions[TAKE_SCREENSHOT] = action;

  action = new QAction(this);
  action->setText(tr("&Perspective Projection"));
  action->setStatusTip(tr("Perspective viewing projection."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::Key_F9);
  action->setCheckable(true);
  mActions[PERSPECTIVE_PROJECTION] = action;

  action = new QAction(this);
  action->setText(tr("&Orthographic Projection"));
  action->setStatusTip(tr("Orthographic viewing projection."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::Key_F10);
  action->setCheckable(true);
  mActions[ORTHOGRAPHIC_PROJECTION] = action;

  QActionGroup *actionGroup = new QActionGroup(this);
  actionGroup->addAction(mActions[PERSPECTIVE_PROJECTION]);
  actionGroup->addAction(mActions[ORTHOGRAPHIC_PROJECTION]);

  action = new QAction(this);
  action->setText(tr("&Plain Rendering"));
  action->setStatusTip(tr("Plain OpenGL rendering."));
  action->setToolTip(action->statusTip());
#ifdef __APPLE__
  action->setShortcut(Qt::SHIFT + Qt::Key_P);
#else
  action->setShortcut(Qt::Key_F11);
#endif
  action->setCheckable(true);
  mActions[PLAIN_RENDERING] = action;

  action = new QAction(this);
  action->setText(tr("&Wireframe Rendering"));
  action->setStatusTip(tr("Rendering only the segments between the vertices."));
  action->setToolTip(action->statusTip());
#ifdef __APPLE__
  action->setShortcut(Qt::SHIFT + Qt::Key_W);
#else
  action->setShortcut(Qt::Key_F12);
#endif
  action->setCheckable(true);
  mActions[WIREFRAME_RENDERING] = action;

  actionGroup = new QActionGroup(this);
  actionGroup->addAction(mActions[PLAIN_RENDERING]);
  actionGroup->addAction(mActions[WIREFRAME_RENDERING]);

  // ----- optional rendering -----
  action = new QAction(this);
  action->setText(tr("Show &Coordinate System"));
  action->setStatusTip(tr("Show coordinate system."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F1);
  action->setCheckable(true);
  mActions[COORDINATE_SYSTEM] = action;

  action = new QAction(this);
  action->setText(tr("Show All &Bounding Objects"));
  action->setStatusTip(tr("Show all bounding objects."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F2);
  action->setCheckable(true);
  mActions[BOUNDING_OBJECT] = action;

  action = new QAction(this);
  action->setText(tr("Show Contact &Points"));
  action->setStatusTip(tr("Show contact points and polygons."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F3);
  action->setCheckable(true);
  mActions[CONTACT_POINTS] = action;

  action = new QAction(this);
  action->setText(tr("Show Connector &Axes"));
  action->setStatusTip(tr("Show connector axes."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F4);
  action->setCheckable(true);
  mActions[CONNECTOR_AXES] = action;

  action = new QAction(this);
  action->setText(tr("Show &Joint Axes"));
  action->setStatusTip(tr("Show joint axes."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F5);
  action->setCheckable(true);
  mActions[JOINT_AXES] = action;

  action = new QAction(this);
  action->setText(tr("Show Range&Finder Frustums"));
  action->setStatusTip(tr("Show range-finder frustums."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F6);
  action->setCheckable(true);
  mActions[RANGE_FINDER_FRUSTUMS] = action;

  action = new QAction(this);
  action->setText(tr("Show Lidar &Ray Paths"));
  action->setStatusTip(tr("Show lidar rays paths."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F7);
  action->setCheckable(true);
  mActions[LIDAR_RAYS_PATH] = action;

  action = new QAction(this);
  action->setText(tr("Show Lidar Point Cl&oud"));
  action->setStatusTip(tr("Show lidar point cloud."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F8);
  action->setCheckable(true);
  mActions[LIDAR_POINT_CLOUD] = action;

  action = new QAction(this);
  action->setText(tr("Show &Camera Frustums"));
  action->setStatusTip(tr("Show camera frustums."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F9);
  action->setCheckable(true);
  mActions[CAMERA_FRUSTUM] = action;

  action = new QAction(this);
  action->setText(tr("Show &DistanceSensor Rays"));
  action->setStatusTip(tr("Show distance sensors rays."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F10);
  action->setCheckable(true);
  mActions[DISTANCE_SENSOR_RAYS] = action;

  action = new QAction(this);
  action->setText(tr("Show &LightSensor Rays"));
  action->setStatusTip(tr("Show light sensors rays."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F11);
  action->setCheckable(true);
  mActions[LIGHT_SENSOR_RAYS] = action;

  action = new QAction(this);
  action->setText(tr("Show L&ight Positions"));
  action->setStatusTip(tr("Show position of light sources."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F12);
  action->setCheckable(true);
  mActions[LIGHT_POSITIONS] = action;

  action = new QAction(this);
  action->setText(tr("Show Center of Buo&yancy"));
  action->setStatusTip(tr("Show the center of buoyancy of a solid."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F3);
  action->setCheckable(true);
  mActions[CENTER_OF_BUOYANCY] = action;

  action = new QAction(this);
  action->setText(tr("Show &Pen Painting Rays"));
  action->setStatusTip(tr("Show pen painting rays."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F1);
  action->setCheckable(true);
  mActions[PEN_PAINTING_RAYS] = action;

  action = new QAction(this);
  action->setText(tr("Show Center of &Mass"));
  action->setStatusTip(tr("Show the center of mass of a solid."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F2);
  action->setCheckable(true);
  mActions[CENTER_OF_MASS] = action;

  action = new QAction(this);
  action->setText(tr("Show S&upport Polygon"));
  action->setStatusTip(tr("Show the center of mass and the support polygon of a solid."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F4);
  action->setCheckable(true);
  mActions[SUPPORT_POLYGON] = action;

  action = new QAction(this);
  action->setText(tr("Show S&kin Skeleton"));
  action->setStatusTip(tr("Turn on visual representation of skeleton used by the Skin device."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F9);
  action->setCheckable(true);
  mActions[SKIN_SKELETON] = action;

  action = new QAction(this);
  action->setText(tr("Show Radar Frustums"));
  action->setStatusTip(tr("Show radar frustums."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F6);
  action->setCheckable(true);
  mActions[RADAR_FRUSTUMS] = action;

  action = new QAction(this);
  action->setText(tr("Show Normals"));
  action->setStatusTip(tr("Show IndexedFaceSet and Mesh nodes normals."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F5);
  action->setCheckable(true);
  mActions[NORMALS] = action;

  action = new QAction(this);
  action->setText(tr("Show Physics Clusters"));
  action->setStatusTip(tr("Show visual representation of ODE clusters."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[PHYSICS_CLUSTERS] = action;

  action = new QAction(this);
  action->setText(tr("Show Bounding Sphere"));
  action->setStatusTip(tr("Show visual representaton of the selected node's bounding sphere."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[BOUNDING_SPHERE] = action;

  action = new QAction(this);
  action->setText(tr("Lock Viewpoint"));
  action->setStatusTip(tr("Disable Viewpoint translation and rotation from 3D view."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[LOCK_VIEWPOINT] = action;

  action = new QAction(this);
  action->setText(tr("Disable Selection"));
  action->setStatusTip(tr("Disable selection change from 3D view."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[DISABLE_SELECTION] = action;

  action = new QAction(this);
  action->setText(tr("Disable 3D View Context Menu"));
  action->setStatusTip(tr("Disable opening the context menu clicking on the 3D view."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[DISABLE_3D_VIEW_CONTEXT_MENU] = action;

  action = new QAction(this);
  action->setText(tr("Disable Object Move"));
  action->setStatusTip(tr("Disable moving objects from 3D view."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[DISABLE_OBJECT_MOVE] = action;

  action = new QAction(this);
  action->setText(tr("Disable Applying Force and Torque"));
  action->setStatusTip(tr("Disable applying force and torque to objects from 3D view."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[DISABLE_FORCE_AND_TORQUE] = action;

  action = new QAction(this);
  action->setText(tr("Disable Rendering"));
  action->setStatusTip(tr("Disable activating the rendering."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[DISABLE_RENDERING] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:insert_after_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:insert_after_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setEnabled(false);
  action->setText(tr("&Add New"));
  action->setStatusTip(tr("Add a new object or import an object. (%1+Shift+A)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
  action->setShortcut(Qt::CTRL + Qt::SHIFT + Qt::Key_A);
  mActions[ADD_NEW] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:delete_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:delete_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setEnabled(false);
  action->setText(tr("&Delete"));
  action->setStatusTip(tr("Delete the selected object. (Del)"));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::Key_Delete);
  action->setIcon(icon);
  action->setProperty("kind", DEL);
  connect(action, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[DEL] = action;

  /* GENERAL ACTIONS  */
  icon = QIcon();
  icon.addFile("enabledIcons:cut_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:cut_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setEnabled(false);
  action->setText(tr("C&ut"));
  action->setStatusTip(tr("Cut object at the selected line. (%1+X)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcuts(QKeySequence::Cut);
  action->setIcon(icon);
  action->setProperty("kind", CUT);
  connect(action, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[CUT] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:copy_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:copy_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setEnabled(false);
  action->setText(tr("&Copy"));
  action->setStatusTip(tr("Copy object at the selected line. (%1+C)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcuts(QKeySequence::Copy);
  action->setIcon(icon);
  action->setProperty("kind", COPY);
  connect(action, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[COPY] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:paste_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:paste_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setEnabled(false);
  action->setText(tr("&Paste"));
  action->setStatusTip(tr("Paste or insert the clipboard object. (%1+V)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcuts(QKeySequence::Paste);
  action->setIcon(icon);
  action->setProperty("kind", PASTE);
  connect(action, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[PASTE] = action;

  action = new QAction(this);
  action->setText(tr("Select &All"));
  action->setStatusTip(tr("Select all text. (%1+A)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcut(QKeySequence::SelectAll);
  action->setProperty("kind", SELECT_ALL);
  connect(action, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[SELECT_ALL] = action;

  action = new QAction(this);
  action->setText(tr("&Undo"));
  action->setStatusTip(tr("Undo manual modification to the simulation world and edited text. (%1+Z)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcuts(QKeySequence::Undo);
  action->setProperty("kind", UNDO);
  connect(action, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[UNDO] = action;

  action = new QAction(this);
  action->setText(tr("Redo"));
  action->setStatusTip(tr("Redo manual modification to the simulation world and edited text. (%1+Y)").arg(mapControlKey()));
  action->setToolTip(action->statusTip());
  action->setShortcuts(QKeySequence::Redo);
  action->setProperty("kind", REDO);
  connect(action, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[REDO] = action;

  /* TEXT EDIT ACTIONS  */

  icon = QIcon();
  icon.addFile("enabledIcons:new_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:new_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&New Text File"));
  action->setStatusTip(tr("Create a new text file."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_N);
  action->setIcon(icon);
  mActions[NEW_FILE] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:open_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:open_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Open Text File..."));
  action->setStatusTip(tr("Open an existing text file."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_O);
  action->setIcon(icon);
  mActions[OPEN_FILE] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:save_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:save_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Save Text File"));
  action->setStatusTip(tr("Save the current text file."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_S);
  action->setIcon(icon);
  mActions[SAVE_FILE] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:save_as_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:save_as_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("Save Text File &As..."));
  action->setStatusTip(tr("Save the current text file with a new name."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
  mActions[SAVE_FILE_AS] = action;

  action = new QAction(this);
  action->setText(tr("Save All Text Files"));
  action->setStatusTip(tr("Save all the opened text files."));
  action->setToolTip(action->statusTip());
  mActions[SAVE_ALL_FILES] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:reload_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:reload_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Revert Text File"));
  action->setStatusTip(tr("Revert the current text file."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
  action->setShortcut(Qt::CTRL + Qt::Key_R);
  mActions[REVERT_FILE] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:find_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:find_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Find..."));
  action->setStatusTip(tr("Find text in current file."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_F);
  action->setIcon(icon);
  action->setProperty("kind", FIND);
  connect(action, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[FIND] = action;

  action = new QAction(this);
  action->setText(tr("Find &Next"));
  action->setStatusTip(tr("Find next occurence of search string in current file."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_G);
  action->setProperty("kind", FIND_NEXT);
  connect(action, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[FIND_NEXT] = action;

  action = new QAction(this);
  action->setText(tr("Find &Previous"));
  action->setStatusTip(tr("Find previous occurence of search string in current file."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::CTRL + Qt::Key_G);
  action->setProperty("kind", FIND_PREVIOUS);
  connect(action, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[FIND_PREVIOUS] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:replace_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:replace_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Replace..."));
  action->setStatusTip(tr("Replace text in current file."));
  action->setToolTip(action->statusTip());
#ifdef __APPLE__  // on Mac, CTRL+H is a system shortcut to hide the window
  action->setShortcut(Qt::CTRL + Qt::ALT + Qt::Key_F);
#else
  action->setShortcut(Qt::CTRL + Qt::Key_H);
#endif
  action->setIcon(icon);
  mActions[REPLACE] = action;

  action = new QAction(this);
  action->setText(tr("&Go to line..."));
  action->setStatusTip(tr("Go to specified line number."));
  action->setToolTip(action->statusTip());
  mActions[GO_TO_LINE] = action;

  action = new QAction(this);
  action->setText(tr("&Toggle Line Comment") + "    ");  // add spaces because the spacing between the menu text
  // and the hotkey text does not expand to adjust for the text length
  action->setStatusTip(tr("Toggle comment of selected lines."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_Slash);
  mActions[TOGGLE_LINE_COMMENT] = action;

  action = new QAction(this);
  action->setText(tr("&Print..."));
  action->setStatusTip(tr("Print text file."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_P);
  mActions[PRINT] = action;

  action = new QAction(this);
  action->setText(tr("&Print Preview..."));
  action->setStatusTip(tr("Preview of printed text file."));
  action->setToolTip(action->statusTip());
  mActions[PRINT_PREVIEW] = action;

  /* CONSOLE ACTIONS */
  action = new QAction(this);
  action->setText(tr("&Clear All Consoles"));
  action->setStatusTip(tr("Clears all the Consoles."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_K);
  mActions[CLEAR_CONSOLE] = action;

  action = new QAction(this);
  action->setText(tr("&New Console"));
  action->setStatusTip(tr("Opens a new Console."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::CTRL + Qt::Key_N);
  mActions[NEW_CONSOLE] = action;

  /* VIEWPOINT ACTIONS */

  action = new QAction(this);
  action->setText(tr("&None"));
  action->setStatusTip(tr("Do not follow the object."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[FOLLOW_NONE] = action;

  action = new QAction(this);
  action->setText(tr("&Tracking Shot"));
  action->setStatusTip(tr("Translate the camera to follow the object."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::Key_F5);
  action->setCheckable(true);
  mActions[FOLLOW_TRACKING] = action;

  action = new QAction(this);
  action->setText(tr("&Mounted Shot"));
  action->setStatusTip(tr("Translate and rotate the camera to follow the object."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F5);
  action->setCheckable(true);
  mActions[FOLLOW_MOUNTED] = action;

  action = new QAction(this);
  action->setText(tr("&Pan and Tilt Shot"));
  action->setStatusTip(tr("Rotate the camera to always look at the object center."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F7);
  action->setCheckable(true);
  mActions[FOLLOW_PAN_AND_TILT] = action;

  actionGroup->addAction(mActions[FOLLOW_NONE]);
  actionGroup->addAction(mActions[FOLLOW_TRACKING]);
  actionGroup->addAction(mActions[FOLLOW_MOUNTED]);
  actionGroup->addAction(mActions[FOLLOW_PAN_AND_TILT]);

  icon = QIcon();
  icon.addFile("enabledIcons:move_viewpoint_to_object_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:move_viewpoint_to_object_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Move Viewpoint to Object"));
#ifdef __APPLE__  // on Mac, CTRL+H is a system shortcut to hide the window
  action->setStatusTip(tr("Move viewpoint to selected object. (CTRL + ALT + 5)"));
#else
  action->setStatusTip(tr("Move viewpoint to selected object. (ALT + 5)"));
#endif
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
#ifdef __APPLE__
  action->setShortcut(Qt::META + Qt::ALT + Qt::Key_5);
#else
  action->setShortcut(Qt::ALT + Qt::Key_5);
#endif
  mActions[MOVE_VIEWPOINT_TO_OBJECT] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:front_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:front_view.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Change View"));
  action->setStatusTip(tr("Open standard Viewpoint positions menu."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
  mActions[VIEW_MENU] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:restore_viewpoint_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:restore_viewpoint_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("Restore &Viewpoint"));
  action->setStatusTip(tr("Restore the initial Viewpoint position and orientation. (CTRL + SHIFT + V)"));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
  action->setShortcut(Qt::CTRL + Qt::SHIFT + Qt::Key_V);
  mActions[RESTORE_VIEWPOINT] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:front_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:front_view.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Front View"));
  action->setStatusTip(tr("Move Viewpoint to see object from the front."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
#ifdef __APPLE__
  action->setShortcut(Qt::META + Qt::ALT + Qt::Key_2);
#else
  action->setShortcut(Qt::ALT + Qt::Key_2);
#endif
  mActions[FRONT_VIEW] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:back_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:back_view.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Back View"));
  action->setStatusTip(tr("Move Viewpoint to see object from the back."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
#ifdef __APPLE__
  action->setShortcut(Qt::META + Qt::ALT + Qt::Key_8);
#else
  action->setShortcut(Qt::ALT + Qt::Key_8);
#endif
  mActions[BACK_VIEW] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:left_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:left_view.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Left View"));
  action->setStatusTip(tr("Move Viewpoint to see object from the left."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
#ifdef __APPLE__
  action->setShortcut(Qt::META + Qt::ALT + Qt::Key_4);
#else
  action->setShortcut(Qt::ALT + Qt::Key_4);
#endif
  mActions[LEFT_VIEW] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:right_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:right_view.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Right View"));
  action->setStatusTip(tr("Move Viewpoint to see object from the right."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
#ifdef __APPLE__
  action->setShortcut(Qt::META + Qt::ALT + Qt::Key_6);
#else
  action->setShortcut(Qt::ALT + Qt::Key_6);
#endif
  mActions[RIGHT_VIEW] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:top_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:top_view.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Top View"));
  action->setStatusTip(tr("Move Viewpoint to see object from the top."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
#ifdef __APPLE__
  action->setShortcut(Qt::META + Qt::ALT + Qt::Key_1);
#else
  action->setShortcut(Qt::ALT + Qt::Key_1);
#endif
  mActions[TOP_VIEW] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:bottom_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:bottom_view.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Bottom View"));
  action->setStatusTip(tr("Move Viewpoint to see object from the bottom."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
#ifdef __APPLE__
  action->setShortcut(Qt::META + Qt::ALT + Qt::Key_7);
#else
  action->setShortcut(Qt::ALT + Qt::Key_7);
#endif
  mActions[BOTTOM_VIEW] = action;

  /* OVERLAY ACTIONS */
  action = new QAction(this);
  action->setText(tr("Hide All &Camera Overlays"));
  action->setStatusTip(tr("Hide all camera overlays."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F10);
  action->setCheckable(true);
  mActions[HIDE_ALL_CAMERA_OVERLAYS] = action;

  action = new QAction(this);
  action->setText(tr("Hide All &RangeFinder Overlays"));
  action->setStatusTip(tr("Hide all range-finder overlays."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F11);
  action->setCheckable(true);
  mActions[HIDE_ALL_RANGE_FINDER_OVERLAYS] = action;

  action = new QAction(this);
  action->setText(tr("Hide All &Display Overlays"));
  action->setStatusTip(tr("Hide all display overlays."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT + Qt::Key_F12);
  action->setCheckable(true);
  mActions[HIDE_ALL_DISPLAY_OVERLAYS] = action;

  /* VIRTUAL REALITY HEADSET */
  action = new QAction(this);
  action->setText(tr("&Enable"));
  action->setStatusTip(tr("View simulation in a virtual reality headset."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_ENABLE] = action;

  action = new QAction(this);
  action->setText(tr("Track headset &position"));
  action->setStatusTip(tr("Enable virtual reality headset position tracking."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_POSITION] = action;

  action = new QAction(this);
  action->setText(tr("Track headset &orientation"));
  action->setStatusTip(tr("Enable virtual reality headset orientation tracking."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_ORIENTATION] = action;

  action = new QAction(this);
  action->setText(tr("View left eye"));
  action->setStatusTip(tr("View the left eye image in the main 3D window."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_LEFT_EYE] = action;

  action = new QAction(this);
  action->setText(tr("View right eye"));
  action->setStatusTip(tr("View the right eye image in the main 3D window."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_RIGHT_EYE] = action;

  action = new QAction(this);
  action->setText(tr("Empty view"));
  action->setStatusTip(tr("View nothing in the main 3D window."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_NO_EYE] = action;

  actionGroup = new QActionGroup(this);
  actionGroup->addAction(mActions[VIRTUAL_REALITY_HEADSET_LEFT_EYE]);
  actionGroup->addAction(mActions[VIRTUAL_REALITY_HEADSET_RIGHT_EYE]);
  actionGroup->addAction(mActions[VIRTUAL_REALITY_HEADSET_NO_EYE]);

  action = new QAction(this);
  action->setText(tr("Anti-aliasing"));
  action->setStatusTip(tr("Enable the anti-aliasing."));
  action->setToolTip(action->statusTip());
  action->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_ANTI_ALIASING] = action;

  /* ROBOT ACTIONS */
  action = new QAction(tr("Edit &Controller"), this);
  action->setShortcut(Qt::ALT + Qt::Key_C);
  action->setStatusTip(tr("Edit controller source code."));
  action->setToolTip(action->statusTip());
  mActions[EDIT_CONTROLLER] = action;

  action = new QAction(tr("Show Robot &Window"), this);
  action->setStatusTip(tr("Show the related robot window."));
  action->setToolTip(action->statusTip());
  mActions[SHOW_ROBOT_WINDOW] = action;

  /* NODE/FIELD ACTIONS */
  icon = QIcon();
  icon.addFile("enabledIcons:help_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:help_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Help..."));
  action->setStatusTip(tr("Open &documentation for this node."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
  mActions[OPEN_HELP] = action;

  icon = QIcon();
  icon.addFile("enabledIcons:reset_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:reset_button.png", QSize(), QIcon::Disabled);
  action = new QAction(this);
  action->setText(tr("&Reset to Default Value"));
  action->setStatusTip(tr("Reset to default value."));
  action->setToolTip(action->statusTip());
  action->setIcon(icon);
  action->setEnabled(false);
  mActions[RESET_VALUE] = action;

  action = new QAction(this);
  action->setText(tr("&Export"));
  action->setStatusTip(tr("Export this scene object."));
  action->setToolTip(action->statusTip());
  mActions[EXPORT_NODE] = action;

  /* PROTO ACTIONS */

  action = new QAction(this);
  action->setText(tr("&View PROTO Source"));
  action->setStatusTip(tr("Open the PROTO file in Text Editor."));
  action->setToolTip(action->statusTip());
  mActions[SHOW_PROTO_SOURCE] = action;

  action = new QAction(this);
  action->setText(tr("View Generated &PROTO Node"));
  action->setStatusTip(tr("Open the temporary file generated by the template engine in Text Editor."));
  action->setToolTip(action->statusTip());
  mActions[SHOW_PROTO_RESULT] = action;

  action = new QAction(this);
  action->setText(tr("&Convert to Base Node(s)"));
  action->setStatusTip(tr("Convert this PROTO node (and nested PROTO nodes) into the equivalent base node(s)."));
  action->setToolTip(action->statusTip());
  mActions[CONVERT_TO_BASE_NODES] = action;

  action = new QAction(this);
  action->setText(tr("Convert &Root to Base Node(s)"));
  action->setStatusTip(tr("Convert this PROTO node into the equivalent base node(s)."));
  action->setToolTip(action->statusTip());
  mActions[CONVERT_ROOT_TO_BASE_NODES] = action;

  assert(NACTIONS == mActions.size());
}

void WbActionManager::connectActions() {
  WbSimulationState *state = WbSimulationState::instance();

  connect(state, &WbSimulationState::enabledChanged, this, &WbActionManager::updateEnabled);
}

void WbActionManager::updateEnabled() {
  bool simulationEnabled = WbSimulationState::instance()->isEnabled();

  mActions[REAL_TIME]->setEnabled(simulationEnabled);
  mActions[PAUSE]->setEnabled(simulationEnabled);
  mActions[STEP]->setEnabled(simulationEnabled);
  mActions[FAST]->setEnabled(simulationEnabled);
}

void WbActionManager::setEnabled(WbActionKind kind, bool enabled) {
  mActions[kind]->setEnabled(enabled);
}

void WbActionManager::resetApplicationActionsState() {
  mActions[CUT]->setEnabled(false);
  mActions[COPY]->setEnabled(false);
  mActions[PASTE]->setEnabled(false);
  mActions[SELECT_ALL]->setEnabled(false);
  mActions[UNDO]->setEnabled(false);
  mActions[REDO]->setEnabled(false);
}

void WbActionManager::enableTextEditActions(bool enabled) {
  mActions[FIND]->setEnabled(enabled);
  mActions[FIND_NEXT]->setEnabled(enabled);
  mActions[FIND_PREVIOUS]->setEnabled(enabled);
  mActions[REPLACE]->setEnabled(enabled);
  mActions[GO_TO_LINE]->setEnabled(enabled);
  mActions[TOGGLE_LINE_COMMENT]->setEnabled(enabled);
  mActions[PRINT]->setEnabled(enabled);
  mActions[PRINT_PREVIEW]->setEnabled(enabled);
}

void WbActionManager::updateRenderingButton() {
  QAction *rendering = action(WbAction::RENDERING);

  if (WbSimulationState::instance()->isRendering()) {
    rendering->setIcon(QIcon("enabledIcons:rendering.png"));
    rendering->setChecked(true);
    rendering->setStatusTip(tr("Turn off rendering to gain better performance. (%1+4)").arg(mapControlKey()));
    rendering->setToolTip(tr("Hide Rendering. (%1+4)").arg(mapControlKey()));
  } else {
    rendering->setIcon(QIcon("enabledIcons:no_rendering.png"));
    rendering->setChecked(false);
    rendering->setStatusTip(tr("Turn on rendering to see the simulation. (%1+4)").arg(mapControlKey()));
    rendering->setToolTip(tr("Show Rendering. (%1+4)").arg(mapControlKey()));
  }
}

void WbActionManager::forwardTransformToActionToSceneTree() {
  QAction *senderAction = static_cast<QAction *>(sender());
  if (!senderAction)
    return;

  emit transformRequested(senderAction->text());
}

void WbActionManager::dispatchUserCommand() {
  QAction *senderAction = static_cast<QAction *>(sender());
  if (!senderAction)
    return;

  QObject *focusObject = mFocusObject;
  if (focusObject == NULL)
    return;

  const QString focusObjectName = focusObject->objectName();
  WbActionKind actionKind = (WbActionKind)senderAction->property("kind").toInt();
  if (focusObjectName.isEmpty())
    return;
  if (focusObjectName == "TextEditor")
    // focus on text editor
    emit userTextEditCommandReceived(actionKind);
  else if (focusObjectName == "ConsoleEdit")
    // focus on console
    emit userConsoleEditCommandReceived(actionKind);
  else if (focusObjectName == "DocumentationWebView")
    // focus on documentation
    emit userDocumentationEditCommandReceived(actionKind);
  else if (focusObjectName == "TreeView" || focusObjectName == "View3D" || focusObjectName == "ContextMenu")
    // focus on simulation view
    emit userWorldEditCommandReceived(actionKind);
}

void WbActionManager::setActionEnabledSilently(QAction *action, bool enabled) {
  action->blockSignals(true);
  action->setEnabled(enabled);
  action->blockSignals(false);
}
