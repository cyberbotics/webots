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

#include "WbActionManager.hpp"

#include "WbSimulationState.hpp"
#include "WbUndoStack.hpp"

#ifdef __linux__
#include <QtCore/QCoreApplication>
#endif
#include <QtGui/QAction>
#include <QtGui/QActionGroup>
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

const QString WbActionManager::mapControlKey() {
#ifdef __APPLE__
  return "⌘";
#else
  return "Ctrl";
#endif
}

void WbActionManager::populateActions() {
  QAction *newAction;
  QIcon icon;

  /* WORLD and SIMULATION ACTIONS */
  icon = QIcon();
  icon.addFile("enabledIcons:open_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:open_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Open World..."));
  newAction->setStatusTip(tr("Open an existing world file. (%1+Shift+O)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::CTRL | Qt::Key_O);
  newAction->setIcon(icon);
  mActions[OPEN_WORLD] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Open Sample World..."));
  newAction->setStatusTip(tr("Open a sample world."));
  newAction->setToolTip(newAction->statusTip());
  mActions[OPEN_SAMPLE_WORLD] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:save_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:save_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Save World"));
  newAction->setStatusTip(tr("Save the current world file. (%1+Shift+S)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::CTRL | Qt::Key_S);
  newAction->setIcon(icon);
  mActions[SAVE_WORLD] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Save World &As..."));
  newAction->setStatusTip(tr("Save the current world file with a new name."));
  newAction->setToolTip(newAction->statusTip());
  mActions[SAVE_WORLD_AS] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:reload_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:reload_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Reload World"));
  newAction->setStatusTip(
    tr("Reload World.\nReload the current world file and restart the simulation. (%1+Shift+R)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
  newAction->setShortcut(Qt::SHIFT | Qt::CTRL | Qt::Key_R);
  mActions[RELOAD_WORLD] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:reset_simulation_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:reset_simulation_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("Reset Simulation"));
  newAction->setStatusTip(tr("Reset Simulation.\nRestore initial state of the simulation. (%1+Shift+T)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::CTRL | Qt::Key_T);
  newAction->setIcon(icon);
  mActions[RESET_SIMULATION] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:real_time_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:real_time_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("Real-&time"));
  newAction->setStatusTip(tr("Run the simulation in real-time. (%1+2)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_2);
  newAction->setIcon(icon);
  mActions[REAL_TIME] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:pause_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:pause_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Pause"));
  newAction->setStatusTip(tr("Pause the simulation. (%1+0)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_0);
  newAction->setIcon(icon);
  mActions[PAUSE] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:step_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:step_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("St&ep"));
  newAction->setStatusTip(tr("Execute one simulation step. (%1+1)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_1);
  newAction->setIcon(icon);
  mActions[STEP] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:fast_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:fast_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Fast"));
  newAction->setStatusTip(tr("Run the simulation as fast as possible. (%1+3)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_3);
  newAction->setIcon(icon);
  mActions[FAST] = newAction;

  newAction = new QAction(this);
  newAction->setCheckable(true);
  newAction->setShortcut(Qt::CTRL | Qt::Key_4);
  newAction->setText(tr("&Rendering"));
  mActions[RENDERING] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Unmute sound"));
  newAction->setStatusTip(tr("Unmute the sound on the main audio device of the computer."));
  newAction->setToolTip(newAction->statusTip());
  // the icon is inverted there to respect the other applications behavior
  newAction->setIcon(QIcon("enabledIcons:sound_mute_button.png"));
  mActions[SOUND_UNMUTE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Mute sound"));
  newAction->setStatusTip(tr("Mute the sound on the main audio device of the computer."));
  newAction->setToolTip(newAction->statusTip());
  // the icon is inverted there to respect the other applications behavior
  newAction->setIcon(QIcon("enabledIcons:sound_unmute_button.png"));
  mActions[SOUND_MUTE] = newAction;

  newAction = new QAction(this);
  mActions[ANIMATION] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Take Screenshot..."));
  newAction->setStatusTip(tr("Save the current image of the simulation. (%1 + SHIFT + P)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::SHIFT | Qt::Key_P);
  newAction->setIcon(QIcon("enabledIcons:screenshot_button.png"));
  mActions[TAKE_SCREENSHOT] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Perspective Projection"));
  newAction->setStatusTip(tr("Perspective viewing projection."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::Key_F9);
  newAction->setCheckable(true);
  mActions[PERSPECTIVE_PROJECTION] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Orthographic Projection"));
  newAction->setStatusTip(tr("Orthographic viewing projection."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::Key_F10);
  newAction->setCheckable(true);
  mActions[ORTHOGRAPHIC_PROJECTION] = newAction;

  QActionGroup *actionGroup = new QActionGroup(this);
  actionGroup->addAction(mActions[PERSPECTIVE_PROJECTION]);
  actionGroup->addAction(mActions[ORTHOGRAPHIC_PROJECTION]);

  newAction = new QAction(this);
  newAction->setText(tr("&Plain Rendering"));
  newAction->setStatusTip(tr("Plain OpenGL rendering."));
  newAction->setToolTip(newAction->statusTip());
#ifdef __APPLE__
  newAction->setShortcut(Qt::SHIFT | Qt::Key_P);
#else
  newAction->setShortcut(Qt::Key_F11);
#endif
  newAction->setCheckable(true);
  mActions[PLAIN_RENDERING] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Wireframe Rendering"));
  newAction->setStatusTip(tr("Rendering only the segments between the vertices."));
  newAction->setToolTip(newAction->statusTip());
#ifdef __APPLE__
  newAction->setShortcut(Qt::SHIFT | Qt::Key_W);
#else
  newAction->setShortcut(Qt::Key_F12);
#endif
  newAction->setCheckable(true);
  mActions[WIREFRAME_RENDERING] = newAction;

  actionGroup = new QActionGroup(this);
  actionGroup->addAction(mActions[PLAIN_RENDERING]);
  actionGroup->addAction(mActions[WIREFRAME_RENDERING]);

  // ----- optional rendering -----
  newAction = new QAction(this);
  newAction->setText(tr("Show &Coordinate System"));
  newAction->setStatusTip(tr("Show coordinate system."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F1);
  newAction->setCheckable(true);
  mActions[COORDINATE_SYSTEM] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show All &Bounding Objects"));
  newAction->setStatusTip(tr("Show all bounding objects."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F2);
  newAction->setCheckable(true);
  mActions[BOUNDING_OBJECT] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Contact &Points"));
  newAction->setStatusTip(tr("Show contact points and polygons."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F3);
  newAction->setCheckable(true);
  mActions[CONTACT_POINTS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Connector &Axes"));
  newAction->setStatusTip(tr("Show connector axes."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F4);
  newAction->setCheckable(true);
  mActions[CONNECTOR_AXES] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show &Joint Axes"));
  newAction->setStatusTip(tr("Show joint axes."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F5);
  newAction->setCheckable(true);
  mActions[JOINT_AXES] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Range&Finder Frustums"));
  newAction->setStatusTip(tr("Show range-finder frustums."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F6);
  newAction->setCheckable(true);
  mActions[RANGE_FINDER_FRUSTUMS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Lidar &Ray Paths"));
  newAction->setStatusTip(tr("Show lidar rays paths."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F7);
  newAction->setCheckable(true);
  mActions[LIDAR_RAYS_PATH] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Lidar Point Cl&oud"));
  newAction->setStatusTip(tr("Show lidar point cloud."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F8);
  newAction->setCheckable(true);
  mActions[LIDAR_POINT_CLOUD] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show &Camera Frustums"));
  newAction->setStatusTip(tr("Show camera frustums."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F9);
  newAction->setCheckable(true);
  mActions[CAMERA_FRUSTUM] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show &DistanceSensor Rays"));
  newAction->setStatusTip(tr("Show distance sensors rays."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F10);
  newAction->setCheckable(true);
  mActions[DISTANCE_SENSOR_RAYS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show &LightSensor Rays"));
  newAction->setStatusTip(tr("Show light sensors rays."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F11);
  newAction->setCheckable(true);
  mActions[LIGHT_SENSOR_RAYS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show L&ight Positions"));
  newAction->setStatusTip(tr("Show position of light sources."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F12);
  newAction->setCheckable(true);
  mActions[LIGHT_POSITIONS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Center of Buo&yancy"));
  newAction->setStatusTip(tr("Show the center of buoyancy of a solid."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F3);
  newAction->setCheckable(true);
  mActions[CENTER_OF_BUOYANCY] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show &Pen Painting Rays"));
  newAction->setStatusTip(tr("Show pen painting rays."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F1);
  newAction->setCheckable(true);
  mActions[PEN_PAINTING_RAYS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Center of &Mass"));
  newAction->setStatusTip(tr("Show the center of mass of a solid."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F2);
  newAction->setCheckable(true);
  mActions[CENTER_OF_MASS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show S&upport Polygon"));
  newAction->setStatusTip(tr("Show the center of mass and the support polygon of a solid."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F4);
  newAction->setCheckable(true);
  mActions[SUPPORT_POLYGON] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show S&kin Skeleton"));
  newAction->setStatusTip(tr("Turn on visual representation of skeleton used by the Skin device."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F9);
  newAction->setCheckable(true);
  mActions[SKIN_SKELETON] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Radar Frustums"));
  newAction->setStatusTip(tr("Show radar frustums."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F6);
  newAction->setCheckable(true);
  mActions[RADAR_FRUSTUMS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Normals"));
  newAction->setStatusTip(tr("Show IndexedFaceSet and Mesh nodes normals."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F5);
  newAction->setCheckable(true);
  mActions[NORMALS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Physics Clusters"));
  newAction->setStatusTip(tr("Show visual representation of ODE clusters."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[PHYSICS_CLUSTERS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Show Bounding Sphere"));
  newAction->setStatusTip(tr("Show visual representaton of the selected node's bounding sphere."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[BOUNDING_SPHERE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Lock Viewpoint"));
  newAction->setStatusTip(tr("Disable Viewpoint translation and rotation from 3D view."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[LOCK_VIEWPOINT] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Disable Selection"));
  newAction->setStatusTip(tr("Disable selection change from 3D view."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[DISABLE_SELECTION] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Disable 3D View Context Menu"));
  newAction->setStatusTip(tr("Disable opening the context menu clicking on the 3D view."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[DISABLE_3D_VIEW_CONTEXT_MENU] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Disable Object Move"));
  newAction->setStatusTip(tr("Disable moving objects from 3D view."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[DISABLE_OBJECT_MOVE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Disable Applying Force and Torque"));
  newAction->setStatusTip(tr("Disable applying force and torque to objects from 3D view."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[DISABLE_FORCE_AND_TORQUE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Disable Rendering"));
  newAction->setStatusTip(tr("Disable activating the rendering."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[DISABLE_RENDERING] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:insert_after_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:insert_after_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setEnabled(false);
  newAction->setText(tr("&Add New"));
  newAction->setStatusTip(tr("Add a new object or import an object. (%1+Shift+A)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
  newAction->setShortcut(Qt::CTRL | Qt::SHIFT | Qt::Key_A);
  mActions[ADD_NEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:delete_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:delete_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setEnabled(false);
  newAction->setText(tr("&Delete"));
  newAction->setStatusTip(tr("Delete the selected object. (Del)"));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::Key_Delete);
  newAction->setIcon(icon);
  newAction->setProperty("kind", DEL);
  connect(newAction, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[DEL] = newAction;

  /* GENERAL ACTIONS  */
  icon = QIcon();
  icon.addFile("enabledIcons:cut_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:cut_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setEnabled(false);
  newAction->setText(tr("C&ut"));
  newAction->setStatusTip(tr("Cut object at the selected line. (%1+X)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcuts(QKeySequence::Cut);
  newAction->setIcon(icon);
  newAction->setProperty("kind", CUT);
  connect(newAction, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[CUT] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:copy_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:copy_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setEnabled(false);
  newAction->setText(tr("&Copy"));
  newAction->setStatusTip(tr("Copy object at the selected line. (%1+C)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcuts(QKeySequence::Copy);
  newAction->setIcon(icon);
  newAction->setProperty("kind", COPY);
  connect(newAction, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[COPY] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:paste_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:paste_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setEnabled(false);
  newAction->setText(tr("&Paste"));
  newAction->setStatusTip(tr("Paste or insert the clipboard object. (%1+V)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcuts(QKeySequence::Paste);
  newAction->setIcon(icon);
  newAction->setProperty("kind", PASTE);
  connect(newAction, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[PASTE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Select &All"));
  newAction->setStatusTip(tr("Select all text. (%1+A)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(QKeySequence::SelectAll);
  newAction->setProperty("kind", SELECT_ALL);
  connect(newAction, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[SELECT_ALL] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Undo"));
  newAction->setStatusTip(tr("Undo manual modification to the simulation world and edited text. (%1+Z)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcuts(QKeySequence::Undo);
  newAction->setProperty("kind", UNDO);
  connect(newAction, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[UNDO] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Redo"));
  newAction->setStatusTip(tr("Redo manual modification to the simulation world and edited text. (%1+Y)").arg(mapControlKey()));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcuts(QKeySequence::Redo);
  newAction->setProperty("kind", REDO);
  connect(newAction, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[REDO] = newAction;

  /* TEXT EDIT ACTIONS  */

  icon = QIcon();
  icon.addFile("enabledIcons:new_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:new_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&New Text File"));
  newAction->setStatusTip(tr("Create a new text file."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_N);
  newAction->setIcon(icon);
  mActions[NEW_FILE] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:open_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:open_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Open Text File..."));
  newAction->setStatusTip(tr("Open an existing text file."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_O);
  newAction->setIcon(icon);
  mActions[OPEN_FILE] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:save_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:save_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Save Text File"));
  newAction->setStatusTip(tr("Save the current text file."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_S);
  newAction->setIcon(icon);
  mActions[SAVE_FILE] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:save_as_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:save_as_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("Save Text File &As..."));
  newAction->setStatusTip(tr("Save the current text file with a new name."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
  mActions[SAVE_FILE_AS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Save All Text Files"));
  newAction->setStatusTip(tr("Save all the opened text files."));
  newAction->setToolTip(newAction->statusTip());
  mActions[SAVE_ALL_FILES] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:reload_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:reload_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Revert Text File"));
  newAction->setStatusTip(tr("Revert the current text file."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
  newAction->setShortcut(Qt::CTRL | Qt::Key_R);
  mActions[REVERT_FILE] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:find_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:find_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Find..."));
  newAction->setStatusTip(tr("Find text in current file."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_F);
  newAction->setIcon(icon);
  newAction->setProperty("kind", FIND);
  connect(newAction, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[FIND] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Find &Next"));
  newAction->setStatusTip(tr("Find next occurence of search string in current file."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_G);
  newAction->setProperty("kind", FIND_NEXT);
  connect(newAction, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[FIND_NEXT] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Find &Previous"));
  newAction->setStatusTip(tr("Find previous occurence of search string in current file."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::CTRL | Qt::Key_G);
  newAction->setProperty("kind", FIND_PREVIOUS);
  connect(newAction, &QAction::triggered, this, &WbActionManager::dispatchUserCommand);
  mActions[FIND_PREVIOUS] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:replace_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:replace_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Replace..."));
  newAction->setStatusTip(tr("Replace text in current file."));
  newAction->setToolTip(newAction->statusTip());
#ifdef __APPLE__  // on Mac, CTRL+H is a system shortcut to hide the window
  newAction->setShortcut(Qt::CTRL | Qt::ALT | Qt::Key_F);
#else
  newAction->setShortcut(Qt::CTRL | Qt::Key_H);
#endif
  newAction->setIcon(icon);
  mActions[REPLACE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Go to line..."));
  newAction->setStatusTip(tr("Go to specified line number."));
  newAction->setToolTip(newAction->statusTip());
  mActions[GO_TO_LINE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Toggle Line Comment") + "    ");  // add spaces because the spacing between the menu text
  // and the hotkey text does not expand to adjust for the text length
  newAction->setStatusTip(tr("Toggle comment of selected lines."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_Slash);
  mActions[TOGGLE_LINE_COMMENT] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Print..."));
  newAction->setStatusTip(tr("Print text file."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_P);
  mActions[PRINT] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Print Preview..."));
  newAction->setStatusTip(tr("Preview of printed text file."));
  newAction->setToolTip(newAction->statusTip());
  mActions[PRINT_PREVIEW] = newAction;

  /* CONSOLE ACTIONS */
  newAction = new QAction(this);
  newAction->setText(tr("&Clear All Consoles"));
  newAction->setStatusTip(tr("Clears all the Consoles."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_K);
  mActions[CLEAR_CONSOLE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&New Console"));
  newAction->setStatusTip(tr("Opens a new Console."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::CTRL | Qt::Key_N);
  mActions[NEW_CONSOLE] = newAction;

  /* VIEWPOINT ACTIONS */

  newAction = new QAction(this);
  newAction->setText(tr("&None"));
  newAction->setStatusTip(tr("Do not follow the object."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[FOLLOW_NONE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Tracking Shot"));
  newAction->setStatusTip(tr("Translate the camera to follow the object."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::Key_F5);
  newAction->setCheckable(true);
  mActions[FOLLOW_TRACKING] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Mounted Shot"));
  newAction->setStatusTip(tr("Translate and rotate the camera to follow the object."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F5);
  newAction->setCheckable(true);
  mActions[FOLLOW_MOUNTED] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Pan and Tilt Shot"));
  newAction->setStatusTip(tr("Rotate the camera to always look at the object center."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F7);
  newAction->setCheckable(true);
  mActions[FOLLOW_PAN_AND_TILT] = newAction;

  actionGroup->addAction(mActions[FOLLOW_NONE]);
  actionGroup->addAction(mActions[FOLLOW_TRACKING]);
  actionGroup->addAction(mActions[FOLLOW_MOUNTED]);
  actionGroup->addAction(mActions[FOLLOW_PAN_AND_TILT]);

  icon = QIcon();
  icon.addFile("enabledIcons:move_viewpoint_to_object_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:move_viewpoint_to_object_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Move Viewpoint to Object"));
#ifdef __APPLE__  // on Mac, CTRL+H is a system shortcut to hide the window
  newAction->setStatusTip(tr("Move viewpoint to selected object. (CTRL + ALT + 5)"));
#else
  newAction->setStatusTip(tr("Move viewpoint to selected object. (ALT + 5)"));
#endif
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::SHIFT | Qt::META | Qt::ALT | Qt::Key_1);
#else
  newAction->setShortcut(Qt::SHIFT | Qt::ALT | Qt::Key_1);
#endif
  mActions[MOVE_VIEWPOINT_TO_OBJECT] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:front_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:front_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Front View"));
  newAction->setStatusTip(tr("Move Viewpoint to see object from the front."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::SHIFT | Qt::META | Qt::ALT | Qt::Key_2);
#else
  newAction->setShortcut(Qt::SHIFT | Qt::ALT | Qt::Key_2);
#endif
  mActions[OBJECT_FRONT_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:back_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:back_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Back View"));
  newAction->setStatusTip(tr("Move Viewpoint to see object from the back."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::SHIFT | Qt::META | Qt::ALT | Qt::Key_3);
#else
  newAction->setShortcut(Qt::SHIFT | Qt::ALT | Qt::Key_3);
#endif
  mActions[OBJECT_BACK_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:left_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:left_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Left View"));
  newAction->setStatusTip(tr("Move Viewpoint to see object from the left."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::SHIFT | Qt::META | Qt::ALT | Qt::Key_4);
#else
  newAction->setShortcut(Qt::SHIFT | Qt::ALT | Qt::Key_4);
#endif
  mActions[OBJECT_LEFT_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:right_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:right_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Right View"));
  newAction->setStatusTip(tr("Move Viewpoint to see object from the right."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::SHIFT | Qt::META | Qt::ALT | Qt::Key_5);
#else
  newAction->setShortcut(Qt::SHIFT | Qt::ALT | Qt::Key_5);
#endif
  mActions[OBJECT_RIGHT_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:top_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:top_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Top View"));
  newAction->setStatusTip(tr("Move Viewpoint to see object from the top."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::SHIFT | Qt::META | Qt::ALT | Qt::Key_6);
#else
  newAction->setShortcut(Qt::SHIFT | Qt::ALT | Qt::Key_6);
#endif
  mActions[OBJECT_TOP_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:bottom_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:bottom_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Bottom View"));
  newAction->setStatusTip(tr("Move Viewpoint to see object from the bottom."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::SHIFT | Qt::META | Qt::ALT | Qt::Key_7);
#else
  newAction->setShortcut(Qt::SHIFT | Qt::ALT | Qt::Key_7);
#endif
  mActions[OBJECT_BOTTOM_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:front_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:front_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Change View"));
  newAction->setStatusTip(tr("Open standard Viewpoint positions menu."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
  mActions[VIEW_MENU] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:restore_viewpoint_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:restore_viewpoint_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("Restore &Viewpoint"));
  newAction->setStatusTip(tr("Restore the initial Viewpoint position and orientation. (CTRL + SHIFT + V)"));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::META | Qt::ALT | Qt::Key_1);
#else
  newAction->setShortcut(Qt::ALT | Qt::Key_1);
#endif
  mActions[RESTORE_VIEWPOINT] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:front_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:front_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&East View"));
  newAction->setStatusTip(tr("Move Viewpoint to see the world from east."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::META | Qt::ALT | Qt::Key_2);
#else
  newAction->setShortcut(Qt::ALT | Qt::Key_2);
#endif
  mActions[EAST_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:back_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:back_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&West View"));
  newAction->setStatusTip(tr("Move Viewpoint to see the world from west."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::META | Qt::ALT | Qt::Key_3);
#else
  newAction->setShortcut(Qt::ALT | Qt::Key_3);
#endif
  mActions[WEST_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:left_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:left_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&North View"));
  newAction->setStatusTip(tr("Move Viewpoint to see the world from north."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::META | Qt::ALT | Qt::Key_4);
#else
  newAction->setShortcut(Qt::ALT | Qt::Key_4);
#endif
  mActions[NORTH_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:right_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:right_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&South View"));
  newAction->setStatusTip(tr("Move Viewpoint to see the world from south."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::META | Qt::ALT | Qt::Key_5);
#else
  newAction->setShortcut(Qt::ALT | Qt::Key_5);
#endif
  mActions[SOUTH_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:top_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:top_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Top View"));
  newAction->setStatusTip(tr("Move Viewpoint to see the world from the top."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::META | Qt::ALT | Qt::Key_6);
#else
  newAction->setShortcut(Qt::ALT | Qt::Key_6);
#endif
  mActions[TOP_VIEW] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:bottom_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:bottom_view.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Bottom View"));
  newAction->setStatusTip(tr("Move Viewpoint to see the world from the bottom."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
#ifdef __APPLE__
  newAction->setShortcut(Qt::META | Qt::ALT | Qt::Key_7);
#else
  newAction->setShortcut(Qt::ALT | Qt::Key_7);
#endif
  mActions[BOTTOM_VIEW] = newAction;

  /* OVERLAY ACTIONS */
  newAction = new QAction(this);
  newAction->setText(tr("Hide All &Camera Overlays"));
  newAction->setStatusTip(tr("Hide all camera overlays."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F10);
  newAction->setCheckable(true);
  mActions[HIDE_ALL_CAMERA_OVERLAYS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Hide All &RangeFinder Overlays"));
  newAction->setStatusTip(tr("Hide all range-finder overlays."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F11);
  newAction->setCheckable(true);
  mActions[HIDE_ALL_RANGE_FINDER_OVERLAYS] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Hide All &Display Overlays"));
  newAction->setStatusTip(tr("Hide all display overlays."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setShortcut(Qt::SHIFT | Qt::Key_F12);
  newAction->setCheckable(true);
  mActions[HIDE_ALL_DISPLAY_OVERLAYS] = newAction;

  /* VIRTUAL REALITY HEADSET */
  newAction = new QAction(this);
  newAction->setText(tr("&Enable"));
  newAction->setStatusTip(tr("View simulation in a virtual reality headset."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_ENABLE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Track headset &position"));
  newAction->setStatusTip(tr("Enable virtual reality headset position tracking."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_POSITION] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Track headset &orientation"));
  newAction->setStatusTip(tr("Enable virtual reality headset orientation tracking."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_ORIENTATION] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("View left eye"));
  newAction->setStatusTip(tr("View the left eye image in the main 3D window."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_LEFT_EYE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("View right eye"));
  newAction->setStatusTip(tr("View the right eye image in the main 3D window."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_RIGHT_EYE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Empty view"));
  newAction->setStatusTip(tr("View nothing in the main 3D window."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_NO_EYE] = newAction;

  actionGroup = new QActionGroup(this);
  actionGroup->addAction(mActions[VIRTUAL_REALITY_HEADSET_LEFT_EYE]);
  actionGroup->addAction(mActions[VIRTUAL_REALITY_HEADSET_RIGHT_EYE]);
  actionGroup->addAction(mActions[VIRTUAL_REALITY_HEADSET_NO_EYE]);

  newAction = new QAction(this);
  newAction->setText(tr("Anti-aliasing"));
  newAction->setStatusTip(tr("Enable the anti-aliasing."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setCheckable(true);
  mActions[VIRTUAL_REALITY_HEADSET_ANTI_ALIASING] = newAction;

  /* ROBOT ACTIONS */
  newAction = new QAction(tr("Edit &Controller"), this);
  newAction->setShortcut(Qt::ALT | Qt::Key_C);
  newAction->setStatusTip(tr("Edit controller source code."));
  newAction->setToolTip(newAction->statusTip());
  mActions[EDIT_CONTROLLER] = newAction;

  newAction = new QAction(tr("Show Robot &Window"), this);
  newAction->setStatusTip(tr("Show the related robot window."));
  newAction->setToolTip(newAction->statusTip());
  mActions[SHOW_ROBOT_WINDOW] = newAction;

  /* NODE/FIELD ACTIONS */
  icon = QIcon();
  icon.addFile("enabledIcons:help_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:help_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Help..."));
  newAction->setStatusTip(tr("Open &documentation for this node."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
  mActions[OPEN_HELP] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:reset_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:reset_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Reset to Default Value"));
  newAction->setStatusTip(tr("Reset to default value."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
  newAction->setEnabled(false);
  mActions[RESET_VALUE] = newAction;

  icon = QIcon();
  icon.addFile("enabledIcons:edit_field_button.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:edit_field_button.png", QSize(), QIcon::Disabled);
  newAction = new QAction(this);
  newAction->setText(tr("&Edit..."));
  newAction->setStatusTip(tr("Open field/node editor."));
  newAction->setToolTip(newAction->statusTip());
  newAction->setIcon(icon);
  newAction->setEnabled(false);
  mActions[EDIT_FIELD] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Export URDF"));
  newAction->setStatusTip(tr("Export this robot model to URDF."));
  newAction->setToolTip(newAction->statusTip());
  mActions[EXPORT_URDF] = newAction;

  /* PROTO ACTIONS */

  newAction = new QAction(this);
  newAction->setText(tr("Edit PROTO &Source"));
  newAction->setStatusTip(tr("Edit the PROTO file in Text Editor."));
  newAction->setToolTip(newAction->statusTip());
  mActions[EDIT_PROTO_SOURCE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&View PROTO Source"));
  newAction->setStatusTip(tr("Open the PROTO file in Text Editor in read-only mode."));
  newAction->setToolTip(newAction->statusTip());
  mActions[SHOW_PROTO_SOURCE] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("View Generated &PROTO Node"));
  newAction->setStatusTip(tr("Open the temporary file generated by the template engine in Text Editor."));
  newAction->setToolTip(newAction->statusTip());
  mActions[SHOW_PROTO_RESULT] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("&Convert to Base Node(s)"));
  newAction->setStatusTip(tr("Convert this PROTO node (and nested PROTO nodes) into the equivalent base node(s)."));
  newAction->setToolTip(newAction->statusTip());
  mActions[CONVERT_TO_BASE_NODES] = newAction;

  newAction = new QAction(this);
  newAction->setText(tr("Convert &Root to Base Node(s)"));
  newAction->setStatusTip(tr("Convert this PROTO node into the equivalent base node(s)."));
  newAction->setToolTip(newAction->statusTip());
  mActions[CONVERT_ROOT_TO_BASE_NODES] = newAction;

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

void WbActionManager::enableTextEditActions(bool enabled, bool isReadOnly) {
  mActions[FIND]->setEnabled(enabled);
  mActions[FIND_NEXT]->setEnabled(enabled);
  mActions[FIND_PREVIOUS]->setEnabled(enabled);
  mActions[REPLACE]->setEnabled(enabled && !isReadOnly);
  mActions[GO_TO_LINE]->setEnabled(enabled);
  mActions[TOGGLE_LINE_COMMENT]->setEnabled(enabled && !isReadOnly);
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

  QObject *currentFocusObject = mFocusObject;
  if (currentFocusObject == NULL)
    return;

  const QString focusObjectName = currentFocusObject->objectName();
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
