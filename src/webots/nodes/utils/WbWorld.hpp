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

#ifndef WB_WORLD_HPP
#define WB_WORLD_HPP

//
// Description: Webots world
//

#include "WbWorldInfo.hpp"

#include <QtCore/QMutex>
#include <QtCore/QObject>
#include <QtCore/QString>

class WbGroup;
class WbNode;
class WbPerspective;
class WbRobot;
class WbSolid;
class WbTokenizer;
class WbViewpoint;

struct dImmersionGeom;
class WbOdeContact;

class WbWorld : public QObject {
  Q_OBJECT;

public:
  // unique world instance (can be NULL)
  static WbWorld *instance();

  // constructor
  // the world is read using 'tokenizer': the file syntax must have been checked with WbParser
  // if 'tokenizer' is not specified, the world is created with default WorldInfo and Viewpoint nodes
  explicit WbWorld(WbTokenizer *tokenizer = NULL);

  // destructor
  virtual ~WbWorld();

  void finalize();

  // current file name
  const QString &fileName() const { return mFileName; }
  bool needSaving() const;
  bool isModifiedFromSceneTree() const { return mIsModifiedFromSceneTree; }
  bool isModified() const { return mIsModified; }
  void setModified(bool isModified = true);
  void setModifiedFromSceneTree();

  // world loading functions
  bool isLoading() const { return mIsLoading; }
  void setIsLoading(bool loading) { mIsLoading = loading; }
  bool isCleaning() const { return mIsCleaning; }
  void setIsCleaning(bool cleaning) { mIsCleaning = cleaning; }
  bool wasWorldLoadingCanceled() { return mWorldLoadingCanceled; }

  bool isVideoRecording() const { return mIsVideoRecording; }

  static QString defaultX3dFrustumCullingParameter() { return "true"; }
  static void enableX3DMetaFileExport() { cX3DMetaFileExport = true; }
  static bool isX3DStreaming() { return cX3DStreaming; }
  static void enableX3DStreaming() { cX3DStreaming = true; }
  static bool printExternUrls() { return cPrintExternUrls; }
  static void setPrintExternUrls() { cPrintExternUrls = true; }

  // save
  bool save();
  virtual bool saveAs(const QString &fileName);

  // save and replace Webots specific nodes by VRML/X3D nodes
  bool exportAsHtml(const QString &fileName, bool animation) const;
  bool exportAsX3d(const QString &fileName) const;
  void write(WbWriter &writer) const;

  // nodes that do always exist
  WbGroup *root() const { return mRoot; }
  WbWorldInfo *worldInfo() const { return mWorldInfo; }
  WbViewpoint *viewpoint() const { return mViewpoint; }
  void setWorldInfo(WbWorldInfo *worldInfo) { mWorldInfo = worldInfo; }
  void setViewpoint(WbViewpoint *viewpoint);
  double orthographicViewHeight() const;
  void setOrthographicViewHeight(double ovh) const;

  // current perspective
  WbPerspective *perspective() const { return mPerspective; }
  bool reloadPerspective();

  // find a solid by its "name" field
  WbSolid *findSolid(const QString &name) const;

  // create a list of all solids (on the fly), look recursively
  // if 'visibleNodes' is true: return list of Solid nodes visible in the scene tree
  // if 'visibleNodes' is false: return instantiated Solid nodes (i.e. excluding proto parameter nodes)
  QList<WbSolid *> findSolids(bool visibleNodes = false) const;

  // return the list of all robots
  QList<WbRobot *> robots() const { return mRobots; }

  // return the list of all top solids (not looking recursively)
  QList<WbSolid *> topSolids() const { return mTopSolids; }

  // return the list of all solids that have a positive radar cross-section (radar target)
  QList<WbSolid *> radarTargetSolids() const { return mRadarTargets; }
  void addRadarTarget(WbSolid *target) { mRadarTargets.append(target); }
  void removeRadarTarget(WbSolid *target) { mRadarTargets.removeAll(target); }

  // return the list of all solids that have a non-empty 'recognitionColors' field
  QList<WbSolid *> cameraRecognitionObjects() const { return mCameraRecognitionObjects; }
  void addCameraRecognitionObject(WbSolid *object) { mCameraRecognitionObjects.append(object); }
  void removeCameraRecognitionObject(WbSolid *object) { mCameraRecognitionObjects.removeAll(object); }

  // functions to maintain global list of robots
  void removeRobotIfPresent(WbRobot *robot);
  void addRobotIfNotAlreadyPresent(WbRobot *robot);

  // return the list of texture files used in this world (no duplicates)
  QList<QPair<QString, WbMFString *>> listTextureFiles() const;

  // shortcut
  double basicTimeStep() const { return mWorldInfo->basicTimeStep(); }
  int optimalThreadCount() const { return mWorldInfo->optimalThreadCount(); }

  const QList<WbOdeContact> &odeContacts() const { return mOdeContacts; }
  const QList<dImmersionGeom> &immersionGeoms() const { return mImmersionGeoms; }
  void appendOdeContact(const WbOdeContact &odeContact);
  void appendOdeImmersionGeom(const dImmersionGeom &immersionGeom);

  void retrieveNodeNamesWithOptionalRendering(QStringList &centerOfMassNodeNames, QStringList &centerOfBuoyancyNodeNames,
                                              QStringList &supportPolygonNodeNames) const;

  void setResetRequested(bool restartControllers) {
    mResetRequested = true;
    if (!mRestartControllers)
      mRestartControllers = restartControllers;
  }
  virtual void reset(bool restartControllers) {
    mResetRequested = false;
    mRestartControllers = false;
  }

signals:
  void modificationChanged(bool modified);
  void worldLoadingStatusHasChanged(QString status);
  void worldLoadingHasProgressed(int percent);
  void viewpointChanged();
  void robotAdded(WbRobot *robot);
  void resetRequested(bool restartControllers);

public slots:
  void awake();
  void updateVideoRecordingStatus(int status) {
    mIsVideoRecording = (status == WB_SUPERVISOR_MOVIE_RECORDING || status == WB_SUPERVISOR_MOVIE_SAVING);
  }

protected:
  // collecting contact and immersion geometries
  QList<WbOdeContact> mOdeContacts;
  QList<dImmersionGeom> mImmersionGeoms;
  bool mWorldLoadingCanceled;
  bool mResetRequested;
  bool mRestartControllers;

  QString logWorldMetrics() const;

  // called when a node is added to the children of a group which checks if a
  // controller needs starting, should the added node be a Robot
  virtual void setUpControllerForNewRobot(WbRobot *robot) {}

protected slots:
  virtual void storeAddedNodeIfNeeded(WbNode *node) {}

private:
  QString mFileName;
  bool mIsModified;
  bool mIsModifiedFromSceneTree;
  WbGroup *mRoot;
  WbWorldInfo *mWorldInfo;
  WbViewpoint *mViewpoint;
  WbPerspective *mPerspective;
  QList<WbRobot *> mRobots;
  QList<WbSolid *> mTopSolids;
  QList<WbSolid *> mRadarTargets;
  QList<WbSolid *> mCameraRecognitionObjects;
  QMutex mOdeContactsMutex;
  double mLastAwakeningTime;
  bool mIsLoading;
  bool mIsCleaning;
  bool mIsVideoRecording;

  void checkPresenceOfMandatoryNodes();
  WbNode *findTopLevelNode(const QString &modelName, int preferredPosition) const;

  virtual void storeLastSaveTime(){};
  void createX3DMetaFile(const QString &filename) const;

  static bool cX3DMetaFileExport;
  static bool cX3DStreaming;
  static bool cPrintExternUrls;

private slots:
  void updateProjectPath(const QString &oldPath, const QString &newPath);
  void updateTopLevelLists();
};

#endif
