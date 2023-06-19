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

#ifndef WB_SUPERVISOR_UTILITIES_HPP
#define WB_SUPERVISOR_UTILITIES_HPP

#include "WbSimulationState.hpp"

#include <QtCore/QObject>
#include <QtCore/QVector>

class QDataStream;

struct WbUpdatedFieldInfo;
struct WbFieldGetRequest;
struct WbTrackedFieldInfo;
struct WbTrackedPoseInfo;
struct WbTrackedContactPointInfo;
class WbFieldSetRequest;

class WbBaseNode;
class WbDataStream;
class WbNode;
class WbRobot;
class WbPose;
class WbSolid;
class WbWrenLabelOverlay;
class WbField;

class WbSupervisorUtilities : public QObject {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbSupervisorUtilities(WbRobot *robot);
  virtual ~WbSupervisorUtilities();

  void handleMessage(QDataStream &stream);
  void writeAnswer(WbDataStream &stream);
  void writeConfigure(WbDataStream &stream);
  void processImmediateMessages(bool blockRegeneration = false);
  void postPhysicsStep();
  void reset();  // should be called when controllers are restarted

  bool shouldBeRemoved() const { return mShouldRemoveNode; }
  QStringList labelsState() const;

signals:
  void worldModified();
  void changeSimulationModeRequested(int newMode);
  void labelChanged(const QString &labelDescription);  // i.e. json format

private slots:
  void animationStartStatusChanged(int status);
  void animationStopStatusChanged(int status);
  void movieStatusChanged(int status);
  void changeSimulationMode(int newMode);
  void updateDeletedNodeList(WbNode *node);
  void notifyNodeUpdate(WbNode *node);
  void notifyFieldUpdate();
  void updateProtoRegeneratedFlag(WbNode *node);
  void removeTrackedContactPoints(QObject *obj);
  void removeTrackedPoseNode(QObject *obj);
  void removeTrackedField(QObject *obj);

private:
  WbRobot *mRobot;
  int mFoundNodeUniqueId;
  int mFoundNodeType;
  int mFoundNodeTag;
  QString mFoundNodeModelName;
  QString mCurrentDefName;
  int mFoundNodeParentUniqueId;
  bool mFoundNodeIsProto;
  bool mFoundNodeIsProtoInternal;
  int mFoundFieldIndex;
  int mFoundFieldType;
  int mFoundFieldCount;
  QString mFoundFieldName;
  bool mFoundFieldIsInternal;
  int mNodeFieldCount;
  int mGetNodeRequest;
  QList<int> mUpdatedNodeIds;
  WbPose *mNodeGetPosition;
  WbPose *mNodeGetOrientation;
  std::pair<WbPose *, WbPose *> mNodeGetPose;
  WbSolid *mNodeGetCenterOfMass;
  WbSolid *mNodeGetContactPoints;
  int mNodeIdGetContactPoints;
  bool mGetContactPointsIncludeDescendants;
  WbSolid *mNodeGetStaticBalance;
  WbSolid *mNodeGetVelocity;
  QString mNodeExportString;
  bool mNodeExportStringRequest;
  bool mIsProtoRegenerated;
  bool mShouldRemoveNode;

  // pointer to a single integer: if not NULL, the new status has to be sent to the libController
  int *mAnimationStartStatus;
  int *mAnimationStopStatus;
  int *mMovieStatus;
  bool *mSaveStatus;

  int mImportedNodeId;
  bool mLoadWorldRequested;
  QString mWorldToLoad;

  bool mVirtualRealityHeadsetIsUsedRequested;
  bool mVirtualRealityHeadsetPositionRequested;
  bool mVirtualRealityHeadsetOrientationRequested;

  QVector<int> mNodesDeletedSinceLastStep;
  QVector<WbUpdatedFieldInfo> mWatchedFields;  // fields used by the libController that need to be updated on change
  QVector<WbUpdatedFieldInfo> mUpdatedFields;  // changed fields that have to be notified to the libController
  QVector<WbFieldSetRequest *> mFieldSetRequests;
  struct WbFieldGetRequest *mFieldGetRequest;

  void pushSingleFieldContentToStream(WbDataStream &stream, WbField *field);
  void pushRelativePoseToStream(WbDataStream &stream, WbPose *fromNode, WbPose *toNode);
  void pushContactPointsToStream(WbDataStream &stream, WbSolid *solid, int solidId, bool includeDescendants);
  void initControllerRequests();
  void deleteControllerRequests();
  void writeNode(WbDataStream &stream, const WbBaseNode *baseNode, int messageType);
  const WbNode *getNodeFromDEF(const QString &defName, bool allowSearchInProto, const WbNode *fromNode = NULL);
  const WbNode *getNodeFromProtoDEF(const WbNode *fromNode, const QString &defName) const;
  WbNode *getProtoParameterNodeInstance(int nodeId, const QString &functionName) const;
  void applyFieldSetRequest(struct field_set_request *request);
  QString readString(QDataStream &);
  void makeFilenameAbsolute(QString &filename);
  WbSimulationState::Mode convertSimulationMode(int supervisorMode);
  QString createLabelUpdateString(const WbWrenLabelOverlay *labelOverlay) const;

  QList<int> mLabelIds;
  QVector<WbTrackedFieldInfo> mTrackedFields;
  QVector<WbTrackedPoseInfo> mTrackedPoses;
  QVector<WbTrackedContactPointInfo> mTrackedContactPoints;
};

#endif
