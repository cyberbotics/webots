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

#ifndef WB_SUPERVISOR_UTILITIES_HPP
#define WB_SUPERVISOR_UTILITIES_HPP

#include "WbSimulationState.hpp"

#include <QtCore/QObject>
#include <QtCore/QVector>

class QDataStream;

struct WbDeletedNodeInfo;
struct WbFieldGetRequest;
class WbFieldSetRequest;

class WbBaseNode;
class WbNode;
class WbRobot;
class WbTransform;
class WbSolid;
class WbWrenLabelOverlay;

class WbSupervisorUtilities : public QObject {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbSupervisorUtilities(WbRobot *robot);
  virtual ~WbSupervisorUtilities();

  void handleMessage(QDataStream &stream);
  void writeAnswer(QDataStream &stream);
  void writeConfigure(QDataStream &stream);
  void processImmediateMessages();
  void postPhysicsStep();
  void reset();

  QStringList labelsState() const;

signals:
  void worldModified();
  void changeSimulationModeRequested(int newMode);
  void labelChanged(const QString &labelDescription);  // i.e. "label:<id>;<font>;<color>;<size>;<x>;<y>;<text>"

private slots:
  void animationStartStatusChanged(int status);
  void animationStopStatusChanged(int status);
  void movieStatusChanged(int status);
  void changeSimulationMode(int newMode);
  void updateDeletedNodeList(WbNode *node);
  void notifyNodeUpdate(WbNode *node);

private:
  WbRobot *mRobot;
  int mFoundNodeUniqueId;
  int mFoundNodeType;
  QString mFoundNodeModelName;
  QString mCurrentDefName;
  int mFoundNodeParentUniqueId;
  int mFoundFieldId;
  int mFoundFieldType;
  int mFoundFieldCount;
  bool mGetSelectedNode;
  bool mGetFromId;
  QList<int> mUpdatedNodeIds;
  bool mNeedToRestartController;
  WbTransform *mNodeGetPosition;
  WbTransform *mNodeGetOrientation;
  WbSolid *mNodeGetCenterOfMass;
  WbSolid *mNodeGetContactPoints;
  WbSolid *mNodeGetStaticBalance;
  WbSolid *mNodeGetVelocity;

  // pointer to a single integer: if not NULL, the new status has to be sent to the libController
  int *mAnimationStartStatus;
  int *mAnimationStopStatus;
  int *mMovieStatus;
  bool *mSaveStatus;

  int mImportedNodesNumber;
  bool mLoadWorldRequested;
  QString mWorldToLoad;

  bool mVirtualRealityHeadsetIsUsedRequested;
  bool mVirtualRealityHeadsetPositionRequested;
  bool mVirtualRealityHeadsetOrientationRequested;

  QVector<struct WbDeletedNodeInfo> mNodesDeletedSinceLastStep;
  QVector<WbFieldSetRequest *> mFieldSetRequests;
  struct WbFieldGetRequest *mFieldGetRequest;

  void initControllerRequests();
  void deleteControllerRequests();
  void writeNode(QDataStream &stream, const WbBaseNode *baseNode, int messageType);
  const WbNode *getNodeFromDEF(const QString &defName, const WbNode *fromNode = NULL);
  WbNode *getProtoParameterNodeInstance(WbNode *const node) const;
  void applyFieldSetRequest(struct field_set_request *request);
  QString readString(QDataStream &);
  void makeFilenameAbsolute(QString &filename);
  WbSimulationState::Mode convertSimulationMode(int supervisorMode);
  QString createLabelUpdateString(const WbWrenLabelOverlay *labelOverlay) const;

  QList<int> mLabelIds;
};

#endif
