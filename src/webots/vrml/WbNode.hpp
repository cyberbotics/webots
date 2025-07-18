// Copyright 1996-2024 Cyberbotics Ltd.
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

#ifndef WB_NODE_HPP
#define WB_NODE_HPP

//
// Description: abstract base class for all VRML nodes
//
// Inherited by:
//   WbBaseNode
//

#include <QtCore/QHash>
#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QStringList>

class WbNodeModel;
class WbNodeProtoInfo;
class WbProtoModel;
class WbField;
class WbFieldModel;
class WbTokenizer;
class WbValue;
class WbSFString;
class WbSFInt;
class WbSFDouble;
class WbSFVector2;
class WbSFVector3;
class WbSFColor;
class WbSFNode;
class WbSFBool;
class WbSFRotation;
class WbMFString;
class WbMFInt;
class WbMFDouble;
class WbMFVector2;
class WbMFVector3;
class WbMFColor;
class WbMFBool;
class WbMFRotation;
class WbMFNode;
class WbWriter;

class WbNode : public QObject {
  Q_OBJECT

public:
  // constructor:
  // if the tokenizer is NULL, then the node is constructed with the default field values
  // otherwise the field values are read from the tokenizer
  WbNode(const QString &modelName, const QString &worldPath, WbTokenizer *tokenizer = NULL);

  // create a copy of this node by calling the polymorphic clone function
  // and reference the created instance if the node is PROTO parameter node
  WbNode *cloneAndReferenceProtoInstance();
  // create a copy of this DEF node
  // it doesn't redirect fields even if the current node is a PROTO parameter node
  WbNode *cloneDefNode();

  // destructor
  virtual ~WbNode();

  // set the parent that will be used by the next WbNode constructor
  static void setGlobalParentNode(WbNode *parent, bool protoParameterNodeFlag = false);
  static WbNode *globalParentNode();

  // unique node number
  int uniqueId() const { return mUniqueId; }
  void setUniqueId(int newId);  // works only if newId points on a node which was deleted
  static int getFreeUniqueId();

  // node model
  WbNodeModel *model() const { return mModel; }

  // node info found in the comments (#) of the .wrl file
  // or .proto file if this node is a proto instance
  const QString &info() const;
  QStringList documentationBookAndPage(bool isRobot) const;

  // hierarchy
  void setParentNode(WbNode *node) { mParentNode = node; }
  WbNode *parentNode() const { return mParentNode; }
  bool isTopLevel() const { return parentNode() && parentNode()->isWorldRoot(); }
  bool isWorldRoot() const { return !parentNode() && mUniqueId != -1; }
  bool isProtoRoot() const { return !parentNode() && mUniqueId == -1; }
  bool isAnAncestorOf(const WbNode *node) const;

  // level in the scene tree
  // the level of the root node is 0
  // top level nodes are at level 1, etc.
  int level() const;

  // list of all nodes contained in WbSFNode and WbMFNode fields
  // if 'recurse' is false: look only for direct subnodes
  // if 'recurse' is true: look for all subnodes at any depth
  // if both 'searchInFields' and 'searchInParameters' are false:
  //    search in parameters if proto node and in fields otherwise
  QList<WbNode *> subNodes(bool recurse, bool searchInFields = true, bool searchInParameters = false) const;
  static QList<WbNode *> subNodes(const WbField *field, bool recurse, bool searchInFields = true,
                                  bool searchInParameters = false);

  // names
  const QString &defName() const { return mDefName; }
  void setDefName(const QString &defName, bool recurse = true);
  const QString &useName() const { return mUseName; }
  void setUseName(const QString &useName, bool signal = true);
  QString fullName() const;  // e.g. "Robot, "DEF MY_BOT Robot" or "USE MY_BOT"
  virtual const QString &w3dName() const { return nodeModelName(); }
  virtual const QString urdfName() const;
  const QString &modelName() const;      // e.g. for Nao -> "Nao"
  const QString &nodeModelName() const;  // e.g. for Nao -> "Robot"
  // it's useless to write a empty Group or an invisible Cylinder (with no top, bottom or side)
  virtual bool shallExport() const { return true; };

  // error reporting
  // field name will be extracted from the first single quoted text in the message
  void parsingWarn(const QString &message) const;  // show parsing warning message formatted for this node
  void parsingInfo(const QString &message) const;  // show parsing info message formatted for this node
  void warn(const QString &message,
            bool parsingMessage = false) const;  // show standard warning message formatted for this node
  void info(const QString &message, bool parsingMessage = false) const;  // show standard info message formatted for this node
  QString usefulName() const;                                            // user friendy node name for error messages
  virtual QString endPointName() const { return QString(); };
  QString computeName() const;  // USE, DEF or "name" field value

  // destruction
  bool isBeingDeleted() const { return mIsBeingDeleted; }

  // DEF-USE mechanism

  // Sets USE name and subscribes to DEF node notifications if reading = false
  void makeUseNode(WbNode *defNode, bool reading = false);
  void makeDefNode();  // Turns an orphan USE node into a DEF node with the same name
  void setDefNode(WbNode *defNode) { mDefNode = defNode; }
  WbNode *defNode() const { return mDefNode; }
  bool isUseNode() const { return !mUseName.isEmpty(); }
  bool isDefNode() const { return !mDefName.isEmpty(); }
  int useCount() const { return mUseNodes.size(); }
  const QList<WbNode *> &useNodes() const { return mUseNodes; }
  virtual void defHasChanged() {}
  static void setDictionaryUpdateFlag(bool b) { cUpdatingDictionary = b; }

  // index functions
  static int subNodeIndex(const WbNode *subNode, const WbNode *root);
  static WbNode *findNodeFromSubNodeIndex(int index, WbNode *root);
  // find descendant node from a list of parent indices
  // indices are listed from the ancestor parent node (position 0) to the searched node index (position indices.size()-1)
  static WbNode *findNodeFromSubNodeIndices(const QList<int> &indices, WbNode *root);

  // PROTO
  static WbNode *createProtoInstance(WbProtoModel *proto, WbTokenizer *tokenizer, const QString &worldPath);
  static WbNode *createProtoInstanceFromParameters(WbProtoModel *proto, const QList<WbField *> &parameters,
                                                   const QString &worldPath, int uniqueId = -1);

  bool isProtoInstance() const { return mProto != NULL; }
  WbProtoModel *proto() const { return mProto; }
  const QList<WbNodeProtoInfo *> &protoParents() const { return mProtoParents; }
  bool isTemplate() const;
  void setRegenerationRequired(bool required);
  bool isRegenerationRequired() const { return mRegenerationRequired; }
  const QByteArray &protoInstanceTemplateContent() const { return mProtoInstanceTemplateContent; }
  const QList<WbField *> &parameters() const { return mParameters; }
  void setProtoInstanceTemplateContent(const QByteArray &content) { mProtoInstanceTemplateContent = content; }
  // pass argument if we know that a PROTO ancestor exists, otherwise if hasAProtoAncestorFlag is FALSE it will be computed
  void updateNestedProtoFlag(bool hasAProtoAncestorFlag = false);

  // return if 'node' is a direct child of this PROTO parameters
  bool isProtoParameterChild(const WbNode *node) const;
  // is a parameter node contained in a PROTO instance
  bool isProtoParameterNode() const;
  // return the node instances redirected to this PROTO parameter node
  const QList<WbNode *> &protoParameterNodeInstances() const { return mProtoParameterNodeInstances; }
  bool hasAProtoAncestor() const;
  WbNode *protoAncestor() const;
  // connect nested PROTO parameters
  void redirectInternalFields(WbField *param = NULL, bool finalize = false);

  const WbNode *containingProto(bool skipThis) const;

  // return the node contained in a PROTO parameter that represents the current instance in the scene tree
  WbNode *protoParameterNode() const { return mProtoParameterNode; }

  // list all the texture files used and the corresponding field
  QList<std::pair<QString, WbMFString *>> listTextureFiles() const;

  // write node and fields as text
  virtual void write(WbWriter &writer) const;
  static void enableDefNodeTrackInWrite(bool substituteInStream);
  static void disableDefNodeTrackInWrite();
  static QList<std::pair<WbNode *, int>> *externalUseNodesPositionsInWrite();

  // fields or proto parameters
  bool isDefault() const;  // true if all fields have default values
  const QList<WbField *> &fields() const { return mFields; }
  const QList<WbField *> &fieldsOrParameters() const { return isProtoInstance() ? mParameters : mFields; }
  int numFields() const { return fieldsOrParameters().size(); }
  WbField *field(int index, bool internal = false) const;
  WbField *findField(const QString &fieldName, bool internal = false) const;
  WbField *parentField(bool internal = false) const {
    int index = -1;
    return parentFieldAndIndex(index, internal);
  }
  WbField *parentFieldAndIndex(int &index, bool internal = false) const;
  int findFieldId(const QString &fieldName, bool internal = false) const;
  int fieldIndex(const WbField *field) const;
  int parameterIndex(const WbField *field) const;
  void disconnectFieldNotification(const WbValue *value);
  // set parent node of fields for this node and its subnodes
  void setFieldsParentNode();

  // to find field values in init() functions
  WbSFString *findSFString(const QString &fieldName) const;
  WbSFInt *findSFInt(const QString &fieldName) const;
  WbSFDouble *findSFDouble(const QString &fieldName) const;
  WbSFVector2 *findSFVector2(const QString &fieldName) const;
  WbSFVector3 *findSFVector3(const QString &fieldName) const;
  WbSFColor *findSFColor(const QString &fieldName) const;
  WbSFNode *findSFNode(const QString &fieldName) const;
  WbSFBool *findSFBool(const QString &fieldName) const;
  WbSFRotation *findSFRotation(const QString &fieldName) const;
  WbMFString *findMFString(const QString &fieldName) const;
  WbMFInt *findMFInt(const QString &fieldName) const;
  WbMFDouble *findMFDouble(const QString &fieldName) const;
  WbMFVector2 *findMFVector2(const QString &fieldName) const;
  WbMFVector3 *findMFVector3(const QString &fieldName) const;
  WbMFColor *findMFColor(const QString &fieldName) const;
  WbMFBool *findMFBool(const QString &fieldName) const;
  WbMFRotation *findMFRotation(const QString &fieldName) const;
  WbMFNode *findMFNode(const QString &fieldName) const;

  // static functions
  static void setInstantiateMode(bool mode);
  static bool instantiateMode();
  static void cleanup();
  static WbNode *findNode(int uniqueId);
  static bool setRestoreUniqueIdOnClone(bool enable);

  // compare two nodes, returns true if they have the same type and all fields have the same values
  bool operator==(const WbNode &other) const;
  bool operator!=(const WbNode &other) const { return !(*this == other); }

  // regenerating this node. The pointer to this node may differ after calling this function
  void regenerateNode() { emit regenerateNodeRequest(this, false); }

  // STRUCTURE_USE represent all the nodes that are not used in a boundingObject field.
  enum NodeUse { UNKNOWN_USE = 0, STRUCTURE_USE = 1, BOUNDING_OBJECT_USE = 2, BOTH_USE = 3 };
  // validate node descendants and remove the invalid ones.
  // If upperNode and upperField are specified, the structure is validated assuming
  // that the parent node is the specified one instead of the current one
  // Currently upperNode and upperField are only used for skipping Slot nodes.
  void validate(const WbNode *upperNode = NULL, const WbField *upperField = NULL, bool isInBoundingObject = false) const;

  void setCreationCompleted();
  void setInsertionCompleted() { mInsertionCompleted = true; }

  // export
  virtual void exportBoundingObjectToW3d(WbWriter &writer) const {}
  virtual QStringList fieldsToSynchronizeWithW3d() const { return QStringList(); }
  virtual void fixMissingResources() const {}

  virtual void reset(const QString &id);
  virtual void save(const QString &id) {}
  virtual const QString &stateId() const { return mCurrentStateId; };

  // debug utility functions
  // void printDebugNodeStructure(int level = 0);
  // void printDebugNodeFields(int level, bool printParameters);
  virtual const bool isRobot() const { return false; };

signals:
  void isBeingDestroyed(WbNode *node);

  // emitted when any value has changed
  void fieldChanged(WbField *field);
  void parameterChanged(WbField *field);

  // emitted when the DEF or USE name has changed
  void defUseNameChanged(WbNode *node, bool fromUseToDef);

  // ask for regenerating this node
  void regenerateNodeRequest(WbNode *node, bool nested);

  // notify this node need to be regenerated
  void regenerationRequired();

protected:
  // copy constructor to be invoked from the copy constructors of derived classes
  // copies all the field values
  WbNode(const WbNode &other);

  // constructor for shallow nodes, should be used exclusively by the CadShape node
  explicit WbNode(const QString &modelName);
  bool mIsShallowNode;

  // DEF-USE dictionary
  static bool cUpdatingDictionary;  // This flag orders to skip any DEF->USEs update when updating the dictionary

  virtual void writeExport(WbWriter &writer) const;
  virtual void writeParameters(WbWriter &writer) const;
  virtual void readHiddenKinematicParameter(WbField *field) {}

  virtual bool exportNodeHeader(WbWriter &writer) const;
  virtual void exportNodeContents(WbWriter &writer) const;
  virtual void exportNodeFields(WbWriter &writer) const;
  virtual void exportNodeSubNodes(WbWriter &writer) const;
  virtual void exportNodeFooter(WbWriter &writer) const;
  virtual void exportExternalSubProto(WbWriter &writer) const;

  // fields which should not be handled by the default exportNodeFields function
  virtual QStringList customExportedFields() const { return QStringList(); }

  // Helper to handle exporting fields with resources which reference external files
  QString exportResource(const QString &rawURL, const QString &resolvedURL, const QString &relativeResourcePath,
                         const WbWriter &writer) const;
  // Wrappers for the most common use case (simply exporting a field with no additional processing)
  void exportMFResourceField(const QString &fieldName, const WbMFString *value, const QString &relativeResourcePath,
                             WbWriter &writer) const;
  void exportSFResourceField(const QString &fieldName, const WbSFString *value, const QString &relativeResourcePath,
                             WbWriter &writer) const;

  // Methods related to URDF export
  const WbNode *findUrdfLinkRoot() const;  // Finds first upper Webots node that is considered as URDF link
  virtual bool isUrdfRootLink() const;     // Determines whether the Webots node is considered as URDF link as well
  virtual void exportUrdfJoint(WbWriter &writer) const {};

  virtual void useNodesChanged() const {};
  bool isNestedProtoNode() const { return mIsNestedProtoNode; }

  QString getUrdfPrefix() const;
  void setUrdfPrefix(const QString &prefix) { mUrdfPrefix = prefix; };
  virtual const bool isJoint() const { return false; };

  static void redirectInternalFields(WbNode *instance, WbNode *parameterNode, bool finalize = false);
  void finalizeProtoParametersRedirection();

private slots:
  void notifyFieldChanged();
  void notifyParameterChanged();
  void removeProtoParameterNodeInstance(QObject *node);

private:
  WbNode &operator=(const WbNode &);  // non copyable

  QString mUrdfPrefix;
  QString mCurrentStateId;

  // for all nodes
  WbNode *mParentNode;
  WbNodeModel *mModel;
  int mUniqueId;
  bool mIsBeingDeleted;
  bool mIsCreationCompleted;
  bool mInsertionCompleted;  // true if the current node is already added to the parent field
  QList<WbField *> mFields;

  // DEF/USE mechanism
  QString mDefName;  // for DEF nodes
  QList<WbNode *> mUseNodes;
  QString mUseName;  // for USE nodes
  WbNode *mDefNode;
  bool mHasUseAncestor;

  // for proto instances only
  WbProtoModel *mProto;
  QList<WbNodeProtoInfo *> mProtoParents;
  QList<WbField *> mParameters;
  QList<WbField *> mInternalProtoParameters;
  QByteArray mProtoInstanceTemplateContent;
  bool mRegenerationRequired;

  // for proto parameter nodes only
  bool mIsNestedProtoNode;
  mutable bool mIsProtoParameterNodeDescendant;
  QList<WbNode *> mProtoParameterNodeInstances;
  static const QStringList cHiddenParameterNames;
  // if this node has is a direct PROTO parameter node, store the parent PROTO node
  // otherwise set mProtoParameterParentNode to this.
  // mProtoParameterParentNode is NULL before the value it is computed.
  mutable const WbNode *mProtoParameterParentNode;
  mutable bool *mIsProtoParameterNode;

  // if PROTO instance is created as a consequence of an explicit insertion from the scene tree or due to a template
  // regeneration, then setup of instance's nested PROTO flag should be postponed to correctly redirect all the parameters
  static WbNode *createProtoInstanceFromParameters(WbProtoModel *proto, const QList<WbField *> &parameters, bool isTopLevel,
                                                   const QString &worldPath, bool fromSceneTree = false, int uniqueId = -1);

  // for proto parameter node instances only
  WbNode *mProtoParameterNode;
  bool mIsRedirectedToParameterNode;

  // create a copy of this node by copying all its field values (polymorphic constructor)
  virtual WbNode *clone() const;

  // full path from root node, separated with " > "
  // if possible displays the visible structure of PROTO nodes and not the internal one
  QString fullPath(const QString &fieldName, QString &parameterName) const;
  // extract first single quoted text from message
  static QString extractFieldName(const QString &message);

  WbValue *findValue(const QString &fieldName) const;
  void readFields(WbTokenizer *tokenizer, const QString &worldPath);
  void addField(WbFieldModel *fieldModel);
  void init();
  void redirectAliasedFields(WbField *param, WbNode *protoInstance, bool searchInParameters = false,
                             bool copyValueOnly = false);
  void resetUseAncestorFlag();
  int findSubFieldIndex(const WbField *const searched) const;
  static void subNodeIndex(const WbNode *currentNode, const WbNode *targetNode, int &index, bool &subNodeFound);
  static WbNode *findNode(int &index, WbNode *root);
  WbField *findSubField(int index, WbNode *&parent) const;
  void readFieldValue(WbField *field, WbTokenizer *tokenizer, const QString &worldPath) const;
  static void copyAliasValue(WbField *field, const QString &alias);
  void addExternProtoFromFile(const WbProtoModel *proto, WbWriter &writer) const;

  bool setProtoParameterNode(WbNode *node);
  static void redirectInternalFields(WbField *field, WbField *parameter, bool finalize = false);
};

#endif
