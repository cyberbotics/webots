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

#ifndef WB_PROTO_MANAGER_HPP
#define WB_PROTO_MANAGER_HPP

//
// Description: a class for managing a list of proto models
//

class WbNode;
class WbProtoModel;
class WbTokenizer;
class WbDownloader;
class WbProtoTreeItem;
class WbVersion;

#include <QtCore/QDateTime>
#include <QtCore/QFileInfoList>
#include <QtCore/QMap>
#include <QtCore/QObject>
#include <QtCore/QRegularExpression>
#include <QtCore/QStringList>

class WbExternProto {
public:
  WbExternProto(const QString &name, const QString &url, bool isImportable) :
    mName(name),
    mUrl(url),
    mImportable(isImportable) {}

  const QString &name() const { return mName; }
  void setUrl(const QString &url) { mUrl = url; }
  const QString &url() const { return mUrl; }
  bool isImportable() const { return mImportable; }
  void setImportable(bool value) { mImportable = value; }

private:
  QString mName;
  QString mUrl;
  bool mImportable;
};

class WbProtoInfo {
public:
  WbProtoInfo(const QString &url, const QString &baseType, const QString &license, const QString &licenseUrl,
              const QString &documentationUrl, const QString &description, const QString &slotType, const QStringList &tags,
              const QStringList &parameters, bool needsRobotAncestor) :
    mUrl(url),
    mBaseType(baseType),
    mLicense(license),
    mLicenseUrl(licenseUrl),
    mDocumentationUrl(documentationUrl),
    mDescription(description),
    mSlotType(slotType),
    mTags(tags),
    mParameters(parameters),
    mNeedsRobotAncestor(needsRobotAncestor),
    mIsDirty(false) {
    // extract parameter names
    foreach (const QString &parameter, mParameters) {
      QRegularExpression re("(?:field|vrmlField)\\s+(?:\\w+|(?:\\{[\\s\\S]*\\}))+\\s+(\\w+)\\s");
      QRegularExpressionMatch match = re.match(parameter);
      if (match.hasMatch())
        mParameterNames << match.captured(1);
    }
    // sanity check, if they differ the script generating proto-list.xml or its parsing on the webots side likely went wrong
    assert(mParameters.size() == mParameterNames.size());
  }
  // copy constructor
  WbProtoInfo(const WbProtoInfo &other) :
    WbProtoInfo(other.mUrl, other.mBaseType, other.mLicense, other.mLicenseUrl, other.mDocumentationUrl, other.mDescription,
                other.mSlotType, other.mTags, other.mParameters, other.mNeedsRobotAncestor) {}

  const QString &url() const { return mUrl; }
  const QString &baseType() const { return mBaseType; }
  const QString &license() const { return mLicense; }
  const QString &licenseUrl() const { return mLicenseUrl; }
  const QString &documentationUrl() const { return mDocumentationUrl; }
  const QString &description() const { return mDescription; }
  const QString &slotType() const { return mSlotType; }
  const QStringList &tags() const { return mTags; }
  const QStringList &parameters() const { return mParameters; }
  const QStringList &parameterNames() const { return mParameterNames; }
  const bool needsRobotAncestor() const { return mNeedsRobotAncestor; }
  void setDirty(bool value) { mIsDirty = value; }
  bool isDirty() const { return mIsDirty; }

private:
  QString mUrl;
  QString mBaseType;
  QString mLicense;
  QString mLicenseUrl;
  QString mDocumentationUrl;
  QString mDescription;
  QString mSlotType;
  QStringList mTags;
  QStringList mParameters;
  bool mNeedsRobotAncestor;
  bool mIsDirty;

  QStringList mParameterNames;
};

class WbProtoManager : public QObject {
  Q_OBJECT
public:
  static WbProtoManager *instance();

  // used to reference the different PROTO lists and by the dialog windows (WbAddNodeDialog, WbNewProtoWizard,
  // WbInsertExternProtoDialog) to categorize their respective PROTO
  enum { BASE_NODE = 10001, PROTO_WORLD = 10002, PROTO_PROJECT = 10003, PROTO_EXTRA = 10004, PROTO_WEBOTS = 10005 };

  void setImportedFromSupervisor(bool value) { mImportedFromSupervisor = value; };

  // given a category and a PROTO name, it returns the URL from the corresponding list
  QString protoUrl(const QString &protoName, int category) const;
  // tests if the PROTO of the provided name exists in the specified category
  bool isProtoInCategory(const QString &protoName, int category) const;

  const QList<WbProtoModel *> &models() const { return mModels; }

  // searches for proto model according to:
  // 1. First in the session list (i.e., PROTO discovered by navigating the PROTO tree stemming from a world file
  // 2. If the first fails, it searches among the known official Webots PROTO (proto-list.xml)
  WbProtoModel *findModel(const QString &modelName, const QString &worldPath, const QString &parentFilePath,
                          const QStringList &baseTypeList = QStringList());

  WbProtoModel *readModel(const QString &url, const QString &worldPath, const QString &prefix = QString(),
                          const QStringList &baseTypeList = QStringList()) const;

  // PROTO retriever for world files
  void retrieveExternProto(const QString &filename, bool reloading);

  // PROTO retriever for inserted PROTO
  void retrieveExternProto(const QString &filename);

  // retrieves all PROTO dependencies referenced by the PROTO in the current and extra project directories
  void retrieveLocalProtoDependencies();

  // used primarily when populating the dialog windows
  QMap<QString, WbProtoInfo *> webotsProtoList() { return mWebotsProtoList; };

  // generates meta info from a PROTO file (license, tags, ...)
  WbProtoInfo *generateInfoFromProtoFile(const QString &protoFileName);

  // generates the PROTO info (metadata) for a specific category
  void generateProtoInfoMap(int category, bool regenerate = true);

  // returns a reference to the maps generated by generateProtoInfoMap
  const QMap<QString, WbProtoInfo *> &protoInfoMap(int category) const;
  const WbProtoInfo *protoInfo(const QString &protoName, int category);

  // lists the existing PROTO in the primary project locations
  QStringList listProtoInCategory(int category) const;

  // list of all EXTERNPROTO (both importable and not), stored in a QVector as order matters when saving to file
  const QVector<WbExternProto *> &externProto() const { return mExternProto; };

  // EXTERNPROTO manipulators
  // declares EXTERNPROTO and returns the previous URL if is another PROTO with the same model if already declared
  QString declareExternProto(const QString &protoName, const QString &protoPath, bool importable, bool forceUpdate = true);
  void purgeUnusedExternProtoDeclarations(const QSet<QString> &protoNamesInUse);
  QString externProtoUrl(const WbNode *node, bool formatted = false) const;
  QString removeProtoUrl(const WbNode *node, bool formatted = false) const;

  QString findExternProtoDeclarationInFile(const QString &url, const QString &modelName);
  void removeImportableExternProto(const QString &protoName, WbNode *root);

  bool isImportableExternProtoDeclared(const QString &protoName);

  // EXTERNPROTO stored after copying or cutting an inserted node
  const QList<WbExternProto *> &externProtoClipboardBuffer() const { return mExternProtoClipboardBuffer; };
  void saveToExternProtoClipboardBuffer(const QList<const WbNode *> &nodes);
  void saveToExternProtoClipboardBuffer(const QString &url);
  void clearExternProtoClipboardBuffer();
  // save currnent clipboard buffer for restoring it later
  QList<QString> externProtoClipboardBufferUrls() const;
  void resetExternProtoClipboardBuffer(const QList<QString> &bufferUrls);

  void updateExternProto(const QString &protoName, const QString &url);
  QString formatExternProtoPath(const QString &url) const;

  void updateCurrentWorld(const QString &world) { mCurrentWorld = world; }

  void setNeedsRobotAncestorCallback(std::function<bool(const QString &)> f) { mNeedsRobotAncestorCallback = f; }

signals:
  void worldLoadCompleted(const QString &worldName, bool reloading, bool isLoadingAfterDownload);
  void retrievalCompleted();
  void dependenciesAvailable();

private slots:
  void loadWorld();
  void protoRetrievalCompleted();

private:
  WbProtoManager();
  ~WbProtoManager();

  // cppcheck-suppress unknownMacro
  Q_DISABLE_COPY(WbProtoManager)

  QList<WbProtoModel *> mModels;

  QString mRetrievalError;
  QString mCurrentWorld;
  bool mReloading;

  bool mImportedFromSupervisor;

  WbProtoTreeItem *mTreeRoot;

  // mSessionProto: list of all the PROTO discovered in the session, it may contain:
  // - PROTO directly referenced in the world file (as EXTERNPROTO) and all the indirect sub-PROTO they themselves reference
  // - PROTO directly inserted from the add-node dialog and all the indirect sub-PROTO they depend on
  // - PROTO declared by the user as EXTERNPROTO through the GUI and all the indirect sub-PROTO they depend on
  // note: this list is reset before every world load (since the urls are not guaranteed to be the same between worlds)
  QStringList mSessionProto;
  QStringList mPreviousSessionProto;  // copy of the previous session, used to purge models that are no longer needed
  // mExternProto: ordered list of PROTO that will be saved to the world file, it may contain:
  // - list of EXTERNPROTO loaded from the world file (unless it has been actively removed by the user through the GUI)
  // - list of PROTO declared by the user through the GUI (which may not be actually used in the world file)
  // note: the list may reference unused PROTO since they might be loaded by a controller on-the-fly instead
  // note: this list is reset before every world load and each time a node is deleted
  QVector<WbExternProto *> mExternProto;

  // mExternProtoClipboardBuffer: contains the externProto reference of the last copies or cut instance
  QList<WbExternProto *> mExternProtoClipboardBuffer;

  // stores PROTO metadata
  QMap<QString, WbProtoInfo *> mWebotsProtoList;     // loaded from proto-list.xml
  QMap<QString, WbProtoInfo *> mWorldFileProtoList;  // compiled from EXTERNPROTO referenced in .wbt
  QMap<QString, WbProtoInfo *> mProjectProtoList;    // compiled from PROTO in current project directory
  QMap<QString, WbProtoInfo *> mExtraProtoList;      // compiled from PROTO in extra project directories

  QMap<int, QDateTime> mProtoInfoGenerationTime;

  QStringList mUniqueErrorMessages;

  std::function<bool(const QString &)> mNeedsRobotAncestorCallback;

  void loadWebotsProtoMap();

  QString injectDeclarationByBackwardsCompatibility(const QString &modelName);

  QMap<QString, QString> undeclaredProtoNodes(const QString &filename);

  void displayMissingDeclarations(const QString &message);

  WbVersion checkProtoVersion(const QString &protoUrl, bool *foundProtoVersion);
  void cleanup();
};

#endif
