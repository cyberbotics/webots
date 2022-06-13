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

#ifndef WB_PROTO_MANAGER_HPP
#define WB_PROTO_MANAGER_HPP

//
// Description: a class for managing a list of proto models
//

class WbProtoModel;
class WbTokenizer;
class WbDownloader;
class WbProtoTreeItem;

#include <QtCore/QDateTime>
#include <QtCore/QFileInfoList>
#include <QtCore/QMap>
#include <QtCore/QObject>
#include <QtCore/QRegularExpression>
#include <QtCore/QStringList>

class WbExternProtoInfo {
public:
  WbExternProtoInfo(QString name, QString url, bool ephemeral = false) : mName(name), mUrl(url), mEphemeral(ephemeral) {}

  const QString &name() const { return mName; }
  const QString &url() const { return mUrl; }
  void ephemeral(bool value) { mEphemeral = value; }
  bool isEphemeral() const { return mEphemeral; }

private:
  QString mName;
  QString mUrl;
  bool mEphemeral;
};

class WbProtoInfo {
public:
  WbProtoInfo(QString url, QString baseType, QString license, QString licenseUrl, QString documentationUrl, QString description,
              QString slotType, QStringList tags, QStringList parameters, bool needsRobotAncestor) :
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
    foreach (const QString parameter, mParameters) {
      QRegularExpression re("(?:field|vrmlField)\\s+\\w+\\s+(\\w+)\\s");
      QRegularExpressionMatch match = re.match(parameter);
      if (match.hasMatch())
        mParameterNames << match.captured(1);
    }
  }

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
  void dirty(bool value) { mIsDirty = value; }
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

  WbProtoManager();
  ~WbProtoManager();

  // used to reference the different PROTO lists and by the dialog windows (WbAddNodeDialog, WbNewProtoWizard,
  // WbInsertExternProtoDialog) to categorize their respective PROTO
  enum { BASE_NODE = 10001, PROTO_WORLD = 10002, PROTO_PROJECT = 10003, PROTO_EXTRA = 10004, PROTO_WEBOTS = 10005 };

  // given a category and a PROTO name, it returns the url from the corresponding list
  QString protoUrl(int category, const QString &protoName) const;
  // tests if the PROTO is among the known official Webots PROTO (proto-list.xml)
  bool isWebotsProto(const QString &protoName) const;

  const QList<WbProtoModel *> &models() const { return mModels; }

  // search for proto model
  // the search is done in 2 steps:
  //  1. The current project's primary search path
  //  2. The system resources
  // if no matching model is found, an empty string is returned
  QString findModelPath(const QString &modelName) const;

  // searches for proto model according to:
  // 1. First in the session list (i.e., PROTO discovered by navigating the PROTO tree stemming from a world file
  // 2. If the first fails, it searches among the known official Webots PROTO (proto-list.xml)
  WbProtoModel *findModel(const QString &modelName, const QString &worldPath, QStringList baseTypeList = QStringList());

  WbProtoModel *readModel(const QString &fileName, const QString &worldPath, const QString &externUrl = QString(),
                          QStringList baseTypeList = QStringList()) const;

  // read a proto model and place it in this list
  void readModel(WbTokenizer *tokenizer, const QString &worldPath);

  // PROTO retriever for world files
  void retrieveExternProto(const QString &filename, bool reloading, const QStringList &unreferencedProtos);

  // PROTO retriever for inserted PROTO
  void retrieveExternProto(const QString &filename);

  // used primarily when populating the dialog windows
  QMap<QString, WbProtoInfo *> webotsProtoList() { return mWebotsProtoList; };

  // generates meta info from a PROTO file (license, tags, ...)
  WbProtoInfo *generateInfoFromProtoFile(const QString &protoFileName);

  // generates the PROTO info (metadata) for a specific category
  void generateProtoInfoMap(int category, bool regenerate);

  // returns a reference to the maps generated by generateProtoInfoMap
  const QMap<QString, WbProtoInfo *> &protoInfoMap(int category) const;
  const WbProtoInfo *protoInfo(int category, const QString &protoName);

  // lists the existing PROTO in the primary project locations
  QStringList listProtoInDirectory(int category) const;

  // exports a copy of a selected PROTO to the current project directory
  void exportProto(const QString &path, int category);

  // list of all EXTERNPROTO (both ephemeral and not), stored in a QVector as order matters when saving to file
  const QVector<WbExternProtoInfo *> &externProto() const { return mExternProto; };

  // EXTERNPROTO manipulators
  void declareExternProto(const QString &protoName, const QString &protoPath, bool ephemeral);
  void removeExternProto(const QString &protoName);
  void refreshExternProtoList();
  bool isDeclaredExternProto(const QString &protoName);

signals:
  void retrievalCompleted();

private slots:
  void loadWorld();
  void singleProtoRetrievalCompleted();

private:
  // cppcheck-suppress unknownMacro
  Q_DISABLE_COPY(WbProtoManager)

  QList<WbProtoModel *> mModels;

  QString mRetrievalError;
  QString mCurrentWorld;
  bool mReloading;

  WbProtoTreeItem *mTreeRoot;

  // mSessionProto: un-ordered map (PROTO name <-> disk location) of all the PROTO discovered in the session, it may contain:
  // - PROTO directly referenced in the world file (as EXTERNPROTO) and all the indirect sub-PROTO they themselves reference
  // - PROTO directly inserted from the add-node dialog and all the indirect sub-PROTO they depend on
  // - PROTO declared by the user as EXTERNPROTO through the GUI and all the indirect sub-PROTO they depend on
  // note: this list is reset before every world load (since the urls are not guaranteed to be the same between worlds)
  QMap<QString, QString> mSessionProto;
  // mExternProto: ordered list of PROTO that will be saved to the world file, it may contain:
  // - list of EXTERNPROTO loaded from the world file (unless it has been actively removed by the user through the GUI)
  // - list of PROTO declared by the user through the GUI (which may not be actually used in the world file)
  // note: the list may reference unused PROTO since they might be loaded by a controller on-the-fly instead
  // note: this list is reset before every world load
  QVector<WbExternProtoInfo *> mExternProto;

  // stores PROTO metadata
  QMap<QString, WbProtoInfo *> mWebotsProtoList;     // loaded from proto-list.xml
  QMap<QString, WbProtoInfo *> mWorldFileProtoList;  // compiled from EXTERNPROTO referenced in .wbt
  QMap<QString, WbProtoInfo *> mProjectProtoList;    // compiled from PROTO in current project directory
  QMap<QString, WbProtoInfo *> mExtraProtoList;      // compiled from PROTO in extra project directories

  QMap<int, QDateTime> mProtoInfoGenerationTime;

  void loadWebotsProtoMap();

  void cleanup();
};

#endif
