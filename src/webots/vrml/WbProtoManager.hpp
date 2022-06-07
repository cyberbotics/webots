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

#include <QtCore/QFileInfoList>
#include <QtCore/QMap>
#include <QtCore/QObject>
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
              QString slotType, QStringList tags, bool needsRobotAncestor) :
    mUrl(url),
    mBaseType(baseType),
    mLicense(license),
    mLicenseUrl(licenseUrl),
    mDocumentationUrl(documentationUrl),
    mDescription(description),
    mSlotType(slotType),
    mTags(tags),
    mNeedsRobotAncestor(needsRobotAncestor){};

  const QString &url() const { return mUrl; }
  const QString &baseType() const { return mBaseType; }
  const QString &license() const { return mLicense; }
  const QString &licenseUrl() const { return mLicenseUrl; }
  const QString &documentationUrl() const { return mDocumentationUrl; }
  const QString &description() const { return mDescription; }
  const QString &slotType() const { return mSlotType; }
  const QStringList &tags() const { return mTags; }
  const bool needsRobotAncestor() const { return mNeedsRobotAncestor; }

private:
  QString mUrl;
  QString mBaseType;
  QString mLicense;
  QString mLicenseUrl;
  QString mDocumentationUrl;
  QString mDescription;
  QString mSlotType;
  QStringList mTags;
  bool mNeedsRobotAncestor;
};

class WbProtoManager : public QObject {
  Q_OBJECT
public:
  static WbProtoManager *instance();

  // return all proto files stored in valid project folders located in the given path
  static void findProtosRecursively(const QString &dirPath, QFileInfoList &protoList, bool inProtos = false);

  enum { PROTO_WORLD, PROTO_PROJECT, PROTO_EXTRA, PROTO_WEBOTS };
  QStringList nameList(int category);

  // create a proto list with a .proto file search path
  // the path will be searched recursively
  // explicit WbProtoManager(const QString &primarySearchPath = "");
  // explicit WbProtoManager(const QString &world, bool reloading = false);
  WbProtoManager();

  const QString getExtraProtoUrl(const QString &protoName);
  const QString getProjectProtoUrl(const QString &protoName);
  const QString getWebotsProtoUrl(const QString &protoName);
  bool isWebotsProto(const QString &protoName);

  // bool backwardsCompatibilityProtoRetrieval(const QStringList &protoList, const QString &filename, bool reloading);

  // destroys the list and all the contained models
  ~WbProtoManager();

  const QList<WbProtoModel *> &models() const { return mModels; }

  // search for proto model
  // the search is done in 2 steps:
  //  1. The current project's primary search path
  //  2. The system resources
  // if no matching model is found, an empty string is returned
  QString findModelPath(const QString &modelName) const;

  // search for proto model
  // the search is done in 3 steps:
  //  1. The current list of loaded proto models
  //  2. The current project's primary search path
  //  3. The system resources
  // if no matching model is found, NULL is returned and the error is notified on WbLog
  // WbProtoModel *findModel(const QString &modelName, const QString &worldPath, QStringList baseTypeList = QStringList());
  WbProtoModel *customFindModel(const QString &modelName, const QString &worldPath, QStringList baseTypeList = QStringList());

  WbProtoModel *readModel(const QString &fileName, const QString &worldPath, const QString &externUrl = QString(),
                          QStringList baseTypeList = QStringList()) const;

  // read a proto model and place it in this list
  // prerequisite: the next token must be the "PROTO" keyword in the tokenizer
  // prerequisite: the syntax must have been checked with WbParser
  void readModel(WbTokenizer *tokenizer, const QString &worldPath);

  void printCurrentWorldProtoList();  // TODO: remove

  void retrieveExternProto(const QString &filename, bool reloading, const QStringList &unreferencedProtos);
  void retrieveExternProto(const QString &filename);

  // used primarely when populating the add-node dialog window
  QMap<QString, WbProtoInfo *> webotsProtoList() { return mWebotsProtoList; };
  QMap<QString, WbProtoInfo *> worldFileProtoList() { return mWorldFileProtoList; };
  QMap<QString, WbProtoInfo *> projectProtoList() { return mProjectProtoList; };
  QMap<QString, WbProtoInfo *> extraProtoList() { return mExtraProtoList; };
  // proto list generators

  WbProtoInfo *generateInfoFromProtoFile(const QString &protoFileName);
  void generateWorldFileProtoList();
  void generateExtraProtoList();
  void generateProjectProtoList();

  void exportProto(const QString &proto);

  const QVector<WbExternProtoInfo *> &externProto() const { return mExternProto; };
  void declareExternProto(const QString &protoName, const QString &protoPath, bool ephemeral);  // TODO: rename insert?
  void removeExternProto(const QString &protoName);
  void refreshExternProtoList();
  bool isDeclaredExternProto(const QString &protoName);

signals:
  void retrievalCompleted();

private slots:
  void tryWorldLoad();
  void singleProtoRetrievalCompleted();

private:
  // cppcheck-suppress unknownMacro
  Q_DISABLE_COPY(WbProtoManager)

  QString mPrimarySearchPath;
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

  // stores metadata about
  QMap<QString, WbProtoInfo *> mWebotsProtoList;     // loaded from proto-list.xml
  QMap<QString, WbProtoInfo *> mWorldFileProtoList;  // compiled from EXTERNPROTO referenced in .wbt
  QMap<QString, WbProtoInfo *> mProjectProtoList;    // compiled from PROTO in current project directory
  QMap<QString, WbProtoInfo *> mExtraProtoList;      // compiled from PROTO in extra project directories

  void generateWebotsProtoList();

  // TODO: add a proper reset function that resets all that needs to be reset inbetween world loads
};

#endif
