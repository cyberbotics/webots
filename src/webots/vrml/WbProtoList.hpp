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

#ifndef WB_PROTO_LIST_HPP
#define WB_PROTO_LIST_HPP

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

  QString url() { return mUrl; }
  QString baseType() { return mBaseType; }
  QString license() { return mLicense; }
  QString licenseUrl() { return mLicenseUrl; }
  QString documentationUrl() { return mDocumentationUrl; }
  QString description() { return mDescription; }
  QString slotType() { return mSlotType; }
  QStringList tags() { return mTags; }
  bool needsRobotAncestor() { return mNeedsRobotAncestor; }

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

class WbProtoList : public QObject {
  Q_OBJECT
public:
  static WbProtoList *instance();

  // return all proto files stored in valid project folders located in the given path
  static void findProtosRecursively(const QString &dirPath, QFileInfoList &protoList, bool inProtos = false);

  static QStringList fileList();

  static QStringList fileList(int cache);

  // create a proto list with a .proto file search path
  // the path will be searched recursively
  // explicit WbProtoList(const QString &primarySearchPath = "");
  // explicit WbProtoList(const QString &world, bool reloading = false);
  WbProtoList();

  // TODO: is there better way to build it?
  bool areProtoAssetsAvailable(const QString &filename, const QStringList &graftedExternProto, bool buildProtoList = true);
  bool isWebotsProto(const QString &protoName);
  const QString getWebotsProtoUrl(const QString &protoName);

  // bool backwardsCompatibilityProtoRetrieval(const QStringList &protoList, const QString &filename, bool reloading);

  // destroys the list and all the contained models
  ~WbProtoList();

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
  QMap<QString, WbProtoInfo *> worldProtoList() { return mWorldProtoList; };
  QMap<QString, WbProtoInfo *> projectProtoList() { return mProjectProtoList; };
  QMap<QString, WbProtoInfo *> extraProtoList() { return mExtraProtoList; };
  // proto list generators

  WbProtoInfo *generateInfoFromProtoFile(const QString &protoFileName);
  void generateWorldProtoList();
  void generateExtraProtoList();
  void generateProjectProtoList();

signals:
  void retrievalCompleted();

private slots:
  void tryWorldLoad();
  void singleProtoRetrievalCompleted();

private:
  // cppcheck-suppress unknownMacro
  Q_DISABLE_COPY(WbProtoList)

  QString mPrimarySearchPath;
  QList<WbProtoModel *> mModels;

  QString mRetrievalError;
  QString mCurrentWorld;
  bool mReloading;

  WbProtoTreeItem *mTreeRoot;

  QMap<QString, QPair<QString, int>> mCurrentWorldProto;  // TODO: probably should be renamed

  // stores metadata about
  QMap<QString, WbProtoInfo *> mWebotsProtoList;   // loaded from proto-list.xml
  QMap<QString, WbProtoInfo *> mWorldProtoList;    // compiled from EXTERNPROTO referenced in .wbt
  QMap<QString, WbProtoInfo *> mProjectProtoList;  // compiled from PROTO in current project directory
  QMap<QString, WbProtoInfo *> mExtraProtoList;    // compiled from PROTO in extra project directories

  void generateWebotsProtoList();
};

#endif
