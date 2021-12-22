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

#ifndef WB_PROTO_CACHED_INFO_HPP
#define WB_PROTO_CACHED_INFO_HPP

//
// Description: PROTO information associated with a PROTO model and cached in file
//

#include <QtCore/QDateTime>
#include <QtCore/QString>
#include <QtCore/QStringList>

class WbProtoCachedInfo {
public:
  // create cached information for this PROTO file name
  explicit WbProtoCachedInfo(const QString &protoFileName);

  // destructor
  ~WbProtoCachedInfo();

  // return if PROTO file was modified since last computation of cached information
  bool isOutOfDate() const;

  // does the associated PROTO model contain at least one device node
  bool needsRobotAncestor() const { return mNeedsRobotAncestor; }
  const QString &info() const { return mInfo; }
  const QString &license() const { return mLicense; }
  const QString &licenseUrl() const { return mLicenseUrl; }
  const QString &documentationUrl() const { return mDocumentationUrl; }
  const QString &baseType() const { return mBaseType; }
  const QString &slotType() const { return mSlotType; }
  const QStringList &parameterNames() const { return mParameterNames; }
  const QStringList &tags() const { return mTags; }

  // load
  bool load();

  // compute and save the information of the PROTO specified in the file
  static WbProtoCachedInfo *computeInfo(const QString &protoFileName);

private:
  enum InfoType { FILE_HASH, DEVICES, BASE_TYPE, SLOT_TYPE, PARAMETERS, TAGS, LICENSE, LICENSE_URL, DOCUMENTATION_URL, INFO };
  QString mAbsoluteFileName;
  QString mAbsoluteProtoFileName;
  QByteArray mHexProtoFileHash;
  bool mNeedsRobotAncestor;
  QString mBaseType;
  QString mSlotType;
  QStringList mParameterNames;
  QStringList mTags;
  QString mInfo;
  QString mLicense;
  QString mLicenseUrl;
  QString mDocumentationUrl;

  // save cache file
  bool save();

  static QByteArray computeHexFileHash(const QString &filePath);
  static QString label(InfoType infoType);

  static QString serializedString(const QString &text);
  static QString deserializedString(const QString &serializedText);
};

#endif
