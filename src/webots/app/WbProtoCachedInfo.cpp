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

#include "WbProtoCachedInfo.hpp"

#include "WbNode.hpp"
#include "WbNodeUtilities.hpp"
#include "WbParser.hpp"
#include "WbProtoModel.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"
#include "WbWorld.hpp"

#include <QtCore/QCryptographicHash>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QTextStream>

#ifdef _WIN32
#include <windows.h>
#include <QtCore/QDir>
#endif

#include <cassert>

static QString gFileHeader = "Webots Proto Cache File version 1.5";

WbProtoCachedInfo::WbProtoCachedInfo(const QString &protoFileName) : mNeedsRobotAncestor(false), mBaseType("UNKNOWN") {
  mHexProtoFileHash.clear();
  QFileInfo info(protoFileName);
  mAbsoluteProtoFileName = info.absoluteFilePath();
  mAbsoluteFileName = info.absolutePath() + "/." + info.completeBaseName() + ".cache";
}

WbProtoCachedInfo::~WbProtoCachedInfo() {
}

bool WbProtoCachedInfo::load() {
  mParameterNames.clear();
  mTags.clear();
  mLicense.clear();
  mLicenseUrl.clear();
  mDocumentationUrl.clear();

  QFile file(mAbsoluteFileName);
  if (!file.exists() || !file.open(QIODevice::ReadOnly | QIODevice::Text))
    return false;

  QTextStream in(&file);
  if (in.atEnd())
    return false;

  QString line = in.readLine();
  if (line != gFileHeader) {
    file.close();
    return false;
  }

  QString word;
  while (!in.atEnd()) {
    line = in.readLine();
    QTextStream ls(&line, QIODevice::ReadOnly);
    QString key;
    ls >> key;
    if (key == label(FILE_HASH)) {
      QByteArray hash;
      ls >> hash;
      mHexProtoFileHash = hash;
    } else if (key == label(DEVICES)) {
      int i;
      ls >> i;
      mNeedsRobotAncestor = i;
    } else if (key == label(BASE_TYPE)) {
      ls >> word;
      mBaseType = word;
    } else if (key == label(SLOT_TYPE)) {
      ls >> word;
      mSlotType = deserializedString(word);
    } else if (key == label(PARAMETERS)) {
      ls >> word;
      mParameterNames = word.split(",");
    } else if (key == label(TAGS)) {
      ls >> word;
      mTags = word.split(",");
    } else if (key == label(LICENSE)) {
      line.remove("license:");
      mLicense = line;
    } else if (key == label(LICENSE_URL)) {
      line.remove("licenseUrl:");
      mLicenseUrl = line;
    } else if (key == label(DOCUMENTATION_URL)) {
      line.remove("documentationUrl:");
      mDocumentationUrl = line.trimmed();
    } else if (key == label(INFO)) {
      ls >> word;
      mInfo = deserializedString(word);
    }
  }

  file.close();
  return true;
}

bool WbProtoCachedInfo::save() {
  mHexProtoFileHash = computeHexFileHash(mAbsoluteProtoFileName);

  QFile file(mAbsoluteFileName);
  if (!file.open(QIODevice::WriteOnly)) {
    mHexProtoFileHash.clear();
    return false;
  }

  QTextStream out(&file);
  out << gFileHeader << "\n";
  out << label(FILE_HASH) << " " << mHexProtoFileHash << "\n";
  out << label(DEVICES) << " " << (mNeedsRobotAncestor ? 1 : 0) << "\n";
  out << label(BASE_TYPE) << " " << mBaseType << "\n";
  out << label(SLOT_TYPE) << " " << serializedString(mSlotType) << "\n";
  out << label(PARAMETERS) << " " << mParameterNames.join(",") << "\n";
  out << label(TAGS) << " " << mTags.join(",") << "\n";
  out << label(LICENSE) << " " << mLicense << "\n";
  out << label(LICENSE_URL) << " " << mLicenseUrl << "\n";
  out << label(DOCUMENTATION_URL) << " " << mDocumentationUrl << "\n";
  out << label(INFO) << " " << serializedString(mInfo) << "\n";

  file.close();

#ifdef _WIN32
  // set hidden attribute
  LPCSTR nativePath = QDir::toNativeSeparators(mAbsoluteFileName).toUtf8().constData();
  SetFileAttributes(nativePath, GetFileAttributes(nativePath) | FILE_ATTRIBUTE_HIDDEN);
#endif

  return true;
}

bool WbProtoCachedInfo::isOutOfDate() const {
  return mHexProtoFileHash != computeHexFileHash(mAbsoluteProtoFileName);
}

WbProtoCachedInfo *WbProtoCachedInfo::computeInfo(const QString &protoFileName) {
  printf("protocacheinfo");
  WbTokenizer tokenizer;
  int errors = tokenizer.tokenize(protoFileName);
  if (errors > 0)
    // invalid PROTO file
    return NULL;

  WbParser parser(&tokenizer);
  if (!parser.parseProtoInterface(WbWorld::instance() ? WbWorld::instance()->fileName() : "")) {
    // invalid PROTO file
    return NULL;
  }

  tokenizer.rewind();
  WbProtoModel *protoModel = NULL;
  bool prevInstantiateMode = WbNode::instantiateMode();
  WbNode *previousParent = WbNode::globalParentNode();
  try {
    WbNode::setGlobalParentNode(NULL);
    WbNode::setInstantiateMode(false);
    protoModel = new WbProtoModel(&tokenizer, WbWorld::instance() ? WbWorld::instance()->fileName() : "", protoFileName);
    WbNode::setInstantiateMode(prevInstantiateMode);
    WbNode::setGlobalParentNode(previousParent);
  } catch (...) {
    WbNode::setInstantiateMode(prevInstantiateMode);
    WbNode::setGlobalParentNode(previousParent);
    return NULL;
  }

  // compute and store proto info
  WbProtoCachedInfo *protoInfo = new WbProtoCachedInfo(protoFileName);
  protoInfo->mInfo = protoModel->info();
  protoInfo->mBaseType = protoModel->baseType();
  protoInfo->mSlotType = protoModel->slotType();
  protoInfo->mParameterNames = protoModel->parameterNames();
  protoInfo->mTags = protoModel->tags();
  protoInfo->mLicense = protoModel->license();
  protoInfo->mLicenseUrl = protoModel->licenseUrl();
  protoInfo->mDocumentationUrl = protoModel->documentationUrl();

  tokenizer.rewind();
  while (tokenizer.hasMoreTokens()) {
    WbToken *token = tokenizer.nextToken();
    if (token->isIdentifier() && WbNodeUtilities::isDeviceTypeName(token->word()) && token->word() != "Connector") {
      printf(">>>%s\n", token->word().toUtf8().constData());
      protoInfo->mNeedsRobotAncestor = true;
      break;
    }
  }

  protoInfo->save();
  protoModel->destroy();
  return protoInfo;
}

QByteArray WbProtoCachedInfo::computeHexFileHash(const QString &filePath) {
  QCryptographicHash crypto(QCryptographicHash::Md5);
  QFile file(filePath);
  file.open(QFile::ReadOnly | QFile::Text);

  while (!file.atEnd())
    crypto.addData(file.read(8192));

  QByteArray hash = crypto.result().toHex();
  file.close();
  return hash;
}

QString WbProtoCachedInfo::label(InfoType infoType) {
  switch (infoType) {
    case FILE_HASH:
      return "protoFileHash:";
    case DEVICES:
      return "needsRobotAncestor:";
    case BASE_TYPE:
      return "baseType:";
    case SLOT_TYPE:
      return "slotType:";
    case PARAMETERS:
      return "parameters:";
    case TAGS:
      return "tags:";
    case LICENSE:
      return "license:";
    case LICENSE_URL:
      return "licenseUrl:";
    case DOCUMENTATION_URL:
      return "documentationUrl:";
    case INFO:
      return "info:";
    default:
      assert(false);
      return QString();
  }
}

QString WbProtoCachedInfo::serializedString(const QString &text) {
  if (text.isEmpty())
    return text;

  QString serializedText(text);
  serializedText.replace("\\", "\\\\");
  serializedText.replace("\n", "\\n");
  serializedText.replace("\r", "\\r");
  serializedText.replace(" ", "\\s");
  serializedText.replace("\t", "\\t");
  return serializedText;
}

QString WbProtoCachedInfo::deserializedString(const QString &serializedText) {
  if (serializedText.isEmpty())
    return serializedText;

  static QList<QRegularExpression> whiteSpacesRegExpList;
  static QList<QString> replaceStringList;
  if (whiteSpacesRegExpList.isEmpty()) {
    whiteSpacesRegExpList.append(QRegularExpression("(^|[^\\\\])\\\\n"));
    whiteSpacesRegExpList.append(QRegularExpression("(^|[^\\\\])\\\\r"));
    whiteSpacesRegExpList.append(QRegularExpression("(^|[^\\\\])\\\\t"));
    whiteSpacesRegExpList.append(QRegularExpression("(^|[^\\\\])\\\\s"));
    replaceStringList.append("\n");
    replaceStringList.append("\r");
    replaceStringList.append("\t");
    replaceStringList.append(" ");
  }

  QString text(serializedText);
  for (int i = 0; i < whiteSpacesRegExpList.size(); ++i) {
    QRegularExpressionMatch match = whiteSpacesRegExpList[i].match(text);
    while (match.hasMatch()) {
      text.replace(match.capturedStart() + 1, 2, replaceStringList[i]);
      match = whiteSpacesRegExpList[i].match(text, match.capturedStart() + 1);
    }
  }
  text = text.replace("\\\\", "\\");
  return text;
}
