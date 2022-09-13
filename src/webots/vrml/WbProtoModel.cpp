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

#include "WbProtoModel.hpp"

#include "WbField.hpp"
#include "WbFieldModel.hpp"
#include "WbFileUtil.hpp"
#include "WbLog.hpp"
#include "WbNetwork.hpp"
#include "WbNode.hpp"
#include "WbNodeModel.hpp"
#include "WbNodeReader.hpp"
#include "WbParser.hpp"
#include "WbProtoManager.hpp"
#include "WbProtoTemplateEngine.hpp"
#include "WbStandardPaths.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"
#include "WbUrl.hpp"
#include "WbValue.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QStringList>
#include <QtCore/QTemporaryFile>
#include <QtCore/QTextStream>
#include <QtCore/QUrl>

#include <cassert>

WbProtoModel::WbProtoModel(WbTokenizer *tokenizer, const QString &worldPath, const QString &url, const QString &prefix,
                           QStringList baseTypeList) {
  // nodes in proto parameters or proto body should not be instantiated
  assert(!WbNode::instantiateMode());

  mDerived = false;
  QString baseTypeSlotType;

  mFileVersion = tokenizer->fileVersion();

  mInfo.clear();
  const QString &tokenizerInfo = tokenizer->info();
  if (!tokenizerInfo.isEmpty() && !tokenizerInfo.trimmed().isEmpty()) {
    const QStringList info = tokenizerInfo.split("\n");  // # comments
    for (int i = 0; i < info.size(); ++i) {
      if (!info.at(i).startsWith("tags:") && !info.at(i).startsWith("license:") && !info.at(i).startsWith("license url:") &&
          !info.at(i).startsWith("documentation url:") && !info.at(i).startsWith("template language:"))
        mInfo += info.at(i) + "\n";
    }
    mInfo.chop(1);
  }
  mTags = tokenizer->tags();
  mLicense = tokenizer->license();
  mLicenseUrl = tokenizer->licenseUrl();
  mDocumentationUrl = tokenizer->documentationUrl();
  mTemplateLanguage = tokenizer->templateLanguage();
  mIsDeterministic = !mTags.contains("nonDeterministic");

  WbParser parser(tokenizer);
  while (tokenizer->peekWord() == "EXTERNPROTO" || tokenizer->peekWord() == "IMPORTABLE")  // consume EXTERNPROTO declarations
    parser.skipExternProto();

  while (tokenizer->hasMoreTokens() && tokenizer->peekWord() != "PROTO")
    tokenizer->nextToken();
  tokenizer->skipToken("PROTO");
  mName = tokenizer->nextWord();
  // check recursive definition
  if (baseTypeList.contains(mName)) {
    tokenizer->reportError(tr("Recursive definition of PROTO node '%1' is not supported").arg(baseTypeList.first()));
    throw 0;
  }
  // check if PROTO name is not an existing base node name
  if (WbNodeModel::baseModelNames().contains(mName)) {
    tokenizer->reportError(tr("PROTO node '%1' cannot have a base node name").arg(mName));
    throw 0;
  }

  mRefCount = 0;
  mAncestorRefCount = 0;

  mPrefix = prefix;
  mUrl = url;

  assert(mUrl.endsWith(".proto", Qt::CaseInsensitive));      // mUrl needs to be the full reference, including file name
  assert(WbUrl::isWeb(mUrl) || QDir::isAbsolutePath(mUrl));  // by this point, all urls must be resolved

  if (!mUrl.endsWith(mName + ".proto", Qt::CaseInsensitive)) {
    tokenizer->reportFileError(tr("'%1' PROTO identifier does not match filename").arg(mName));
    throw 0;
  }

  // start proto parameters list
  tokenizer->skipToken("[");

  // read proto parameters
  while (tokenizer->hasMoreTokens() && tokenizer->peekWord() != "]") {
    WbFieldModel *parameter = NULL;
    try {
      parameter = new WbFieldModel(tokenizer, worldPath);
    } catch (...) {
      tokenizer->reportError(tr("Errors when parsing the PROTO parameters"), tokenizer->peekToken());
      throw 0;
    }
    // qDebug() << parameter->name() << ((parameter->defaultValue() != NULL) ? parameter->defaultValue()->vrmlTypeName() :
    // "NULL");
    if (findFieldModel(parameter->name())) {
      tokenizer->reportError(tr("Ignoring duplicate '%1' PROTO parameter declaration").arg(parameter->name()),
                             parameter->nameToken());
      parameter->destroy();
    } else {
      parameter->ref();
      mFieldModels.append(parameter);
    }
  }

  tokenizer->skipToken("]");
  tokenizer->skipToken("{");

  const WbToken *token = tokenizer->peekToken();
  int contentLine = token->line() - 1;
  mContentStartingLine = contentLine;
  int contentColumn = token->column() - 1;

  const QString &open = WbProtoTemplateEngine::openingToken();
  const QString &close = WbProtoTemplateEngine::closingToken();

  QFile file(diskPath());
  if (file.open(QIODevice::ReadOnly)) {
    for (int i = 0; i < contentLine; i++)
      file.readLine();
    file.read(contentColumn);

    mContent.clear();

    QTextStream in(&file);
    bool insideTemplateStatement = false;
    while (!in.atEnd()) {
      QString line = in.readLine();

      // Remove comments in order to filter out the template statements using VRML comments.
      // Typical cases to support:
      //   #"my string" # my comment
      //   "my string" # my comment
      //   "my string" # my "quoted comment"
      //   "my string with # character" # my comment
      //   "my string with protected \"quote\"." # my comment
      QString lineWithoutComments;
      bool insideDoubleQuotes = false;
      QChar pc;
      for (int i = 0; i < line.size(); ++i) {
        const QChar c = line[i];
        if (c == open[1] && pc == open[0])
          insideTemplateStatement = true;
        else if (c == close[1] && pc == close[0])
          insideTemplateStatement = false;
        else if (c == '"' && pc != '\\')
          insideDoubleQuotes = !insideDoubleQuotes;
        else if (!insideTemplateStatement && c == '#' && !insideDoubleQuotes && mTemplateLanguage == "lua")
          // ignore VRML comments
          // but '#' is the lua length operator and has to be kept if found inside a template statement
          break;
        lineWithoutComments.append(c);
        pc = c;
      }

      mContent += lineWithoutComments + "\n";
    }
    int lastClosingBracket = mContent.lastIndexOf('}');
    if (lastClosingBracket > 0)
      mContent = mContent.left(lastClosingBracket);
    else {
      tokenizer->reportFileError(tr("Bracket issue"));
      throw 0;
    }
    file.close();
  }

  // inject the prefix prior to tokenizing the content
  if (!mPrefix.isEmpty() && mPrefix != "webots://")
    mContent.replace(QString("webots://").toUtf8(), mPrefix.toUtf8());

  // read the remaining tokens in order to
  // - determine if it's a template
  // - check which parameter need to regenerate the template instance
  mTemplate = false;
  mAncestorProtoModel = NULL;
  token = NULL;
  const WbToken *previousToken = NULL;
  bool readBaseType = true;
  QStringList sharedParameterNames;
  QString previousRedirectedFieldName;
  bool hasPreviousSlotTypeFieldName = false;
  int insideBrackets = -1;
  while (tokenizer->hasMoreTokens()) {
    previousToken = token;
    token = tokenizer->nextToken();

    if (token->isTemplateStatement()) {
      mTemplate = true;

      foreach (WbFieldModel *model, mFieldModels) {
        // condition explanation: if (token contains modelName and not a Lua identifier containing modelName such as
        // "my_awesome_modelName")
        if (token->word().contains(QRegularExpression(
              QString("(^|[^a-zA-Z0-9_])fields\\.%1($|[^a-zA-Z0-9_])").arg(QRegularExpression::escape(model->name()))))) {
          model->setTemplateRegenerator(true);
        }
      }
    } else if (readBaseType) {
      // read base type
      if (token->word() == "DEF") {
        // skip DEF name
        tokenizer->nextToken();
        // read base type
        token = tokenizer->nextToken();
      }

      mBaseType = token->word();
      mDerived = !WbNodeModel::isBaseModelName(mBaseType);
      mAncestorProtoName = "";
      readBaseType = false;

      if (mDerived) {
        QString protoInfo, baseNodeType;
        bool error = false;
        try {
          baseTypeList.append(mName);
          WbProtoModel *baseProtoModel = WbProtoManager::instance()->findModel(mBaseType, worldPath, url, baseTypeList);
          mAncestorProtoModel = baseProtoModel;
          if (baseProtoModel) {
            mAncestorProtoName = mBaseType;
            mBaseType = baseProtoModel->baseType();
            QStringList derivedParameterNames = parameterNames();
            QStringList baseParameterNames = baseProtoModel->parameterNames();
            baseTypeSlotType = baseProtoModel->slotType();
            foreach (QString name, derivedParameterNames) {
              if (baseParameterNames.contains(name))
                sharedParameterNames.append(name);
            }
          } else
            error = true;
        } catch (...) {
          error = true;
        }

        if (error) {
          tokenizer->reportError(tr("Errors when parsing the base PROTO"), token);
          throw 0;
        }

        baseTypeList.removeLast();
      }
    } else if (sharedParameterNames.contains(token->word()) && !previousRedirectedFieldName.isEmpty()) {
      // check that derived parameter is only redirected to corresponding base parameter
      QString parameterName = token->word();
      if (previousRedirectedFieldName != token->word()) {
        tokenizer->reportError(
          tr("Derived and base PROTO can use the same parameter name only if it is linked directly, like '%1 IS %1'")
            .arg(parameterName));
        throw 0;
      }
    } else if (token->isString()) {
      // check which parameter need to regenerate the template instance from inside a string
      foreach (WbFieldModel *model, mFieldModels) {
        // regex test cases:
        // "You know nothing, John Snow."  => false
        // "%{=fields.model->name()}%"  => false
        // "%{= fields.model->name().value.x }% %{= fields.model->name().value.y }%"  => true
        // "abc %{= fields.model->name().value.y }% def"  => true
        // "%{= 17 % fields.model->name().value.y * 88 }%"  => true
        // "fields.model->name().value.y"  => false
        // "%{}% fields.model->name().value.y %{}%"  => false
        // "%{ a = \"fields.model->name().value.y\" }%"  => false
        // "%{= \"fields.model->name().value.y\" }%"  => false
        // "%{= fields.model->name().value.y }%"  => true
        if (token->word().contains(QRegularExpression(QString("%1(?:(?!%2|\").)*fields\\.%3(?:(?!%4|\").)*%5")
                                                        .arg(open)
                                                        .arg(close)
                                                        .arg(QRegularExpression::escape(model->name()))
                                                        .arg(close)
                                                        .arg(close))))
          model->setTemplateRegenerator(true);
      }
    }

    if (mSlotType.isEmpty() && mBaseType == "Slot") {
      if (hasPreviousSlotTypeFieldName) {
        if (token->isString())
          mSlotType = token->toString();
        hasPreviousSlotTypeFieldName = false;
      } else if (insideBrackets == 0 && token->word() == "type") {
        hasPreviousSlotTypeFieldName = true;
      } else if (token->word() == "{") {
        ++insideBrackets;
      } else if (token->word() == "}") {
        --insideBrackets;
      }
    }

    if (token->word() == "IS") {
      assert(previousToken);
      previousRedirectedFieldName = previousToken->word();
    } else {
      previousRedirectedFieldName.clear();
    }
  }

  if (mSlotType.isEmpty() && mBaseType == "Slot" && mDerived)
    mSlotType = baseTypeSlotType;

  if (mDocumentationUrl.isEmpty()) {
    const QStringList &bookAndPage = documentationBookAndPage(mBaseType == "Robot", true);
    if (!bookAndPage.isEmpty())
      mDocumentationUrl =
        QString("%1/doc/%2/%3").arg(WbStandardPaths::cyberboticsUrl()).arg(bookAndPage[0]).arg(bookAndPage[1]);
  }
}

WbProtoModel::~WbProtoModel() {
  foreach (WbFieldModel *model, mFieldModels)
    model->unref();
  mFieldModels.clear();
  mDeterministicContentMap.clear();
}

WbNode *WbProtoModel::generateRoot(const QVector<WbField *> &parameters, const QString &worldPath, int uniqueId) {
  if (mContent.isEmpty())
    return NULL;

  assert(!WbNode::instantiateMode());

  WbTokenizer tokenizer;
  tokenizer.setErrorOffset(mContentStartingLine);

  int rootUniqueId = -1;
  QString content = mContent;
  if (mTemplate) {
    QString key;
    if (mIsDeterministic) {
      foreach (WbField *parameter, parameters) {
        if (parameter->isTemplateRegenerator()) {
          QString statement = WbProtoTemplateEngine::convertFieldValueToJavaScriptStatement(parameter);
          if (mTemplateLanguage == "lua")
            statement = WbProtoTemplateEngine::convertStatementFromJavaScriptToLua(statement);
          key += statement;
        }
      }
    }
    if (!mIsDeterministic || (!mDeterministicContentMap.contains(key) || mDeterministicContentMap.value(key).isEmpty())) {
      WbProtoTemplateEngine te(mContent);
      rootUniqueId = uniqueId >= 0 ? uniqueId : WbNode::getFreeUniqueId();
      if (!te.generate(name() + ".proto", parameters, mUrl, worldPath, rootUniqueId, mTemplateLanguage)) {
        tokenizer.setReferralFile(mUrl);
        tokenizer.reportFileError(tr("Template engine error: %1").arg(te.error()));
        return NULL;
      }
      content = te.result();
      if (mIsDeterministic)
        mDeterministicContentMap.insert(key, content);
    } else
      content = mDeterministicContentMap.value(key);
  } else
    mIsDeterministic = true;

  tokenizer.setReferralFile(mUrl);
  if (tokenizer.tokenizeString(content) > 0) {
    tokenizer.reportFileError(tr("Failed to load due to syntax error(s)"));
    return NULL;
  }

  // parse generated PROTO
  WbParser parser(&tokenizer);
  if (!parser.parseProtoBody(worldPath))
    return NULL;
  tokenizer.rewind();

  // read node in a local DEF/USE scope
  WbNode *root = NULL;

  try {
    WbNodeReader reader(WbNodeReader::PROTO_MODEL);
    root = reader.readNode(&tokenizer, worldPath);
  } catch (...) {
    tokenizer.reportFileError(tr("Cannot create the '%1' PROTO").arg(mName));
    return NULL;
  }

  if (!root) {
    tokenizer.reportFileError(tr("PROTO has unknown base type"));
    return NULL;
  }

  // aliasing error reports are based on the header, so the error offset has no sense here
  tokenizer.setErrorOffset(0);

  verifyAliasing(root, &tokenizer);

  if (mTemplate) {
    root->setProtoInstanceTemplateContent(content.toUtf8());
    if (rootUniqueId >= 0 && rootUniqueId != uniqueId)
      root->setUniqueId(rootUniqueId);
  }

  return root;
}

void WbProtoModel::ref(bool isFromProtoInstanceCreation) {
  mRefCount++;
  if (isFromProtoInstanceCreation)
    mAncestorRefCount++;
}

void WbProtoModel::unref() {
  mRefCount--;
  if (mRefCount < mAncestorRefCount) {
    mAncestorRefCount--;
    if (isDerived() && mAncestorProtoModel)
      mAncestorProtoModel->unref();
  }
  if (mRefCount == 0)
    delete this;
}

void WbProtoModel::destroy() {
  assert(mRefCount == 0);
  delete this;
}

WbFieldModel *WbProtoModel::findFieldModel(const QString &fieldName) const {
  foreach (WbFieldModel *fieldModel, mFieldModels)
    if (fieldModel->name() == fieldName)
      return fieldModel;

  return NULL;  // not found
}

const QString WbProtoModel::projectPath() const {
  QString protoPath = path();

  if (!protoPath.isEmpty()) {
    if (WbUrl::isWeb(protoPath))
      protoPath.replace(QRegularExpression(WbUrl::remoteWebotsAssetRegex(false)), WbStandardPaths::webotsHomePath());
#ifdef __APPLE__
    if (WbFileUtil::isLocatedInInstallationDirectory(protoPath, true))
      protoPath.insert(WbStandardPaths::webotsHomePath().length(), "Contents/");
#endif

    QDir protoProjectDir(protoPath);
    while (protoProjectDir.dirName() != "protos") {
      QString dir = protoProjectDir.path();
      // cd up (we don't use QDir::cdUp() as it doesn't cd up if the upper folder doesn't exist which may happen here)
      dir.chop(protoProjectDir.dirName().size());
      assert(!dir.isEmpty());
      protoProjectDir.setPath(dir);
      if (protoProjectDir.isRoot())
        return QString();
    }
    protoProjectDir.cdUp();
    return protoProjectDir.absolutePath();
  }
  return QString();
}

QStringList WbProtoModel::parameterNames() const {
  QStringList names;
  foreach (WbFieldModel *fieldModel, mFieldModels)
    names.append(fieldModel->name());
  return names;
}

void WbProtoModel::verifyNodeAliasing(WbNode *node, WbFieldModel *param, WbTokenizer *tokenizer, bool searchInParameters,
                                      bool &ok) const {
  QVector<WbField *> fields;
  if (searchInParameters)
    fields = node->parameters();
  else
    fields = node->fields();

  // search self
  foreach (WbField *field, fields) {
    if (field->alias() == param->name()) {
      if (field->type() == param->type())
        ok = true;
      else
        tokenizer->reportError(
          tr("Type mismatch between '%1' PROTO parameter and field '%2'").arg(param->name(), field->name()),
          param->nameToken());
    }
  }

  // recursively search sub nodes
  const QList<WbNode *> &l = node->subNodes(false, !searchInParameters, true);
  foreach (WbNode *subnode, l) {
    if (subnode->isProtoInstance())
      // search only in parameters of sub protos: fields are in sub proto parameter scope
      verifyNodeAliasing(subnode, param, tokenizer, true, ok);
    else
      verifyNodeAliasing(subnode, param, tokenizer, false, ok);
  }
}

// verify that each proto parameter has at least one matching IS parameter
void WbProtoModel::verifyAliasing(WbNode *root, WbTokenizer *tokenizer) const {
  if (!root)
    return;

  foreach (WbFieldModel *param, mFieldModels) {
    if (param->isUnconnected())
      continue;
    bool ok = false;
    verifyNodeAliasing(root, param, tokenizer, isDerived(), ok);
    if (!isTemplate() && !ok)
      tokenizer->reportError(tr("PROTO parameter '%1' has no matching IS field").arg(param->name()), param->nameToken());
  }
}

QStringList WbProtoModel::documentationBookAndPage(bool isRobot, bool skipProtoTag) const {
  QStringList bookAndPage;
  if (isRobot) {
    // check for robot doc
    const QString &name = mName.toLower();

    const QString page("guide/" + name + ".md");
    if (checkIfDocumentationPageExist(page)) {
      bookAndPage << "guide" << name;
      return bookAndPage;
    }
  } else {
    // check for object doc
    const QDir &objectsDir(WbStandardPaths::projectsPath() + "objects");
    QDir dir(projectPath());
    QString name = dir.dirName().replace('_', '-');
    while (!dir.isRoot()) {
      if (dir == objectsDir) {
        const QString page("guide/object-" + name + ".md");
        if (checkIfDocumentationPageExist(page)) {
          bookAndPage << "guide"
                      << "object-" + name;
          return bookAndPage;
        }
        break;
      }
      name = dir.dirName().replace('_', '-');
      if (!dir.cdUp())
        break;
    }
  }
  if (!skipProtoTag) {
    const QString &documentationUrl = mDocumentationUrl;
    if (!documentationUrl.isEmpty()) {
      const QStringList &splittedPath = documentationUrl.split("doc/");
      if (splittedPath.size() == 2) {
        const QString file(splittedPath[1].split('#')[0]);
        const QString page(file + ".md");
        if (checkIfDocumentationPageExist(page)) {
          bookAndPage = file.split('/');
          if (splittedPath[1].contains('#'))
            bookAndPage[1] += '#' + splittedPath[1].split('#')[1];
          return bookAndPage;
        }
      }
    }
  }

  return bookAndPage;  // return empty
}

bool WbProtoModel::checkIfDocumentationPageExist(const QString &page) const {
  bool exist = false;
  QFile file(WbStandardPaths::webotsHomePath() + "docs/list.txt");
  if (!file.open(QIODevice::ReadOnly))
    return false;
  QTextStream in(&file);
  QString line = in.readLine();
  while (!line.isNull()) {
    if (line.contains(page, Qt::CaseSensitive)) {
      exist = true;
      break;
    }
    line = in.readLine();
  }

  file.close();

  return exist;
}

const QString WbProtoModel::diskPath() const {
  if (WbUrl::isWeb(mUrl))
    return WbNetwork::instance()->get(mUrl);

  return mUrl;
}

const QString WbProtoModel::path() const {
  if (WbUrl::isWeb(mUrl))
    return QUrl(mUrl).adjusted(QUrl::RemoveFilename).toString();

  return QFileInfo(mUrl).absolutePath() + "/";
}
