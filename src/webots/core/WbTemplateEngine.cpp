// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "WbTemplateEngine.hpp"

#include "WbLog.hpp"
#include "WbProject.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QDir>
#include <QtCore/QDirIterator>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QStringList>
#include <QtCore/QTemporaryFile>
#include <QtCore/QTextStream>
#include <QtCore/QVector>
#include <QtQml/QJSEngine>

#include <lua.hpp>

#include <cassert>

static bool gValidLuaResources = true;
static QString gTemplateFileContent;

namespace {
  // Note: not the default opening/closing tokens in order to allow
  //       VRML comments to comment templates
  const QString gOpeningToken("%{");  // default: "#{"
  const QString gClosingToken("}%");  // default: "}#"
};                                    // namespace

void WbTemplateEngine::copyModuleToTemporaryFile(QString modulePath) {
  QDir luaModulesPath(modulePath);
  if (luaModulesPath.exists()) {
    QStringList filters;
    filters << "*.lua";
#ifdef _WIN32
    filters << "*.dll";
#endif
#ifdef __linux__
    filters << "*.so";
#endif
#ifdef __APPLE__
    filters << "*.dylib";
#endif
    QFileInfoList files = luaModulesPath.entryInfoList(filters, QDir::Files | QDir::NoSymLinks);
    foreach (const QFileInfo &file, files)
      QFile::copy(file.absoluteFilePath(), WbStandardPaths::webotsTmpPath() + file.fileName());
  }
}

void WbTemplateEngine::initialize() {
  printf("WbTemplateEngine::initialize lua\n");
  QFileInfo luaSLT2Script(WbStandardPaths::resourcesPath() + "lua/liluat/liluat.lua");
  if (!luaSLT2Script.exists()) {
    gValidLuaResources = false;
    return;
  }

  // copy Lua modules files in the temp directory
  QDirIterator it(WbStandardPaths::resourcesPath() + "lua/modules");
  while (it.hasNext())
    copyModuleToTemporaryFile(it.next());

  // reference:  http://www.lua.org/pil/8.1.html
  QString LUA_PATH = qgetenv("LUA_PATH");
  QString LUA_PATH_ADD = luaSLT2Script.absolutePath() + "/?.lua;?.lua";
  if (LUA_PATH.isEmpty())
    qputenv("LUA_PATH", LUA_PATH_ADD.toUtf8());
  else
    qputenv("LUA_PATH", (LUA_PATH + ";" + LUA_PATH_ADD).toUtf8());

  // template file
  QFile templateFile(WbStandardPaths::resourcesPath() + "lua/liluat/templateScript.lua");
  if (!templateFile.open(QIODevice::ReadOnly)) {
    gValidLuaResources = false;
    return;
  }
  gTemplateFileContent = templateFile.readAll();
  templateFile.close();
}

WbTemplateEngine::WbTemplateEngine(const QString &templateContent) {
  mTemplateContent = templateContent;
}

const QString &WbTemplateEngine::openingToken() {
  return gOpeningToken;
}

const QString &WbTemplateEngine::closingToken() {
  return gClosingToken;
}

bool WbTemplateEngine::generate(QHash<QString, QString> tags, const QString &logHeaderName, const QString &templateLanguage) {
  printf("WbTemplateEngine::generate: %s\n", templateLanguage.toUtf8().constData());
  static bool firstCall = true;
  bool result;

  if (templateLanguage == "lua") {
    printf("GENERATING LUA\n");
    if (firstCall) {
      initialize();
      firstCall = false;
    }

    result = generateLua(tags, logHeaderName);
  } else {
    printf("GENERATING JAVASCRIPT\n");
    result = generateJavascript(tags, logHeaderName);
  }

  return result;
}

bool WbTemplateEngine::generateJavascript(QHash<QString, QString> tags, const QString &logHeaderName) {
  printf("WbTemplateEngine::generateJavascript\n\n");

  mResult.clear();
  mError = "";

  // cd to temporary directory
  bool success = QDir::setCurrent(WbStandardPaths::webotsTmpPath());
  if (!success) {
    mError = tr("Cannot change directory to: '%1'").arg(WbStandardPaths::webotsTmpPath());
    return false;
  }

  tags["templateContent"] = mTemplateContent;
  // tags["templateContent"] = tags["templateContent"].replace("\\", "\\\\");
  // tags["templateContent"] = tags["templateContent"].replace("\n", "\\n");
  // tags["templateContent"] = tags["templateContent"].replace("'", "\\'");
  tags["templateContent"] = tags["templateContent"].toUtf8();

  QJSEngine engine;
  // engine.installExtensions(QJSEngine::ConsoleExtension);
  // engine.globalObject().setProperty("stdout", QString(""));
  // engine.globalObject().setProperty("stderr", QString(""));
  QJSValue stdout = engine.newArray();
  engine.globalObject().setProperty("stdout", stdout);
  QJSValue stderr = engine.newArray();
  engine.globalObject().setProperty("stderr", stderr);
  /*
  QJSValue modulee = engine.importModule(WbStandardPaths::resourcesPath() + "javascript/test.js");

  if (modulee.isError())
    printf("ERROR LOADING MODULE\n");
  else
    printf("MODULE LOADED\n");

  QJSValue mainn = modulee.property("main");
  if (mainn.isError())
    printf("PROPERTY ERROR\n");
  else
    printf("PROPERTY FINE\n");

  QJSValue res = mainn.call();

  printf(">>>>>>>>>>>>>>>>>>>%s<<<<<<<<<<<<<<<<\n", res.toString().toUtf8().constData());
  return 0;
  */

  // translate mixed proto into pure JavaScript
  QString javaScriptBody = "";
  QString javaScriptImport = "";

  QString templateContent = tags["templateContent"];
  int positionOpeningToken = 0;
  int positionClosingToken = 0;
  // bool firstMatch = true;
  int previousPositionClosingToken = -1;
  while (1) {
    positionOpeningToken = templateContent.indexOf(gOpeningToken, positionClosingToken);
    if (positionOpeningToken == -1) {
      if (positionClosingToken < templateContent.size()) {
        javaScriptBody += "result += render(`" +
                          templateContent.mid(positionClosingToken, templateContent.size() - positionClosingToken) + "`);";
      }
      break;
    }

    positionClosingToken = templateContent.indexOf(gClosingToken, positionOpeningToken);
    positionClosingToken = positionClosingToken + 2;  // point after the template token

    if (positionClosingToken == -1) {
      printf("Missing closing bracket\n");  // mError?
      mError = tr("Expected JavaScript closing token is missing.");
      return false;
    }

    if (positionOpeningToken > 0 && previousPositionClosingToken == -1) {
      // what comes before the first opening token should be treated as plain text
      javaScriptBody += "result += render(`" + templateContent.left(positionOpeningToken) + "`);";
    }

    if (previousPositionClosingToken != -1 && positionOpeningToken - previousPositionClosingToken > 0) {
      // what is between the previous closing token and the current opening token should be treated as plain text
      javaScriptBody += "result += render(`" +
                        templateContent.mid(previousPositionClosingToken, positionOpeningToken - previousPositionClosingToken) +
                        "`);";
    }

    // anything inbetween the tokens is either an expression or plain JavaScript
    QString statement = templateContent.mid(positionOpeningToken, positionClosingToken - positionOpeningToken);
    // if it starts with '%{=' it's an expression
    if (statement.startsWith(gOpeningToken + "=")) {
      statement = statement.replace(gOpeningToken + "=", "").replace(gClosingToken, "");
      javaScriptBody +=
        "var __tmp = " + statement + "; result += eval(\"__tmp\");";  // var because there might be multiple expressions
    } else {
      // raw javascript snippet
      javaScriptBody += statement.replace(gOpeningToken, "").replace(gClosingToken, "");
    }

    previousPositionClosingToken = positionClosingToken;
  }

  // extract imports from javaScriptBody, if any
  QRegularExpression reImport("import(.*?)[;\n]");
  QRegularExpressionMatchIterator it = reImport.globalMatch(javaScriptBody);
  while (it.hasNext()) {
    QRegularExpressionMatch match = it.next();
    if (match.hasMatch()) {
      QString statement = match.captured(0);
      javaScriptBody.replace(statement, "");  // remove import from javaScriptBody

      if (statement.endsWith(";"))
        statement.append("\n");
      else if (statement.endsWith("\n") && statement.at(statement.size() - 2) != ";")
        statement.insert(statement.size() - 2, ";");
      else
        statement.append(";\n");

      javaScriptImport += statement;
    }
  }

  // load template and replace body with javascript code
  QFile templateFile(WbStandardPaths::resourcesPath() + "javascript/jsTemplate.js");
  if (!templateFile.open(QIODevice::ReadOnly)) {
    mError = tr("Javascript template not found.");
    return false;
  }
  QString javaScriptTemplate = templateFile.readAll();

  // add imports
  // javaScriptImport = javaScriptImport.replace("\\n", "\n");
  // javaScriptImport = javaScriptImport.replace("\\'", "'");
  // javaScriptImport = javaScriptImport.replace("\\\\", "\\");
  javaScriptTemplate.replace("%import%", javaScriptImport);

  printf("&& context raw &&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
  printf("%s\n", tags["context"].toUtf8().constData());
  printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n\n");
  javaScriptTemplate.replace("%context%", tags["context"]);

  printf("## fields raw ###############################\n");
  printf("%s\n", tags["fields"].toUtf8().constData());
  printf("#############################################\n\n");
  javaScriptTemplate.replace("%fields%", tags["fields"]);

  printf("\n== tags[\"templateContent\"] ================\n");
  printf("%s\n", tags["templateContent"].toUtf8().constData());
  printf("== jsBody literal ===========================\n");
  printf("%s\n", javaScriptBody.toLatin1().constData());
  printf("== jsBody ===================================\n");

  // restore escapements
  // javaScriptBody = javaScriptBody.replace("\\n", "\n");
  // javaScriptBody = javaScriptBody.replace("\\'", "'");
  // javaScriptBody = javaScriptBody.replace("\\\\", "\\");

  printf("%s\n", javaScriptBody.toUtf8().constData());
  printf("=============================================\n\n");
  // replace body
  javaScriptTemplate.replace("%body%", javaScriptBody);

  // write to file (note: can't evaluate directly because it doesn't support importing of modules)
  QFile outputFile(WbStandardPaths::resourcesPath() + "javascript/jsTemplateFilled.js");
  if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
    mError = tr("Couldn't write jsTemplateFilled to disk.");
    return false;
  }

  QTextStream outputStream(&outputFile);
  outputStream << javaScriptTemplate;
  outputFile.close();

  // import filled template as module
  QJSValue module = engine.importModule(WbStandardPaths::resourcesPath() + "javascript/jsTemplateFilled.js");
  if (module.isError()) {
    mError = tr("failed to import JavaScript template. On line %1: %2")
               .arg(module.property("lineNumber").toInt())
               .arg(module.property("message").toString());
    return false;
  }
  QJSValue main = module.property("main");
  QJSValue result = main.call();
  if (result.isError()) {
    mError = tr("failed to execute JavaScript template. On line %1: %2")
               .arg(result.property("lineNumber").toInt())
               .arg(result.property("message").toString());
    return false;
  }

  printf(">>>>stdout:\n");
  for (int i = 0; i < stdout.property("length").toInt(); ++i) {
    WbLog::instance()->info(QString("'%1': JavaScript output: %2").arg(logHeaderName).arg(stdout.property(i).toString()), false,
                            WbLog::PARSING);
    printf("%d: %s\n", i, stdout.property(i).toString().toUtf8().constData());
  }
  printf("<<<<<<\n");

  printf(">>>>stderr:\n");
  for (int i = 0; i < stderr.property("length").toInt(); ++i) {
    WbLog::instance()->error(QString("'%1': JavaScript error: %2").arg(logHeaderName).arg(stderr.property(i).toString()), false,
                             WbLog::PARSING);
    printf("%d: %s\n", i, stderr.property(i).toString().toUtf8().constData());
  }
  printf("<<<<<<\n");

  printf(">> result >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
  printf("%s\n", result.toString().toUtf8().constData());
  printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n\n");

  // remove temporary file. TODO: UNCOMMENT IN BEFORE MERGE
  // QFile::remove(WbStandardPaths::resourcesPath() + "javascript/jsTemplateFilled.js");

  mResult = result.toString().toUtf8();
  return true;
}

bool WbTemplateEngine::generateLua(QHash<QString, QString> tags, const QString &logHeaderName) {
  printf("WbTemplateEngine::generateLua\n\n");
  mResult.clear();

  if (!gValidLuaResources) {
    mError = tr("Installation error: Lua resources are not found");
    return false;
  }

  mError = "";
  QString initialDir = QDir::currentPath();

  // cd to temporary directory
  bool success = QDir::setCurrent(WbStandardPaths::webotsTmpPath());
  if (!success) {
    mError = tr("Cannot change directory to: '%1'").arg(WbStandardPaths::webotsTmpPath());
    return false;
  }

// Update 'package.cpath' variable to be able to load '*.dll' and '*.dylib'
#ifdef _WIN32
  tags["cpath"] = "package.cpath = package.cpath .. \";?.dll\"";
#endif
#ifdef __linux__
  tags["cpath"] = "";
#endif
#ifdef __APPLE__
  tags["cpath"] = "package.cpath = package.cpath .. \";?.dylib\"";
#endif

  tags["templateContent"] = mTemplateContent;
  tags["templateContent"] = tags["templateContent"].replace("\\", "\\\\");
  tags["templateContent"] = tags["templateContent"].replace("\n", "\\n");
  tags["templateContent"] = tags["templateContent"].replace("'", "\\'");
  tags["templateContent"] = tags["templateContent"].toUtf8();

  // make sure these key are set
  if (!tags.contains("fields"))
    tags["fields"] = "";
  if (!tags.contains("context"))
    tags["context"] = "";

  printf("## context raw ###############################\n");
  printf("%s\n", tags["context"].toUtf8().constData());
  printf("#############################################\n\n");

  printf("## fields raw ###############################\n");
  printf("%s\n", tags["fields"].toUtf8().constData());
  printf("#############################################\n\n");

  tags["openingToken"] = gOpeningToken;
  tags["closingToken"] = gClosingToken;
  tags["templateFileName"] = logHeaderName;

  QString scriptContent = gTemplateFileContent;
  QHashIterator<QString, QString> i(tags);
  while (i.hasNext()) {
    i.next();
    QString keyTag = QString("") + "%" + i.key() + "%";
    scriptContent.replace(keyTag, i.value());
  }

  printf("\n== tags[\"templateContent\"] ================\n");
  printf("%s\n", tags["templateContent"].toUtf8().constData());
  printf("=============================================\n\n");

  // needed for procedurale PROTO using lua-gd
  QString webotsFontsPath(QDir::toNativeSeparators(WbStandardPaths::fontsPath()));
  QString projectFontsPath(QDir::toNativeSeparators(WbProject::current()->path() + "fonts/"));
#ifdef _WIN32
  QString fontsPath = projectFontsPath + ";" + webotsFontsPath;
#else
  QString fontsPath = projectFontsPath + ":" + webotsFontsPath;
#endif

  qputenv("GDFONTPATH", fontsPath.toUtf8());

  // init lua
  lua_State *state;
  state = luaL_newstate();
  luaL_openlibs(state);

  // run the temporary Lua script
  int errors = luaL_dostring(state, scriptContent.toUtf8());
  if (errors != 0) {
    mError = tr("luaL_dostring error : %1").arg(lua_tostring(state, -1));
    QDir::setCurrent(initialDir);
    return false;
  }

  // Get stderr and display it to the console
  lua_getglobal(state, "stderrString");
  QString stderrContent = lua_tostring(state, -1);
  const QString newLine =
#ifdef _WIN32
    "\r\n";
#else
    "\n";
#endif
  QStringList stderrSplitted = stderrContent.split(newLine, Qt::SkipEmptyParts);
  foreach (const QString &line, stderrSplitted)
    WbLog::instance()->error(QString("'%1': Lua error: %2").arg(logHeaderName).arg(line), false, WbLog::PARSING);

  // Get stdout and display it to the console
  lua_getglobal(state, "stdoutString");
  QString stdoutContent = lua_tostring(state, -1);
  QStringList stdoutSplitted = stdoutContent.split(newLine, Qt::SkipEmptyParts);
  foreach (const QString &line, stdoutSplitted)
    WbLog::instance()->info(QString("'%1': Lua output: %2").arg(logHeaderName).arg(line), false, WbLog::PARSING);

  // Get the result
  lua_getglobal(state, "content");
  mResult = lua_tostring(state, -1);

  QString res = mResult;
  printf(">> result >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
  printf("%s\n", res.toUtf8().constData());
  printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n\n");

  QDir::setCurrent(initialDir);

  // cleanup lua
  lua_close(state);

  return true;
}
