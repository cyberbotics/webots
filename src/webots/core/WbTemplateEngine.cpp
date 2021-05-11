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
  printf("WbTemplateEngine::initialize()\n");
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

void WbTemplateEngine::initializeJavascriptEngine() {
  printf("WbTemplateEngine::initializeJavascriptEngine\n");
}

WbTemplateEngine::WbTemplateEngine(const QString &templateContent, const QString &language) {
  static bool firstCall = true;
  mLanguage = language;

  if (language == "lua" && firstCall) {
    initialize();
    firstCall = false;
  }

  if (language == "javascript")
    initializeJavascriptEngine();

  mTemplateContent = templateContent;
}

const QString &WbTemplateEngine::openingToken() {
  return gOpeningToken;
}

const QString &WbTemplateEngine::closingToken() {
  return gClosingToken;
}

bool WbTemplateEngine::generate(QHash<QString, QString> tags, const QString &logHeaderName) {
  bool result;

  if (mLanguage == "javascript")
    result = generateJavascript(tags, logHeaderName);
  else
    result = generateLua(tags, logHeaderName);

  return result;
}

bool WbTemplateEngine::generateJavascript(QHash<QString, QString> tags, const QString &logHeaderName) {
  printf("WbTemplateEngine::generateJavascript\n\n\n");

  mResult.clear();
  mError = "";

  // cd to temporary directory
  bool success = QDir::setCurrent(WbStandardPaths::webotsTmpPath());
  if (!success) {
    mError = tr("Cannot change directory to: '%1'").arg(WbStandardPaths::webotsTmpPath());
    return false;
  }

  tags["templateContent"] = mTemplateContent;
  tags["templateContent"] = tags["templateContent"].replace("\\", "\\\\");
  tags["templateContent"] = tags["templateContent"].replace("\n", "\\n");
  tags["templateContent"] = tags["templateContent"].replace("'", "\\'");
  tags["templateContent"] = tags["templateContent"].toUtf8();

  QJSEngine engine;
  engine.installExtensions(QJSEngine::ConsoleExtension);
  // engine.evaluate("console.log(\"%1\".arg(\"TEST\")");
  // translate mixed proto into javascript
  int start = -1;
  int end = -1;
  QString jsBody = "";
  QRegularExpression reTemplate("\%{(.*?)}\%");
  QRegularExpressionMatchIterator it = reTemplate.globalMatch(tags["templateContent"]);

  while (it.hasNext()) {
    QRegularExpressionMatch match = it.next();
    if (match.hasMatch()) {
      if (start == -1 && end == -1)
        start = match.capturedEnd();
      if (start != -1 && end == -1)
        if (match.capturedStart() != 0)
          end = match.capturedStart();

      if (start != -1 && end != -1) {
        jsBody += "result += render(`" + tags["templateContent"].mid(start, end - start) + "`);";
        start = match.capturedEnd();
        end = -1;
      }

      QString matched = match.captured(0);

      if (matched.startsWith("%{=")) {
        matched = matched.replace("%{=", "");
        matched = matched.replace("}%", "");
        jsBody += "result += eval(\"" + matched + "\");";
      } else if (matched.startsWith("%{")) {
        matched = matched.replace("%{", "");
        matched = matched.replace("}%", "");
        jsBody += matched;
      }
    }
  }

  // load template and replace body with javascript code
  QFile templateFile(WbStandardPaths::resourcesPath() + "javascript/jsTemplate.js");
  if (!templateFile.open(QIODevice::ReadOnly)) {
    mError = tr("Javascript template not found.");
    return false;
  }
  QString jsTemplate = templateFile.readAll();

  // replace fields, TODO: move to WbProtoTemplateEngine
  // should be: var car = {field:"size", value:2, defaultValue:1};
  printf("&& context raw &&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
  printf("%s\n", tags["context"].toUtf8().constData());
  printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n\n");
  jsTemplate.replace("%context%", tags["context"]);

  // replace fields, TODO: move to WbProtoTemplateEngine
  // should be: var car = {field:"size", value:2, defaultValue:1};
  printf("## fields raw ###############################\n");
  printf("%s\n", tags["fields"].toUtf8().constData());
  // printf("## jsFields #################################\n");
  // QString jsFields = "";
  // QStringList f = tags["fields"].split("\n");
  // if (f.size() > 0) {
  //  jsFields += "var fields = {";
  //  for (int i = 0; i < f.size(); ++i) {
  //    jsFields += f[i].replace("value = ", "value:").replace("defaultValue = ", "defaultValue:");
  //  }
  //  jsFields += "};";
  //}
  // printf("%s\n", jsFields.toUtf8().constData());
  printf("#############################################\n\n");
  jsTemplate.replace("%fields%", tags["fields"]);

  jsBody += "result += render(`" + tags["templateContent"].mid(start) + "`);";
  // remove escapes
  printf("\n== tags[\"templateContent\"] ================\n");
  printf("%s\n", tags["templateContent"].toUtf8().constData());
  printf("== jsBody literal ===========================\n");
  printf("%s\n", jsBody.toLatin1().constData());
  printf("== jsBody ===================================\n");
  jsBody = jsBody.replace("\\n", "\n");
  printf("%s\n", jsBody.toUtf8().constData());
  printf("=============================================\n\n");

  // replace body
  jsTemplate.replace("%body%", jsBody);

  // evaluate and get result
  QJSValue result = engine.evaluate(jsTemplate);
  printf(">> result >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
  printf("%s\n", result.toString().toUtf8().constData());
  printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n\n");

  mResult = result.toString().toUtf8();
  return true;
}

bool WbTemplateEngine::generateLua(QHash<QString, QString> tags, const QString &logHeaderName) {
  printf("WbTemplateEngine::generateLua\n\n\n");
  mResult.clear();

  if (!gValidLuaResources) {
    mError = tr("Installation error: Lua resources are not found");
    return false;
  }

  mError = "";
  QString initialDir = QDir::currentPath();

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
