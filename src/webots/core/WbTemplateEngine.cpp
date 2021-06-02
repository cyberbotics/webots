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

WbTemplateEngine::WbTemplateEngine(const QString &templateContent) : mTemplateContent(templateContent) {
}

const QString &WbTemplateEngine::openingToken() {
  return gOpeningToken;
}

const QString &WbTemplateEngine::closingToken() {
  return gClosingToken;
}

bool WbTemplateEngine::generate(QHash<QString, QString> tags, const QString &logHeaderName, const QString &templateLanguage) {
  bool result;

  if (templateLanguage == "lua") {
    static bool firstCall = true;

    if (firstCall) {
      initialize();
      firstCall = false;
    }
    result = generateLua(tags, logHeaderName);
  } else
    result = generateJavascript(tags, logHeaderName);

  return result;
}

bool WbTemplateEngine::generateJavascript(QHash<QString, QString> tags, const QString &logHeaderName) {
  printf("JS: tokens are %s %s\n", gOpeningToken.toUtf8().constData(), gClosingToken.toUtf8().constData());

  mResult.clear();
  mError = "";

  // cd to temporary directory
  bool success = QDir::setCurrent(WbStandardPaths::webotsTmpPath());
  if (!success) {
    mError = tr("Cannot change directory to: '%1'").arg(WbStandardPaths::webotsTmpPath());
    return false;
  }

  // translate mixed proto into pure JavaScript
  QString javaScriptBody = "";
  QString javaScriptImport = "";

  int indexClosingToken = 0;
  int lastIndexClosingToken = -1;
  mTemplateContent = mTemplateContent.toUtf8();
  while (1) {
    int indexOpeningToken = mTemplateContent.indexOf(gOpeningToken, indexClosingToken);
    if (indexOpeningToken == -1) {  // no more matches
      if (indexClosingToken < mTemplateContent.size()) {
        javaScriptBody +=
          "result += render(`" + mTemplateContent.mid(indexClosingToken, mTemplateContent.size() - indexClosingToken) + "`);";
      }
      break;
    }

    indexClosingToken = mTemplateContent.indexOf(gClosingToken, indexOpeningToken);
    indexClosingToken = indexClosingToken + gClosingToken.size();  // point after the template token

    if (indexClosingToken == -1) {
      mError = tr("Expected JavaScript closing token '%1' is missing.").arg(gClosingToken);
      return false;
    }

    if (indexOpeningToken > 0 && lastIndexClosingToken == -1) {
      // what comes before the first opening token should be treated as plain text
      javaScriptBody += "result += render(`" + mTemplateContent.left(indexOpeningToken) + "`);";
    }
    if (lastIndexClosingToken != -1 && indexOpeningToken - lastIndexClosingToken > 0) {
      // what is between the previous closing token and the current opening token should be treated as plain text
      javaScriptBody +=
        "result += render(`" + mTemplateContent.mid(lastIndexClosingToken, indexOpeningToken - lastIndexClosingToken) + "`);";
    }
    // anything inbetween the tokens is either an expression or plain JavaScript
    QString statement = mTemplateContent.mid(indexOpeningToken, indexClosingToken - indexOpeningToken);
    // if it starts with '%{=' it's an expression
    if (statement.startsWith(gOpeningToken + "=")) {
      statement = statement.replace(gOpeningToken + "=", "").replace(gClosingToken, "");
      javaScriptBody +=
        "var __tmp = " + statement + "; result += eval(\"__tmp\");";  // var because there might be multiple expressions
    } else {
      // raw javascript snippet
      javaScriptBody += statement.replace(gOpeningToken, "").replace(gClosingToken, "");
    }

    lastIndexClosingToken = indexClosingToken;
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

  // load template and fill it
  QFile templateFile(WbStandardPaths::resourcesPath() + "javascript/jsTemplate.js");
  if (!templateFile.open(QIODevice::ReadOnly)) {
    mError = tr("Javascript template not found.");
    return false;
  }
  QString javaScriptTemplate = templateFile.readAll();

  // create engine and stream holders
  QJSEngine engine;
  QJSValue stdout = engine.newArray();
  engine.globalObject().setProperty("stdout", stdout);
  QJSValue stderr = engine.newArray();
  engine.globalObject().setProperty("stderr", stderr);

  // fill template
  javaScriptTemplate.replace("%import%", javaScriptImport);
  javaScriptTemplate.replace("%context%", tags["context"]);
  javaScriptTemplate.replace("%fields%", tags["fields"]);
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

  for (int i = 0; i < stdout.property("length").toInt(); ++i)
    WbLog::instance()->info(QString("'%1': JavaScript output: %2").arg(logHeaderName).arg(stdout.property(i).toString()), false,
                            WbLog::PARSING);

  for (int i = 0; i < stderr.property("length").toInt(); ++i)
    WbLog::instance()->error(QString("'%1': JavaScript error: %2").arg(logHeaderName).arg(stderr.property(i).toString()), false,
                             WbLog::PARSING);

  // remove temporary file. TODO: UNCOMMENT BEFORE MERGE
  // QFile::remove(WbStandardPaths::resourcesPath() + "javascript/jsTemplateFilled.js");

  mResult = result.toString().toUtf8();
  return true;
}

bool WbTemplateEngine::generateLua(QHash<QString, QString> tags, const QString &logHeaderName) {
  mResult.clear();

  printf("LUA: tokens are %s %s\n", gOpeningToken.toUtf8().constData(), gClosingToken.toUtf8().constData());
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

  QDir::setCurrent(initialDir);

  // cleanup lua
  lua_close(state);

  return true;
}
