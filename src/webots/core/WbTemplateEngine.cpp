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

#include "WbTemplateEngine.hpp"

#include "WbLog.hpp"
#include "WbProject.hpp"
#include "WbQjsFile.hpp"
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
static bool gValidJavaScriptResources = true;
static QString gLuaTemplateFileContent;
static QString gJavaScriptTemplateFileContent;

namespace {
  // Note: not the default opening/closing tokens in order to allow
  //       VRML comments to comment templates
  QString gOpeningToken("%{");  // default: "#{"
  QString gClosingToken("}%");  // default: "}#"
};                              // namespace

void WbTemplateEngine::copyModuleToTemporaryFile(QString modulePath) {
  QDir luaModulesPath(modulePath);
  if (luaModulesPath.exists()) {
    QStringList filters;
    filters << "*.lua";
#ifdef _WIN32
    filters << "*.dll";
#elif defined(__linux__)
    filters << "*.so";
#else  // __APPLE__
    filters << "*.dylib";
#endif
    QFileInfoList files = luaModulesPath.entryInfoList(filters, QDir::Files | QDir::NoSymLinks);
    foreach (const QFileInfo &file, files)
      QFile::copy(file.absoluteFilePath(), WbStandardPaths::webotsTmpPath() + file.fileName());
  }
}

void WbTemplateEngine::initializeJavaScript() {
  // copy JavaScript modules to the temporary directory
  QDirIterator it(WbStandardPaths::resourcesPath() + "web/wwi/protoVisualizer/templating/", QDirIterator::Subdirectories);
  while (it.hasNext()) {
    QDir jsModulesPath(it.next());

    if (jsModulesPath.exists()) {
      QStringList filter("*.js");
      QFileInfoList files = jsModulesPath.entryInfoList(filter, QDir::Files | QDir::NoSymLinks);
      foreach (const QFileInfo &file, files)
        QFile::copy(file.absoluteFilePath(), WbStandardPaths::webotsTmpPath() + file.fileName());
    }
  }

  // load template skeleton only once
  QFile templateFile(WbStandardPaths::webotsTmpPath() + "jsTemplate.js");
  if (!templateFile.open(QIODevice::ReadOnly)) {
    gValidJavaScriptResources = false;
    return;
  }

  gJavaScriptTemplateFileContent = templateFile.readAll();
}

void WbTemplateEngine::initializeLua() {
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
  gLuaTemplateFileContent = templateFile.readAll();
  templateFile.close();
}

WbTemplateEngine::WbTemplateEngine(const QString &templateContent) : mTemplateContent(templateContent) {
}

void WbTemplateEngine::setOpeningToken(const QString &token) {
  gOpeningToken = token;
}

void WbTemplateEngine::setClosingToken(const QString &token) {
  gClosingToken = token;
}

const QString &WbTemplateEngine::openingToken() {
  return gOpeningToken;
}

const QString &WbTemplateEngine::closingToken() {
  return gClosingToken;
}

QString WbTemplateEngine::escapeString(const QString &string) {
  QString escaped(string);
  return escaped.replace("\\", "\\\\").replace("\n", "\\n").replace("'", "\\'").toUtf8();
}

bool WbTemplateEngine::generate(QHash<QString, QString> tags, const QString &logHeaderName, const QString &templateLanguage) {
  bool output;

  if (templateLanguage == "lua") {
    static bool firstLuaCall = true;
    if (firstLuaCall) {
      initializeLua();
      firstLuaCall = false;
    }

    gOpeningToken = "%{";
    gClosingToken = "}%";

    output = generateLua(tags, logHeaderName);
  } else {
    static bool firstJavaScriptCall = true;
    if (firstJavaScriptCall) {
      initializeJavaScript();
      firstJavaScriptCall = false;
    }

    gOpeningToken = "%<";
    gClosingToken = ">%";

    output = generateJavascript(tags, logHeaderName);
  }

  return output;
}

bool WbTemplateEngine::generateJavascript(QHash<QString, QString> tags, const QString &logHeaderName) {
  mResult.clear();
  mError = "";

  QString initialDir = QDir::currentPath();

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
  const QString expressionToken = gOpeningToken + "=";
  while (1) {
    int indexOpeningToken = mTemplateContent.indexOf(gOpeningToken, indexClosingToken);
    if (indexOpeningToken == -1) {  // no more matches
      if (indexClosingToken < mTemplateContent.size()) {
        // what comes after the last closing token is plain vrml
        // note: ___vrml is a local variable to the generateVrml javascript function
        javaScriptBody +=
          "___vrml += render(`" + mTemplateContent.mid(indexClosingToken, mTemplateContent.size() - indexClosingToken) + "`);";
        break;
      }
    }

    indexClosingToken = mTemplateContent.indexOf(gClosingToken, indexOpeningToken);
    if (indexClosingToken == -1) {
      mError = tr("Expected JavaScript closing token '%1' is missing.").arg(gClosingToken);
      return false;
    }

    indexClosingToken = indexClosingToken + gClosingToken.size();  // point after the template token

    if (indexOpeningToken > 0 && lastIndexClosingToken == -1)
      // what comes before the first opening token should be treated as plain vrml
      javaScriptBody += "___vrml += render(`" + mTemplateContent.left(indexOpeningToken) + "`);";

    if (lastIndexClosingToken != -1 && indexOpeningToken - lastIndexClosingToken > 0)
      // what is between the previous closing token and the current opening token should be treated as plain vrml
      javaScriptBody +=
        "___vrml += render(`" + mTemplateContent.mid(lastIndexClosingToken, indexOpeningToken - lastIndexClosingToken) + "`);";

    // anything inbetween the tokens is either an expression or plain JavaScript
    QString statement = mTemplateContent.mid(indexOpeningToken, indexClosingToken - indexOpeningToken);
    // if it starts with '%<=' it's an expression
    if (statement.startsWith(expressionToken)) {
      statement = statement.replace(expressionToken, "").replace(gClosingToken, "");
      // note: ___tmp is a local variable to the generateVrml javascript function
      javaScriptBody += "___tmp = " + statement + "; ___vrml += eval(\"___tmp\");";
    } else {
      // raw javascript snippet, remove the tokens
      javaScriptBody += statement.replace(gOpeningToken, "").replace(gClosingToken, "");
    }

    lastIndexClosingToken = indexClosingToken;
  }

  // extract imports from javaScriptBody, if any
  // QRegularExpression explanation: any statement of the form "import ... from '...' " that ends with a new line or semi-colon
  QRegularExpression reImport("import(.*?from.*?'.*?')[;\n]");
  QRegularExpressionMatchIterator it = reImport.globalMatch(javaScriptBody);
  while (it.hasNext()) {
    const QRegularExpressionMatch match = it.next();
    if (match.hasMatch()) {
      QString statement = match.captured();
      javaScriptBody.replace(statement, "");  // remove it from javaScriptBody

      if (statement.endsWith(";"))
        statement.append("\n");
      else if (statement.endsWith("\n") && statement.at(statement.size() - 2) != QString(";"))
        statement.insert(statement.size() - 2, ";");
      else
        statement.append(";\n");

      javaScriptImport += statement;
    }
  }

  // fill template skeleton
  QString javaScriptTemplate = gJavaScriptTemplateFileContent;
  javaScriptTemplate.replace("%import%", javaScriptImport);
  javaScriptTemplate.replace("%context%", tags["context"]);
  javaScriptTemplate.replace("%fields%", tags["fields"]);
  javaScriptTemplate.replace("%body%", javaScriptBody);

  // write to file (note: can't evaluate directly because the evaluator doesn't support importing of modules)
  QFile outputFile("jsTemplateFilled.js");
  if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
    mError = tr("Couldn't write jsTemplateFilled in %1").arg(QDir::currentPath());
    return false;
  }

  QTextStream outputStream(&outputFile);
  outputStream << javaScriptTemplate;
  outputFile.close();

  // create engine and define global space
  QJSEngine engine;
  // create and add file manipulation module
  WbQjsFile *jsFileObject = new WbQjsFile();
  QJSValue jsFile = engine.newQObject(jsFileObject);
  engine.globalObject().setProperty("wbfile", jsFile);
  // add stream holders
  QJSValue jsStdOut = engine.newArray();
  engine.globalObject().setProperty("stdout", jsStdOut);
  QJSValue jsStdErr = engine.newArray();
  engine.globalObject().setProperty("stderr", jsStdErr);
  // import filled template as module
  QJSValue module = engine.importModule("jsTemplateFilled.js");
  if (module.isError()) {
    mError = tr("failed to import JavaScript template: %1").arg(module.property("message").toString());
    return false;
  }

  QJSValue generateVrml = module.property("generateVrml");
  QJSValue r = generateVrml.call();
  if (r.isError()) {
    mError = tr("failed to execute JavaScript template: %1").arg(r.property("message").toString());
    return false;
  }

  mResult = r.toString().toUtf8();

  // display stream messages
  for (int i = 0; i < jsStdOut.property("length").toInt(); ++i)
    WbLog::instance()->info(QString("'%1': JavaScript output: %2").arg(logHeaderName).arg(jsStdOut.property(i).toString()),
                            false, WbLog::PARSING);

  for (int i = 0; i < jsStdErr.property("length").toInt(); ++i)
    WbLog::instance()->error(QString("'%1': JavaScript error: %2").arg(logHeaderName).arg(jsStdErr.property(i).toString()),
                             false, WbLog::PARSING);

  // restore initial directory
  QDir::setCurrent(initialDir);

  return true;
}

bool WbTemplateEngine::generateLua(QHash<QString, QString> tags, const QString &logHeaderName) {
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
#elif defined(__linux__)
  tags["cpath"] = "";
#else  // __APPLE__
  tags["cpath"] = "package.cpath = package.cpath .. \";?.dylib\"";
#endif

  tags["templateContent"] = escapeString(mTemplateContent);

  // make sure these key are set
  if (!tags.contains("fields"))
    tags["fields"] = "";
  if (!tags.contains("context"))
    tags["context"] = "";

  tags["openingToken"] = gOpeningToken;
  tags["closingToken"] = gClosingToken;
  tags["templateFileName"] = logHeaderName;

  QString scriptContent = gLuaTemplateFileContent;
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
