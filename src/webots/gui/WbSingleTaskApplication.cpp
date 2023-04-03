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

#include "WbSingleTaskApplication.hpp"

#include "WbApplicationInfo.hpp"
#include "WbBasicJoint.hpp"
#include "WbField.hpp"
#include "WbProtoManager.hpp"
#include "WbProtoModel.hpp"
#include "WbSolid.hpp"
#include "WbSolidReference.hpp"
#include "WbSoundEngine.hpp"
#include "WbSysInfo.hpp"
#include "WbTokenizer.hpp"
#include "WbVersion.hpp"
#include "WbWorld.hpp"

#include <QtCore/QCommandLineParser>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtGui/QOpenGLContext>
#include <QtGui/QOpenGLFunctions>
#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QtWidgets/QMainWindow>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#endif

#include <iostream>

using namespace std;

void WbSingleTaskApplication::run() {
  if (mTask == WbGuiApplication::SYSINFO)
    showSysInfo();
  else if (mTask == WbGuiApplication::HELP)
    showHelp();
  else if (mTask == WbGuiApplication::VERSION)
    cout << tr("Webots version: %1").arg(WbApplicationInfo::version().toString(true, false, true)).toUtf8().constData() << endl;
  else if (mTask == WbGuiApplication::UPDATE_WORLD)
    WbWorld::instance()->save();
  else if (mTask == WbGuiApplication::CONVERT)
    convertProto();

  emit finished(mTask == WbGuiApplication::FAILURE ? EXIT_FAILURE : EXIT_SUCCESS);
}

void WbSingleTaskApplication::convertProto() const {
  QCommandLineParser cliParser;
  cliParser.setApplicationDescription("Convert a PROTO file to a URDF file");
  cliParser.addHelpOption();
  cliParser.addPositionalArgument("input", "Path to the input PROTO file.");
  cliParser.addOption(QCommandLineOption("o", "Path to the output file.", "output"));
  cliParser.addOption(QCommandLineOption("p", "Override default PROTO parameters.", "parameter=value"));
  cliParser.process(mTaskArguments);
  const QStringList positionalArguments = cliParser.positionalArguments();
  if (positionalArguments.size() != 1)
    cliParser.showHelp(1);

  const bool toStdout = cliParser.values("o").size() == 0;

  QString outputFile;
  if (!toStdout)
    outputFile = cliParser.values("o")[0];

  // Compute absolute paths for input and output files
  QString inputFile = positionalArguments[0];
  if (QDir::isRelativePath(inputFile))
    inputFile = mStartupPath + '/' + inputFile;
  if (!toStdout && QDir::isRelativePath(outputFile))
    outputFile = mStartupPath + '/' + outputFile;

  if (!QFile(inputFile).exists()) {
    cerr << tr("File '%1' is not locally available, the conversion cannot take place.").arg(inputFile).toUtf8().constData()
         << endl;
    return;
  }

  // Get user parameters strings
  QMap<QString, QString> userParameters;
  for (QString param : cliParser.values("p")) {
    QStringList pair = param.split("=");
    if (pair.size() != 2) {
      cerr << tr("A parameter is not properly formatted!\n").toUtf8().constData();
      cliParser.showHelp(1);
    }
    userParameters[pair[0]] = pair[1].replace(QRegularExpression("^\"*"), "").replace(QRegularExpression("\"*$"), "");
  }

  // Parse PROTO
  WbNode::setInstantiateMode(false);
  WbProtoModel *model = WbProtoManager::instance()->readModel(inputFile, "");
  if (!toStdout)
    cout << tr("Parsing the %1 PROTO...").arg(model->name()).toUtf8().constData() << endl;

  // Combine the user parameters with the default ones
  QVector<WbField *> fields;
  for (WbFieldModel *fieldModel : model->fieldModels()) {
    WbField *field = new WbField(fieldModel);
    if (userParameters.contains(field->name())) {
      WbTokenizer tokenizer;
      tokenizer.tokenizeString(userParameters[field->name()]);
      field->readValue(&tokenizer, "");
    }

    if (!toStdout)
      cout << tr("  field %1 [%2] = %3")
                .arg(field->name())
                .arg(field->value()->vrmlTypeName())
                .arg(field->value()->toString())
                .toUtf8()
                .constData()
           << endl;
    fields.append(field);
  }

  // Generate a node structure
  WbNode::setInstantiateMode(true);
  WbNode *node = WbNode::createProtoInstanceFromParameters(model, fields, "");
  for (WbNode *subNode : node->subNodes(true)) {
    if (dynamic_cast<WbSolidReference *>(subNode))
      cout << tr("Warning: Exporting a Joint node with a SolidReference endpoint (%1) to URDF is not supported.")
                .arg(static_cast<WbSolidReference *>(subNode)->name())
                .toUtf8()
                .constData()
           << endl;
    if (dynamic_cast<WbSolid *>(subNode))
      static_cast<WbSolid *>(subNode)->updateChildren();
    if (dynamic_cast<WbBasicJoint *>(subNode)) {
      static_cast<WbBasicJoint *>(subNode)->updateEndPoint();
      static_cast<WbBasicJoint *>(subNode)->updateEndPointZeroTranslationAndRotation();
    }
  }

  // Export
  QString output;
  WbWriter writer(&output, "robot.urdf");
  writer.writeHeader(outputFile);
  node->write(writer);
  writer.writeFooter();

  // Output the content
  if (toStdout)
    cout << output.toUtf8().toStdString() << endl;
  else {
    QFile file(outputFile);
    if (!file.open(QIODevice::WriteOnly)) {
      cerr << tr("Cannot open the file!\n").toUtf8().constData();
      cliParser.showHelp(1);
    }
    file.write(output.toUtf8());
    file.close();
  }

  if (!toStdout)
    cout << tr("The %1 PROTO is written to the file.").arg(model->name()).toUtf8().constData() << endl;
}

void WbSingleTaskApplication::showHelp() const {
  cerr << tr("Usage: webots [options] [worldfile]").toUtf8().constData() << endl << endl;
  cerr << tr("Options:").toUtf8().constData() << endl << endl;
  cerr << "  --help" << endl;
  cerr << tr("    Display this help message and exit.").toUtf8().constData() << endl << endl;
  cerr << "  --version" << endl;
  cerr << tr("    Display version information and exit.").toUtf8().constData() << endl << endl;
  cerr << "  --sysinfo" << endl;
  cerr << tr("    Display information about the system and exit.").toUtf8().constData() << endl << endl;
  cerr << "  --mode=<mode>" << endl;
  cerr << tr("    Choose the startup mode, overriding application preferences. The <mode>").toUtf8().constData() << endl;
  cerr << tr("    argument must be either pause, realtime or fast.").toUtf8().constData() << endl << endl;
  cerr << "  --no-rendering" << endl;
  cerr << tr("    Disable rendering in the main 3D view.").toUtf8().constData() << endl << endl;
  cerr << "  --fullscreen" << endl;
  cerr << tr("    Start Webots in fullscreen.").toUtf8().constData() << endl << endl;
  cerr << "  --minimize" << endl;
  cerr << tr("    Minimize the Webots window on startup.").toUtf8().constData() << endl << endl;
  cerr << "  --batch" << endl;
  cerr << tr("    Prevent Webots from creating blocking pop-up windows.").toUtf8().constData() << endl << endl;
  cerr << "  --clear-cache" << endl;
  cerr << tr("    Clear the cache of Webots on startup.").toUtf8().constData() << endl << endl;
  cerr << "  --stdout" << endl;
  cerr << tr("    Redirect the stdout of the controllers to the terminal.").toUtf8().constData() << endl << endl;
  cerr << "  --stderr" << endl;
  cerr << tr("    Redirect the stderr of the controllers to the terminal.").toUtf8().constData() << endl << endl;
  cerr << "  --port" << endl;
  cerr << tr("    Change the TCP port used by Webots (default value is 1234).").toUtf8().constData() << endl << endl;
  cerr << "  --stream[=<mode>]" << endl;
  cerr << tr("    Start the Webots streaming server. The <mode> argument should be either").toUtf8().constData() << endl;
  cerr << tr("    x3d (default) or mjpeg.").toUtf8().constData() << endl << endl;
  cerr << "  --extern-urls" << endl;
  cerr << tr("    Print on stdout the URL of extern controllers that should be started.").toUtf8().constData() << endl << endl;
  cerr << "  --heartbeat[=<time>]" << endl;
  cerr << tr("    Print a dot (.) on stdout every second or <time> milliseconds if specified.").toUtf8().constData() << endl
       << endl;
  cerr << "  --log-performance=<file>[,<steps>]" << endl;
  cerr << tr("    Measure the performance of Webots and log it in the file specified in the").toUtf8().constData() << endl;
  cerr << tr("    <file> argument. The optional <steps> argument is an integer value that").toUtf8().constData() << endl;
  cerr << tr("    specifies how many steps are logged. If the --sysinfo option is used, the").toUtf8().constData() << endl;
  cerr << tr("    system information is prepended into the log file.").toUtf8().constData() << endl << endl;
  cerr << "  convert" << endl;
  cerr << tr("    Convert a PROTO file to a URDF file.").toUtf8().constData() << endl << endl;
  cerr << tr("Please report any bug to https://cyberbotics.com/bug").toUtf8().constData() << endl;
}

void WbSingleTaskApplication::showSysInfo() const {
  cout << tr("System: %1").arg(WbSysInfo::sysInfo()).toUtf8().constData() << endl;
  cout << tr("Processor: %1").arg(WbSysInfo::processor()).toUtf8().constData() << endl;
  cout << tr("Number of cores: %1").arg(WbSysInfo::coreCount()).toUtf8().constData() << endl;
  cout << tr("OpenAL device: %1").arg(WbSoundEngine::device()).toUtf8().constData() << endl;

  // create simply an OpenGL context
  QMainWindow mainWindow;
  QOpenGLWidget openGlWidget(&mainWindow);
  mainWindow.setCentralWidget(&openGlWidget);
  mainWindow.show();

  // An OpenGL context is required there for the OpenGL calls like `glGetString`.
  // The format is QSurfaceFormat::defaultFormat() => OpenGL 3.3 defined in main.cpp.
  QOpenGLContext *context = new QOpenGLContext();
  context->create();
  QOpenGLFunctions *gl = context->functions();  // QOpenGLFunctions_3_3_Core cannot be initialized here on some systems like
                                                // macOS High Sierra and some Ubuntu environments.

#ifdef _WIN32
  const quint32 vendorId = WbSysInfo::gpuVendorId(gl);
  const quint32 rendererId = WbSysInfo::gpuDeviceId(gl);
#else
  const quint32 vendorId = 0;
  const quint32 rendererId = 0;
#endif

  const char *vendor = reinterpret_cast<const char *>(gl->glGetString(GL_VENDOR));
  const char *renderer = reinterpret_cast<const char *>(gl->glGetString(GL_RENDERER));
  // cppcheck-suppress knownConditionTrueFalse
  if (vendorId == 0)
    cout << tr("OpenGL vendor: %1").arg(vendor).toUtf8().constData() << endl;
  else
    cout << tr("OpenGL vendor: %1 (0x%2)").arg(vendor).arg(vendorId, 0, 16).toUtf8().constData() << endl;
  // cppcheck-suppress knownConditionTrueFalse
  if (rendererId == 0)
    cout << tr("OpenGL renderer: %1").arg(renderer).toUtf8().constData() << endl;
  else
    cout << tr("OpenGL renderer: %1 (0x%2)").arg(renderer).arg(rendererId, 0, 16).toUtf8().constData() << endl;
  cout << tr("OpenGL version: %1").arg(reinterpret_cast<const char *>(gl->glGetString(GL_VERSION))).toUtf8().constData()
       << endl;

  delete context;
}
