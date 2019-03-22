// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "OSMImportWindow.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QTemporaryDir>
#include <QtNetwork/QNetworkReply>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

OSMImportWindow::OSMImportWindow(QString stringPath, QWidget *parent):
  QMainWindow(parent)
{
  mImporterProcess = NULL;
  setWindowTitle(tr("OpenStreetMap importer"));

  mCentralWidget = new QWidget(this);
  setCentralWidget(mCentralWidget);

  mMainLayout = new QGridLayout();
  mCentralWidget->setLayout(mMainLayout);

  // input
  mInputGroupBox = new QGroupBox(tr("Input"), this);
  mInputLayout = new QGridLayout();

  mFileRadioButton = new QRadioButton("File", this);
  mFileRadioButton->setChecked(false);
  mInputLayout->addWidget(mFileRadioButton, 0, 0);
  connect(mFileRadioButton, &QRadioButton::toggled, this, &OSMImportWindow::updateInputMethod);

  mFileLabel      = new QLabel("File:", this);
  mFileLineEdit   = new QLineEdit("", this);
  mFilePushButton = new QPushButton("Select", this);
  mFileLineEdit->setEnabled(false);
  mFilePushButton->setEnabled(false);
  mInputLayout->addWidget(mFileLabel, 1, 0);
  mInputLayout->addWidget(mFileLineEdit, 1, 1);
  mInputLayout->addWidget(mFilePushButton, 1, 2);
  connect(mFilePushButton, &QPushButton::pressed, this, &OSMImportWindow::selectInputFile);

  mCoordinateRadioButton = new QRadioButton("Coordinate", this);
  mCoordinateRadioButton->setChecked(true);
  mInputLayout->addWidget(mCoordinateRadioButton, 2, 0);
  connect(mCoordinateRadioButton, &QRadioButton::toggled, this, &OSMImportWindow::updateInputMethod);

  mMinLatLabel     = new QLabel("Min lat:", this);
  mMaxLatLabel     = new QLabel("Max lat:", this);
  mMinLongLabel    = new QLabel("Min long:", this);
  mMaxLongLabel    = new QLabel("Max long:", this);
  mMinLatLineEdit  = new QLineEdit("46.505", this);
  mMinLatLineEdit->setInputMask("#000.000000");
  mMaxLatLineEdit  = new QLineEdit("46.517", this);
  mMaxLatLineEdit->setInputMask("#000.000000");
  mMinLongLineEdit = new QLineEdit("6.496", this);
  mMinLongLineEdit->setInputMask("#000.000000");
  mMaxLongLineEdit = new QLineEdit("6.5140", this);
  mMaxLongLineEdit->setInputMask("#000.000000");
  mInputLayout->addWidget(mMinLatLineEdit, 3, 1);
  mInputLayout->addWidget(mMinLatLabel, 3, 0);
  mInputLayout->addWidget(mMaxLatLineEdit, 4, 1);
  mInputLayout->addWidget(mMaxLatLabel, 4, 0);
  mInputLayout->addWidget(mMinLongLineEdit, 5, 1);
  mInputLayout->addWidget(mMinLongLabel, 5, 0);
  mInputLayout->addWidget(mMaxLongLineEdit, 6, 1);
  mInputLayout->addWidget(mMaxLongLabel, 6, 0);

  mInputGroupBox->setLayout(mInputLayout);
  mMainLayout->addWidget(mInputGroupBox, 0, 0);

  // Projection
  mProjectionGroupBox   = new QGroupBox(tr("Projection"), this);
  mProjectionHBoxLayout = new QHBoxLayout();
  mProjectionLabel      = new QLabel("Projection:", this);
  mProjectionLineEdit   = new QLineEdit("", this);
  mProjectionHBoxLayout->addWidget(mProjectionLabel);
  mProjectionHBoxLayout->addWidget(mProjectionLineEdit);
  mProjectionGroupBox->setLayout(mProjectionHBoxLayout);
  mProjectionGroupBox->setToolTip(tr("Defines the projection parameters.\nLeave empty to use an UTM projection."));
  mMainLayout->addWidget(mProjectionGroupBox, 1, 0);

  // config file
  mConfigGroupBox   = new QGroupBox(tr("Configuration file"), this);
  mConfigHBoxLayout = new QHBoxLayout();
  mConfigLineEdit   = new QLineEdit("", this);
  mConfigPushButton = new QPushButton(tr("Select"), this);
  mConfigHBoxLayout->addWidget(mConfigLineEdit);
  mConfigHBoxLayout->addWidget(mConfigPushButton);
  mConfigGroupBox->setLayout(mConfigHBoxLayout);
  mConfigGroupBox->setToolTip(tr("Defines which configuration file to use.\nLeave empty to use the default one."));
  mMainLayout->addWidget(mConfigGroupBox, 2, 0, 1, 2);
  connect(mConfigPushButton, &QPushButton::pressed, this, &OSMImportWindow::selectConfigurationFile);

  // features
  mFeaturesGroupBox    = new QGroupBox(tr("Features"), this);
  mFeaturesVBoxLayout = new QVBoxLayout();

  mForestsCheckBox   = new QCheckBox(tr("Forests"));
  mForestsCheckBox->setToolTip(tr("Include the forests in the generated world."));
  mRoadsCheckBox     = new QCheckBox(tr("Roads"));
  mRoadsCheckBox->setToolTip(tr("Include the roads in the generated world."));
  mAreasCheckBox     = new QCheckBox(tr("Areas (water area, landuse, etc.)"));
  mAreasCheckBox->setToolTip(tr("Include the areas in the generated world. Areas are typicaly used to represent lake, farmland, etc."));
  mTreesCheckBox     = new QCheckBox(tr("Isolated trees"));
  mTreesCheckBox->setToolTip(tr("Include the isolated trees in the generated world."));
  mBarriersCheckBox  = new QCheckBox(tr("Barriers"));
  mBarriersCheckBox->setToolTip(tr("Include barriers (fence, wall, etc.) in the generated world."));
  mRiversCheckBox    = new QCheckBox(tr("Rivers"));
  mRiversCheckBox->setToolTip(tr("Include the rivers in the generated world."));
  mBuildingsCheckBox = new QCheckBox(tr("Buildings"));
  mBuildingsCheckBox->setToolTip(tr("Include the buildings in the generated world."));
  mEnableMultipolygonBuildings = new QCheckBox(tr("Buildings from multipolygon"));
  mEnableMultipolygonBuildings->setToolTip(tr("Generate buildings from multipolygons."));
  mEnable3DCheckBox = new QCheckBox(tr("Third dimension"));
  mEnable3DCheckBox->setToolTip(tr("Use an external service to retrieve elevation information\nand use an ElevationGrid for the ground (require an internet connexion)."));
  connect(mEnable3DCheckBox, &QPushButton::pressed, this, &OSMImportWindow::warnAboutThirdDimension);

  mForestsCheckBox->setChecked(true);
  mRoadsCheckBox->setChecked(true);
  mAreasCheckBox->setChecked(true);
  mTreesCheckBox->setChecked(true);
  mBarriersCheckBox->setChecked(true);
  mRiversCheckBox->setChecked(true);
  mBuildingsCheckBox->setChecked(true);
  mEnableMultipolygonBuildings->setChecked(true);
  mEnable3DCheckBox->setChecked(false);

  mFeaturesVBoxLayout->addWidget(mForestsCheckBox);
  mFeaturesVBoxLayout->addWidget(mRoadsCheckBox);
  mFeaturesVBoxLayout->addWidget(mAreasCheckBox);
  mFeaturesVBoxLayout->addWidget(mTreesCheckBox);
  mFeaturesVBoxLayout->addWidget(mBarriersCheckBox);
  mFeaturesVBoxLayout->addWidget(mRiversCheckBox);
  mFeaturesVBoxLayout->addWidget(mBuildingsCheckBox);
  mFeaturesVBoxLayout->addWidget(mEnableMultipolygonBuildings);
  mFeaturesVBoxLayout->addWidget(mEnable3DCheckBox);

  mFeaturesGroupBox->setLayout(mFeaturesVBoxLayout);
  mMainLayout->addWidget(mFeaturesGroupBox, 0, 1);

  // generate button
  mGeneratePushButton = new QPushButton(tr("Generate the world file"), this);
  mMainLayout->addWidget(mGeneratePushButton, 3, 0, 1, 2);
  connect(mGeneratePushButton, &QPushButton::pressed, this, &OSMImportWindow::generateWorld);

  // console
  mConsoleTextEdit = new QTextEdit(this);
  mConsoleTextEdit->setReadOnly(true);
  mConsoleTextEdit->setAcceptRichText(true);
  mMainLayout->addWidget(mConsoleTextEdit, 4, 0, 2, 2);

  // locate the script file
  QString executableDir = QFileInfo(QCoreApplication::applicationFilePath()).absoluteDir().absolutePath();
  if (!stringPath.isEmpty())
    mScriptFile = stringPath;
  else if (QFile::exists(executableDir + "/../importer.py"))
    mScriptFile = executableDir + "/../importer.py";
  else if (QFile::exists(qgetenv("WEBOTS_HOME") + "/projects/automobile/resources/OSM_importer/importer.py"))
    mScriptFile = qgetenv("WEBOTS_HOME") + "/projects/automobile/resources/OSM_importer/importer.py";
  else {
    mScriptFile = "";
    mConsoleTextEdit->append("<font color='red'>Importer script not found.</font>");
  }
}

OSMImportWindow::~OSMImportWindow() {
}

void OSMImportWindow::updateInputMethod() {
  if (mFileRadioButton->isChecked()) {
    mFileLineEdit->setEnabled(true);
    mFilePushButton->setEnabled(true);
  } else {
    mFileLineEdit->setEnabled(false);
    mFilePushButton->setEnabled(false);
  }

  if (mCoordinateRadioButton->isChecked()) {
    mMinLatLineEdit->setEnabled(true);
    mMaxLatLineEdit->setEnabled(true);
    mMinLongLineEdit->setEnabled(true);
    mMaxLongLineEdit->setEnabled(true);
  } else {
    mMinLatLineEdit->setEnabled(false);
    mMaxLatLineEdit->setEnabled(false);
    mMinLongLineEdit->setEnabled(false);
    mMaxLongLineEdit->setEnabled(false);
  }
}

void OSMImportWindow::selectInputFile() {
  QString inputFile = QFileDialog::getOpenFileName(this, tr("Choose an osm file"), QDir::currentPath());
  if (!inputFile.isEmpty())
    mFileLineEdit->setText(inputFile);
}

void OSMImportWindow::selectConfigurationFile() {
  QString configFile = QFileDialog::getOpenFileName(this, tr("Choose a configuration file"), QDir::currentPath(), "Config file (*.ini)");
  if (!configFile.isEmpty())
    mConfigLineEdit->setText(configFile);
}

void OSMImportWindow::warnAboutThirdDimension() {
  if (!mEnable3DCheckBox->isChecked()) {
    QMessageBox::warning(this, tr("Enabling third dimension"), tr("The third dimension feature is experimental.\nIt requires an internet connexion and can be very slow."));
    mEnable3DCheckBox->setChecked(true);
  }
}

void OSMImportWindow::generateWorld() {
  QString osmFile;
  mConsoleTextEdit->clear();
  mGeneratePushButton->setEnabled(false);

  if (mFileRadioButton->isChecked())
    osmFile = mFileLineEdit->text();
  else {
    mConsoleTextEdit->append("Acquiring the map (please wait).");
    QNetworkAccessManager *manager = new QNetworkAccessManager(this);
    mReply = manager->get(QNetworkRequest(QUrl("http://overpass-api.de/api/map?bbox=" +
                                              mMinLongLineEdit->text() + "," +
                                              mMinLatLineEdit->text() + "," +
                                              mMaxLongLineEdit->text() + "," +
                                              mMaxLatLineEdit->text())));
    mReply->ignoreSslErrors();
    connect(mReply, &QNetworkReply::finished, this, &OSMImportWindow::readReply, Qt::UniqueConnection);
    return;
  }
  launchImporter(osmFile);
}

void OSMImportWindow::launchImporter(QString osmFile) {
  QString outputfile = QFileDialog::getSaveFileName(this, tr("Chose a name for the generated world"), QDir::currentPath(), "Webots world (*.wbt)");
  if (outputfile.isEmpty()) {
    mGeneratePushButton->setEnabled(true);
    return;
  }

  if (!outputfile.endsWith(".wbt"))
    outputfile.append(".wbt");

  if (!QFile::exists(osmFile)) {
    mConsoleTextEdit->append("<font color='red'>OSM file does not exist!</font>");
    mGeneratePushButton->setEnabled(true);
    return;
  }

  // get path to the script
  if (!QFile::exists(mScriptFile)) {
    mConsoleTextEdit->append("<font color='red'>Importer script not present.</font>");
    mConsoleTextEdit->append(mScriptFile);
    mGeneratePushButton->setEnabled(true);
    return;
  }

  // construct argument list
  QStringList arguments;
  arguments << mScriptFile;
  arguments << ("--input=" + osmFile);
  arguments << ("--output=" + outputfile);
  if(!mForestsCheckBox->isChecked())
    arguments << "--no-forests";
  if(!mRoadsCheckBox->isChecked())
    arguments << "--no-roads";
  if(!mAreasCheckBox->isChecked())
    arguments << "--no-areas";
  if(!mTreesCheckBox->isChecked())
    arguments << "--no-trees";
  if(!mBarriersCheckBox->isChecked())
    arguments << "--no-barriers";
  if(!mRiversCheckBox->isChecked())
    arguments << "--no-rivers";
  if(!mBuildingsCheckBox->isChecked())
    arguments << "--no-buildings";
  if(!mEnableMultipolygonBuildings->isChecked())
    arguments << "--disable-multipolygon-buildings";
  if(mEnable3DCheckBox->isChecked())
    arguments << "--enable-3D";
  if (!mProjectionLineEdit->text().isEmpty())
    arguments << ("--projection=" + mProjectionLineEdit->text());
  if (!QFile::exists(mConfigLineEdit->text()) && !mConfigLineEdit->text().isEmpty()) {
    mConsoleTextEdit->append("<font color='blue'>Config file does not exist, using default one instead.</font>");
  } else if (QFile::exists(mConfigLineEdit->text()))
    arguments << ("--config-file=" + mConfigLineEdit->text());

  // create process
  mImporterProcess = new QProcess(this);
  mImporterProcess->setWorkingDirectory(qgetenv("WEBOTS_HOME") + QString("/projects/automobile/resources/OSM_importer"));
  connect(mImporterProcess, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(setProcessFinished(int, QProcess::ExitStatus)));
  connect(mImporterProcess, &QProcess::readyReadStandardOutput, this, &OSMImportWindow::readStandardOutput);
  connect(mImporterProcess, &QProcess::readyReadStandardError, this, &OSMImportWindow::readStandardError);
  mImporterProcess->start("python", arguments);
}

void OSMImportWindow::setProcessFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  if (exitStatus == QProcess::CrashExit)
    mConsoleTextEdit->append("<font color='red'>Generation of the world failed.</font>");
  disconnect(mImporterProcess, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(setProcessFinished(int, QProcess::ExitStatus)));
  disconnect(mImporterProcess, &QProcess::readyReadStandardOutput, this, &OSMImportWindow::readStandardOutput);
  disconnect(mImporterProcess, &QProcess::readyReadStandardError, this, &OSMImportWindow::readStandardError);
  delete mImporterProcess;
  mImporterProcess = NULL;
  mGeneratePushButton->setEnabled(true);
}

void OSMImportWindow::readStandardOutput() {
  mConsoleTextEdit->append(mImporterProcess->readAllStandardOutput());
}

void OSMImportWindow::readStandardError() {
  mConsoleTextEdit->append("<font color='red'>" + mImporterProcess->readAllStandardError() + "</font>");
}

void OSMImportWindow::readReply() {
  if (mReply->size() == 0) {
    mGeneratePushButton->setEnabled(true);
    mConsoleTextEdit->append("<font color='red'>No internet access.</font>");
    return;
  }
  QTemporaryDir dir;
  dir.setAutoRemove(false);
  QFile file(dir.path() + "/map.osm");
  file.open(QIODevice::WriteOnly);
  file.write(mReply->readAll());
  file.close();
  mReply->deleteLater();
  launchImporter(dir.path() + "/map.osm");
}
