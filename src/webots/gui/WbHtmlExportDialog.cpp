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

#include "WbHtmlExportDialog.hpp"
#include "WbLight.hpp"
#include "WbMathsUtilities.hpp"
#include "WbPerspective.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbWorld.hpp"

#include <QtCore/QFileInfo>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

#include <cassert>
#include <cmath>

const QString gShadowMapSizeString = "shadowMapSize";
const QString gShadowRadiusString = "shadowRadius";
const QString gShadowBiasString = "shadowBias";

WbHtmlExportDialog::WbHtmlExportDialog(const QString &title, const QString &worldFilePath, QWidget *parent) :
  QDialog(parent),
  mTitle(title),
  mWorldFilePath(worldFilePath) {
  setWindowTitle(title);
  setModal(true);

  mShadowMapLabel = new QLabel(this);
  mShadowMapSlider = createSlider(gShadowMapSizeString);
  mShadowMapSlider->setRange(1, 8);
  mShadowMapSlider->setTickInterval(1);
  mShadowMapSlider->setTickPosition(QSlider::TicksBelow);

  mShadowBiasEdit = createSpinBox(gShadowBiasString);
  mShadowBiasEdit->setRange(0.0, 0.1);
  mShadowBiasEdit->setDecimals(3);
  mShadowBiasSlider = createSlider(gShadowBiasString);
  mShadowBiasSlider->setRange(0, 10);

  mShadowRadiusEdit = createSpinBox(gShadowRadiusString);
  mShadowRadiusEdit->setRange(0.0, 10.0);
  mShadowRadiusEdit->setDecimals(2);
  mShadowRadiusSlider = createSlider(gShadowRadiusString);
  mShadowRadiusSlider->setRange(0.0, 10.0);

  connect(mShadowMapSlider, &QSlider::valueChanged, this, &WbHtmlExportDialog::updateShadowEditValue);
  connect(mShadowBiasSlider, &QSlider::valueChanged, this, &WbHtmlExportDialog::updateShadowEditValue);
  connect(mShadowRadiusSlider, &QSlider::valueChanged, this, &WbHtmlExportDialog::updateShadowEditValue);
  connect(mShadowRadiusEdit, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
          &WbHtmlExportDialog::updateShadowSliderValue);
  connect(mShadowBiasEdit, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
          &WbHtmlExportDialog::updateShadowSliderValue);

  QGroupBox *shadowsGroup = new QGroupBox(tr("X3D Shadow Parameters"), this);
  QGridLayout *shadowsLayout = new QGridLayout(shadowsGroup);
  shadowsLayout->setColumnStretch(0, 0);
  shadowsLayout->setColumnStretch(1, 0);
  shadowsLayout->setColumnStretch(2, 1);
  shadowsLayout->setColumnStretch(3, 0);
  shadowsLayout->addWidget(new QLabel(gShadowMapSizeString + ":", this), 0, 0, Qt::AlignLeft);
  shadowsLayout->addWidget(new QLabel("<small>32</small>", this), 0, 1, Qt::AlignRight);
  shadowsLayout->addWidget(mShadowMapSlider, 0, 2);
  shadowsLayout->addWidget(new QLabel("<small>4096</small>", this), 0, 3, Qt::AlignLeft);
  shadowsLayout->addWidget(mShadowMapLabel, 0, 4, Qt::AlignCenter);
  shadowsLayout->addWidget(new QLabel(gShadowRadiusString + ":", this), 2, 0, Qt::AlignLeft);
  shadowsLayout->addWidget(new QLabel("<small>0</small>", this), 2, 1, Qt::AlignRight);
  shadowsLayout->addWidget(mShadowRadiusSlider, 2, 2);
  shadowsLayout->addWidget(new QLabel("<small>10</small>", this), 2, 3, Qt::AlignLeft);
  shadowsLayout->addWidget(mShadowRadiusEdit, 2, 4);
  shadowsLayout->addWidget(new QLabel(gShadowBiasString + ":", this), 3, 0, Qt::AlignLeft);
  shadowsLayout->addWidget(new QLabel("<small>0</small>", this), 3, 1, Qt::AlignRight);
  shadowsLayout->addWidget(mShadowBiasSlider, 3, 2);
  shadowsLayout->addWidget(new QLabel("<small>0.1</small>", this), 3, 3, Qt::AlignLeft);
  shadowsLayout->addWidget(mShadowBiasEdit, 3, 4);

  QGroupBox *fileGroup = new QGroupBox(this);
  mFileLineEdit = new QLineEdit(this);
  QString proposedFilename;
  for (int i = 0; i < 100; ++i) {
    QString suffix = i == 0 ? "" : QString("_%1").arg(i);
    proposedFilename =
      WbPreferences::instance()->value("Directories/www").toString() + QFileInfo(mWorldFilePath).baseName() + suffix + ".html";
    if (!QFileInfo::exists(proposedFilename))
      break;
  }
  mFileLineEdit->setText(WbProject::computeBestPathForSaveAs(proposedFilename));
  mFileLineEdit->setReadOnly(true);
  QPushButton *browseButton = new QPushButton("...", this);
  browseButton->setMaximumWidth(30);
  connect(browseButton, &QPushButton::clicked, this, &WbHtmlExportDialog::browse);

  QHBoxLayout *fileLayout = new QHBoxLayout(fileGroup);
  fileLayout->addWidget(new QLabel(tr("File name:"), this), 0, Qt::AlignLeft);
  fileLayout->addWidget(mFileLineEdit, 1);
  fileLayout->addWidget(browseButton, 0, Qt::AlignLeft);

  mButtonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this);
  connect(mButtonBox, &QDialogButtonBox::accepted, this, &WbHtmlExportDialog::accept);
  connect(mButtonBox, &QDialogButtonBox::rejected, this, &WbHtmlExportDialog::reject);

  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(shadowsGroup);
  mainLayout->addWidget(fileGroup);
  mainLayout->addWidget(mButtonBox);

  // read intial export values
  const QHash<QString, QString> parameters = WbWorld::instance()->perspective()->x3dExportParameters();
  QString mapValueString;
  if (parameters.contains(gShadowMapSizeString))
    mapValueString = parameters.value(gShadowMapSizeString);
  else
    mapValueString = WbLight::defaultX3dShadowsParameter(gShadowMapSizeString);
  mShadowMapSlider->setValue(computeShadowMapSliderIndex(mapValueString.toInt()));

  QString radiusValueString;
  if (parameters.contains(gShadowRadiusString))
    radiusValueString = parameters.value(gShadowRadiusString);
  else
    radiusValueString = WbLight::defaultX3dShadowsParameter(gShadowRadiusString);
  mShadowRadiusSlider->blockSignals(true);
  mShadowRadiusEdit->setValue(radiusValueString.toDouble());
  mShadowRadiusSlider->blockSignals(false);

  QString biasString;
  if (parameters.contains(gShadowBiasString))
    biasString = parameters.value(gShadowBiasString);
  else
    biasString = WbLight::defaultX3dShadowsParameter(gShadowBiasString);
  mShadowBiasSlider->blockSignals(true);
  mShadowBiasEdit->setValue(biasString.toDouble());
  mShadowBiasSlider->blockSignals(false);
}

WbHtmlExportDialog::~WbHtmlExportDialog() {
}

QSlider *WbHtmlExportDialog::createSlider(const QString &parameterName) {
  QSlider *slider = new QSlider(Qt::Horizontal, this);
  slider->setTickInterval(0);
  slider->setTracking(true);
  slider->setMinimumWidth(200);
  slider->setProperty("parameterName", parameterName);
  return slider;
}

QDoubleSpinBox *WbHtmlExportDialog::createSpinBox(const QString &parameterName) {
  QDoubleSpinBox *spinBox = new QDoubleSpinBox(this);
  spinBox->setMaximumWidth(50);
  spinBox->setButtonSymbols(QAbstractSpinBox::NoButtons);
  spinBox->setProperty("parameterName", parameterName);
  return spinBox;
}

void WbHtmlExportDialog::accept() {
  // store x3d export preferences
  WbPerspective *perspective = WbWorld::instance()->perspective();
  perspective->setX3dExportParameter(gShadowMapSizeString, mShadowMapLabel->text());
  perspective->setX3dExportParameter(gShadowRadiusString, QString::number(mShadowRadiusEdit->value()));
  perspective->setX3dExportParameter(gShadowBiasString, QString::number(mShadowBiasEdit->value()));
  perspective->save();
  QDialog::accept();
}

int WbHtmlExportDialog::computeShadowMapSliderIndex(int value) {
  int indexValue = 32;
  for (int i = 1; i <= 8; ++i) {
    if (indexValue == value)
      return i;
    indexValue *= 2;
  }
  assert(false);
  return 0;
}

void WbHtmlExportDialog::updateShadowEditValue(int value) {
  QObject *senderObject = QObject::sender();
  const QString &parameterName = senderObject->property("parameterName").toString();
  senderObject->blockSignals(true);
  if (parameterName == gShadowMapSizeString)
    mShadowMapLabel->setText(QString::number(std::pow(2, value + 4)));
  else if (parameterName == gShadowRadiusString)
    mShadowRadiusEdit->setValue(value);
  else if (parameterName == gShadowBiasString)
    mShadowBiasEdit->setValue(0.01 * value);
  senderObject->blockSignals(false);
}

void WbHtmlExportDialog::updateShadowSliderValue(double value) {
  QObject *senderObject = QObject::sender();
  const QString &parameterName = senderObject->property("parameterName").toString();
  senderObject->blockSignals(true);
  if (parameterName == gShadowRadiusString)
    mShadowRadiusSlider->setValue(value);
  else if (parameterName == gShadowBiasString)
    mShadowBiasSlider->setValue(100.0 * value);
  senderObject->blockSignals(false);
}

void WbHtmlExportDialog::browse() {
  const QString &fileName = QFileDialog::getSaveFileName(this, mTitle, mFileLineEdit->text(), tr("HTML Files (*.html *.HTML)"));
  if (!fileName.isEmpty())
    mFileLineEdit->setText(fileName);
}

QString WbHtmlExportDialog::fileName() {
  return mFileLineEdit->text();
}
