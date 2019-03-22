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

const QString gShadowIntensityString = "shadowIntensity";
const QString gShadowMapSizeString = "shadowMapSize";
const QString gShadowFilterSizeString = "shadowFilterSize";
const QString gShadowCascadesString = "shadowCascades";
const QString gFrustumCullingString = "frustumCulling";

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

  mShadowCascadesEdit = createSpinBox(gShadowCascadesString);
  mShadowCascadesEdit->setRange(0, 10);
  mShadowCascadesSlider = createSlider(gShadowCascadesString);
  mShadowCascadesSlider->setRange(0, 10);

  mShadowFilterEdit = createSpinBox(gShadowFilterSizeString);
  mShadowFilterEdit->setRange(0, 64);
  mShadowFilterSlider = createSlider(gShadowFilterSizeString);
  mShadowFilterSlider->setRange(0, 32);

  mShadowIntensityEdit = new QDoubleSpinBox(this);
  mShadowIntensityEdit->setFixedWidth(50);
  mShadowIntensityEdit->setRange(0.0, 1.0);
  mShadowIntensityEdit->setButtonSymbols(QAbstractSpinBox::NoButtons);
  mShadowIntensityEdit->setProperty("parameterName", gShadowIntensityString);
  mShadowIntensitySlider = createSlider(gShadowIntensityString);
  mShadowIntensitySlider->setRange(0, 100);

  mFrustumCullingCheckBox = new QCheckBox(this);

  connect(mShadowMapSlider, &QSlider::valueChanged, this, &WbHtmlExportDialog::updateShadowEditValue);
  connect(mShadowCascadesSlider, &QSlider::valueChanged, this, &WbHtmlExportDialog::updateShadowEditValue);
  connect(mShadowFilterSlider, &QSlider::valueChanged, this, &WbHtmlExportDialog::updateShadowEditValue);
  connect(mShadowIntensitySlider, &QSlider::valueChanged, this, &WbHtmlExportDialog::updateShadowEditValue);
  connect(mShadowFilterEdit, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this,
          &WbHtmlExportDialog::updateShadowSliderValue);
  connect(mShadowCascadesEdit, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this,
          &WbHtmlExportDialog::updateShadowSliderValue);
  connect(mShadowIntensityEdit, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
          &WbHtmlExportDialog::updateShadowIntensitySliderValue);

  QGroupBox *shadowsGroup = new QGroupBox(tr("X3D Shadow Parameters"), this);
  QGridLayout *shadowsLayout = new QGridLayout(shadowsGroup);
  shadowsLayout->setColumnStretch(0, 0);
  shadowsLayout->setColumnStretch(1, 0);
  shadowsLayout->setColumnStretch(2, 1);
  shadowsLayout->setColumnStretch(3, 0);
  shadowsLayout->setColumnStretch(3, 0);
  shadowsLayout->addWidget(new QLabel(gShadowIntensityString + ":", this), 0, 0, Qt::AlignLeft);
  shadowsLayout->addWidget(new QLabel("<small>0</small>", this), 0, 1, Qt::AlignRight);
  shadowsLayout->addWidget(mShadowIntensitySlider, 0, 2);
  shadowsLayout->addWidget(new QLabel("<small>1</small>", this), 0, 3, Qt::AlignLeft);
  shadowsLayout->addWidget(mShadowIntensityEdit, 0, 4);
  shadowsLayout->addWidget(new QLabel(gShadowMapSizeString + ":", this), 1, 0, Qt::AlignLeft);
  shadowsLayout->addWidget(new QLabel("<small>32</small>", this), 1, 1, Qt::AlignRight);
  shadowsLayout->addWidget(mShadowMapSlider, 1, 2);
  shadowsLayout->addWidget(new QLabel("<small>4096</small>", this), 1, 3, Qt::AlignLeft);
  shadowsLayout->addWidget(mShadowMapLabel, 1, 4, Qt::AlignCenter);
  shadowsLayout->addWidget(new QLabel(gShadowFilterSizeString + ":", this), 2, 0, Qt::AlignLeft);
  shadowsLayout->addWidget(new QLabel("<small>0</small>", this), 2, 1, Qt::AlignRight);
  shadowsLayout->addWidget(mShadowFilterSlider, 2, 2);
  shadowsLayout->addWidget(new QLabel("<small>32</small>", this), 2, 3, Qt::AlignLeft);
  shadowsLayout->addWidget(mShadowFilterEdit, 2, 4);
  shadowsLayout->addWidget(new QLabel(gShadowCascadesString + ":", this), 3, 0, Qt::AlignLeft);
  shadowsLayout->addWidget(new QLabel("<small>0</small>", this), 3, 1, Qt::AlignRight);
  shadowsLayout->addWidget(mShadowCascadesSlider, 3, 2);
  shadowsLayout->addWidget(new QLabel("<small>10</small>", this), 3, 3, Qt::AlignLeft);
  shadowsLayout->addWidget(mShadowCascadesEdit, 3, 4);

  QGroupBox *renderingGroup = new QGroupBox(tr("X3D Rendering Parameters"), this);
  QGridLayout *renderingLayout = new QGridLayout(renderingGroup);
  renderingLayout->addWidget(new QLabel(gFrustumCullingString + ":", this), 0, 0, Qt::AlignLeft);
  renderingLayout->addWidget(mFrustumCullingCheckBox, 0, 1);

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
  mainLayout->addWidget(renderingGroup);
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

  QString filterValueString;
  if (parameters.contains(gShadowFilterSizeString))
    filterValueString = parameters.value(gShadowFilterSizeString);
  else
    filterValueString = WbLight::defaultX3dShadowsParameter(gShadowFilterSizeString);
  mShadowFilterSlider->setValue(filterValueString.toInt());

  QString intensityValueString;
  if (parameters.contains(gShadowIntensityString))
    intensityValueString = parameters.value(gShadowIntensityString);
  else
    intensityValueString = WbLight::defaultX3dShadowsParameter(gShadowIntensityString);
  mShadowIntensitySlider->setValue(intensityValueString.toDouble() * 100);

  QString cascadesValueString;
  if (parameters.contains(gShadowCascadesString))
    cascadesValueString = parameters.value(gShadowCascadesString);
  else
    cascadesValueString = WbLight::defaultX3dShadowsParameter(gShadowCascadesString);
  mShadowCascadesSlider->setValue(cascadesValueString.toInt());

  QString frustumCullingValueString = parameters.value(gFrustumCullingString);
  if (frustumCullingValueString.isEmpty())
    frustumCullingValueString = WbWorld::defaultX3dFrustumCullingParameter();
  mFrustumCullingCheckBox->setChecked(frustumCullingValueString == "true");
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

QSpinBox *WbHtmlExportDialog::createSpinBox(const QString &parameterName) {
  QSpinBox *spinBox = new QSpinBox(this);
  spinBox->setMaximumWidth(50);
  spinBox->setButtonSymbols(QAbstractSpinBox::NoButtons);
  spinBox->setProperty("parameterName", parameterName);
  return spinBox;
}

void WbHtmlExportDialog::accept() {
  // store x3d export preferences
  WbPerspective *perspective = WbWorld::instance()->perspective();
  perspective->setX3dExportParameter(gShadowMapSizeString, mShadowMapLabel->text());
  perspective->setX3dExportParameter(gShadowFilterSizeString, QString::number(mShadowFilterEdit->value()));
  perspective->setX3dExportParameter(gShadowIntensityString, QString::number(mShadowIntensityEdit->value()));
  perspective->setX3dExportParameter(gShadowCascadesString, QString::number(mShadowCascadesEdit->value()));
  perspective->setX3dExportParameter(gFrustumCullingString, convertBoolToString(mFrustumCullingCheckBox->isChecked()));
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
  else if (parameterName == gShadowFilterSizeString)
    mShadowFilterEdit->setValue(value);
  else if (parameterName == gShadowCascadesString)
    mShadowCascadesEdit->setValue(value);
  else if (parameterName == gShadowIntensityString)
    mShadowIntensityEdit->setValue(((double)value) / 100.0);
  senderObject->blockSignals(false);
}

void WbHtmlExportDialog::updateShadowSliderValue(int value) {
  QObject *senderObject = QObject::sender();
  const QString &parameterName = senderObject->property("parameterName").toString();
  senderObject->blockSignals(true);
  if (parameterName == gShadowFilterSizeString)
    mShadowFilterSlider->setValue(value);
  else if (parameterName == gShadowCascadesString)
    mShadowCascadesSlider->setValue(value);
  senderObject->blockSignals(false);
}

void WbHtmlExportDialog::updateShadowIntensitySliderValue(double value) {
  mShadowIntensitySlider->blockSignals(true);
  mShadowIntensitySlider->setValue(value * 100);
  mShadowIntensitySlider->blockSignals(false);
}

void WbHtmlExportDialog::browse() {
  const QString &fileName = QFileDialog::getSaveFileName(this, mTitle, mFileLineEdit->text(), tr("HTML Files (*.html *.HTML)"));
  if (!fileName.isEmpty())
    mFileLineEdit->setText(fileName);
}

QString WbHtmlExportDialog::fileName() {
  return mFileLineEdit->text();
}
