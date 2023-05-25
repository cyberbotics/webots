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

#include "WbPhysicsViewer.hpp"

#include "WbGuiRefreshOracle.hpp"
#include "WbSolid.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>

WbPhysicsViewer::WbPhysicsViewer(QWidget *parent) :
  QWidget(parent),
  mSolid(NULL),
  mIsSelected(false),
  mIncludingExcludingDescendants(new QComboBox(this)),
  mRelativeAbsolute(new QComboBox(this)),
  mMassLabel(new QLabel(this)),
  mDensityLabel(new QLabel(this)),
  mInertiaMatrixMainLabel(new QLabel(tr("inertia matrix:"), this)) {
  void (QComboBox::*indexChangedSignal)(int) = &QComboBox::currentIndexChanged;
  QGridLayout *gridLayout = new QGridLayout(this);

  mIncludingExcludingDescendants->setMinimumHeight(mIncludingExcludingDescendants->sizeHint().height());
  mIncludingExcludingDescendants->insertItem(0, tr("excluding descendants"));
  mIncludingExcludingDescendants->insertItem(1, tr("including descendants"));
  mIncludingExcludingDescendants->setToolTip(tr("Display mass properties of the selected solid only"));
  gridLayout->addWidget(mIncludingExcludingDescendants, 0, 2, 1, 3, Qt::AlignVCenter);
  connect(mIncludingExcludingDescendants, indexChangedSignal, this, &WbPhysicsViewer::updateIncludingExcludingDescendantsData);

  // Mass
  QLabel *label = new QLabel(tr("mass:"), this);
  mMassLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  gridLayout->addWidget(label, 1, 0, 1, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mMassLabel, 1, 2, 1, 3, Qt::AlignVCenter);

  // Density
  label = new QLabel(tr("density:"), this);
  mDensityLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
  gridLayout->addWidget(label, 2, 0, 1, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mDensityLabel, 2, 2, 1, 3, Qt::AlignVCenter);

  // Center of mass
  label = new QLabel("CoM:", this);
  label->setToolTip("Solid's center of mass");
  QLabel *valueLabel = NULL;
  for (int i = 0; i < 3; ++i) {
    valueLabel = new QLabel(this);
    valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    mCenterOfMassLabel.append(valueLabel);
  }

  mRelativeAbsolute->setMinimumHeight(mRelativeAbsolute->sizeHint().height());
  mRelativeAbsolute->setToolTip(tr("Coordinates relative to selected's solid frame"));

  connect(mRelativeAbsolute, indexChangedSignal, this, &WbPhysicsViewer::updateCoordinatesSystem);

  gridLayout->addWidget(label, 3, 0, Qt::AlignVCenter);
  gridLayout->addWidget(mRelativeAbsolute, 3, 1, Qt::AlignVCenter);
  gridLayout->addWidget(mCenterOfMassLabel[0], 3, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mCenterOfMassLabel[1], 3, 3, Qt::AlignVCenter);
  gridLayout->addWidget(mCenterOfMassLabel[2], 3, 4, Qt::AlignVCenter);

  // Inertia matrix
  mInertiaMatrixMainLabel->setToolTip("Inertia matrix expressed within the solid frame centered at CoM");
  for (int i = 0; i < 9; ++i) {
    valueLabel = new QLabel(this);
    valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    mInertiaMatrixLabel.append(valueLabel);
  }
  gridLayout->addWidget(mInertiaMatrixLabel[0], 4, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[1], 4, 3, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[2], 4, 4, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixMainLabel, 5, 0, 1, 2, Qt::AlignTop);
  gridLayout->addWidget(mInertiaMatrixLabel[3], 5, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[4], 5, 3, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[5], 5, 4, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[6], 6, 2, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[7], 6, 3, Qt::AlignVCenter);
  gridLayout->addWidget(mInertiaMatrixLabel[8], 6, 4, Qt::AlignVCenter);

  // Set labels to be modified by the main stylesheet
  mInertiaMatrixLabel[0]->setObjectName("inertiaMatrixDiagonalCoefficientLabel");
  mInertiaMatrixLabel[4]->setObjectName("inertiaMatrixDiagonalCoefficientLabel");
  mInertiaMatrixLabel[8]->setObjectName("inertiaMatrixDiagonalCoefficientLabel");
  mInertiaMatrixLabel[1]->setObjectName("inertiaMatrixPrimaryCoefficientLabel");
  mInertiaMatrixLabel[2]->setObjectName("inertiaMatrixPrimaryCoefficientLabel");
  mInertiaMatrixLabel[5]->setObjectName("inertiaMatrixPrimaryCoefficientLabel");
  mInertiaMatrixLabel[3]->setObjectName("inertiaMatrixSecondaryCoefficientLabel");
  mInertiaMatrixLabel[6]->setObjectName("inertiaMatrixSecondaryCoefficientLabel");
  mInertiaMatrixLabel[7]->setObjectName("inertiaMatrixSecondaryCoefficientLabel");

  gridLayout->setColumnStretch(0, 0);
  gridLayout->setColumnStretch(1, 0);
  gridLayout->setColumnStretch(2, 1);
  gridLayout->setColumnStretch(3, 1);
  gridLayout->setColumnStretch(4, 1);

  mRelativeAbsolute->insertItem(0, tr("relative"));
  mRelativeAbsolute->insertItem(1, tr("absolute"));
}

WbPhysicsViewer::~WbPhysicsViewer() {
  mSolid = NULL;
}

void WbPhysicsViewer::clean() {
  mSolid = NULL;
}

void WbPhysicsViewer::stopUpdating() {
  if (mSolid) {
    disconnect(mSolid, &WbSolid::massPropertiesChanged, this, &WbPhysicsViewer::update);
    disconnect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this,
               &WbPhysicsViewer::updateCenterOfMass);
    disconnect(mSolid, &WbSolid::positionChangedArtificially, this, &WbPhysicsViewer::updateCenterOfMass);
  }
}

void WbPhysicsViewer::show(WbSolid *solid) {
  mSolid = solid;

  if (mSolid)
    connect(mSolid, &WbSolid::destroyed, this, &WbPhysicsViewer::clean, Qt::UniqueConnection);

  if (mSolid && mIsSelected) {
    connect(mSolid, &WbSolid::massPropertiesChanged, this, &WbPhysicsViewer::update, Qt::UniqueConnection);
    connect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbPhysicsViewer::updateCenterOfMass,
            Qt::UniqueConnection);
    connect(mSolid, &WbSolid::positionChangedArtificially, this, &WbPhysicsViewer::updateCenterOfMass, Qt::UniqueConnection);
  }
}

bool WbPhysicsViewer::update() {
  bool enabled = mSolid && (mSolid->globalMass() > 0.0);
  if (mIsSelected && enabled && mSolid->areOdeObjectsCreated()) {
    updateMass();
    updateDensity();
    updateCenterOfMass();
    updateInertiaMatrix();
    return enabled;
  }

  mMassLabel->clear();
  mDensityLabel->clear();
  for (int i = 0; i < 9; ++i)
    mInertiaMatrixLabel[i]->clear();
  for (int j = 0; j < 3; ++j)
    mCenterOfMassLabel[j]->clear();
  return enabled;
}

void WbPhysicsViewer::updateMass() {
  const double lm = mSolid->mass();
  const double gm = mSolid->globalMass();
  if (gm > 0.0) {
    const double currentMass = mIncludingExcludingDescendants->currentIndex() == LOCAL ? lm : gm;
    mMassLabel->setText(QString("%1 kg").arg(WbPrecision::doubleToString(currentMass, WbPrecision::GUI_MEDIUM)));
  } else
    mMassLabel->clear();
}

void WbPhysicsViewer::updateDensity() {
  const double d = mSolid->density();
  const double ad = mSolid->averageDensity();
  if (ad >= 0.0) {
    const double currentDensity = mIncludingExcludingDescendants->currentIndex() == LOCAL ? d : ad;
    mDensityLabel->setText(QString("%1 kg/m^3").arg(WbPrecision::doubleToString(currentDensity, WbPrecision::GUI_MEDIUM)));
  } else
    mDensityLabel->clear();
}

void WbPhysicsViewer::updateCenterOfMass() {
  bool skipUpdate = WbGuiRefreshOracle::instance()->canRefreshNow() == false;
  skipUpdate |= !mIsSelected || (!mSolid) || mSolid->areOdeObjectsCreated() == false;
  if (skipUpdate)
    return;

  mSolid->updateGlobalCenterOfMass();
  mCenterOfMass[LOCAL][RELATIVE_POSITION] = mSolid->centerOfMass();
  const WbMatrix4 &m = mSolid->matrix();
  mCenterOfMass[LOCAL][ABSOLUTE_POSITION] = m * mSolid->centerOfMass();
  mCenterOfMass[GLOBAL][ABSOLUTE_POSITION] = mSolid->globalCenterOfMass();
  mCenterOfMass[GLOBAL][RELATIVE_POSITION] = m.pseudoInversed(mCenterOfMass[GLOBAL][ABSOLUTE_POSITION]);
  if (mSolid->globalMass() != 0.0) {
    const WbVector3 &com = mCenterOfMass[mIncludingExcludingDescendants->currentIndex()][mRelativeAbsolute->currentIndex()];
    for (int i = 0; i < 3; ++i)
      mCenterOfMassLabel[i]->setText(WbPrecision::doubleToString(com[i], WbPrecision::GUI_MEDIUM));
  } else {
    for (int i = 0; i < 3; ++i)
      mCenterOfMassLabel[i]->clear();
  }
}

void WbPhysicsViewer::updateInertiaMatrix() {
  if (mSolid->mass() != 0.0 && (mIncludingExcludingDescendants->currentIndex() == LOCAL)) {
    mInertiaMatrixMainLabel->setText(tr("Inertia matrix:"));
    const double *const I = mSolid->inertiaMatrix();
    for (int i = 0; i < 3; ++i) {
      mInertiaMatrixLabel[i]->setText(WbPrecision::doubleToString(I[i], WbPrecision::GUI_MEDIUM));
      mInertiaMatrixLabel[i + 3]->setText(WbPrecision::doubleToString(I[i + 4], WbPrecision::GUI_MEDIUM));
      mInertiaMatrixLabel[i + 6]->setText(WbPrecision::doubleToString(I[i + 8], WbPrecision::GUI_MEDIUM));
    }
  } else {
    for (int i = 0; i < 9; ++i)
      mInertiaMatrixLabel[i]->clear();
    mInertiaMatrixMainLabel->clear();
  }
}

void WbPhysicsViewer::setSelected(bool selected) {
  mIsSelected = selected;
  triggerPhysicsUpdates();
}

void WbPhysicsViewer::triggerPhysicsUpdates() {
  if (mSolid == NULL)
    return;

  if (mIsSelected) {
    connect(mSolid, &WbSolid::massPropertiesChanged, this, &WbPhysicsViewer::update, Qt::UniqueConnection);
    connect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbPhysicsViewer::updateCenterOfMass,
            Qt::UniqueConnection);
    connect(mSolid, &WbSolid::positionChangedArtificially, this, &WbPhysicsViewer::updateCenterOfMass, Qt::UniqueConnection);
    update();
  } else {
    disconnect(mSolid, &WbSolid::massPropertiesChanged, this, &WbPhysicsViewer::update);
    disconnect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this,
               &WbPhysicsViewer::updateCenterOfMass);
    disconnect(mSolid, &WbSolid::positionChangedArtificially, this, &WbPhysicsViewer::updateCenterOfMass);
  }
}

void WbPhysicsViewer::updateCoordinatesSystem() {
  if (mRelativeAbsolute->currentIndex() == RELATIVE_POSITION)
    mRelativeAbsolute->setToolTip(tr("Coordinates with respect to selected's solid frame"));
  else
    mRelativeAbsolute->setToolTip(tr("Coordinates with respect to world's frame"));

  updateCenterOfMass();
}

void WbPhysicsViewer::updateIncludingExcludingDescendantsData() {
  if (mIncludingExcludingDescendants->currentIndex() == LOCAL)
    mIncludingExcludingDescendants->setToolTip(tr("Display mass properties of the selected solid only"));
  else
    mIncludingExcludingDescendants->setToolTip(
      tr("Display averaged mass properties of the selected solid augmented by its descendants"));

  update();
}
