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

#include "WbVelocityViewer.hpp"

#include "WbGuiRefreshOracle.hpp"
#include "WbNodeUtilities.hpp"
#include "WbSolid.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

WbVelocityViewer::WbVelocityViewer(QWidget *parent) :
  QWidget(parent),
  mSolid(NULL),
  mIsSelected(false),
  mRelativeToComboBox(new QComboBox(this)) {
  QVBoxLayout *vBoxLayout = new QVBoxLayout(this);

  // Relative to combo box
  mRelativeToComboBox->setMinimumHeight(mRelativeToComboBox->sizeHint().height());
  mRelativeToComboBox->setToolTip(tr("Select relatively to which solid the velocity should be measured"));
  vBoxLayout->addWidget(mRelativeToComboBox);
  connect(mRelativeToComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this,
          &WbVelocityViewer::updateRelativeTo);

  // Labels
  QGridLayout *labelLayout = new QGridLayout();
  labelLayout->addWidget(new QLabel(tr("Linear velocity:")), 0, 0);
  labelLayout->addWidget(new QLabel(tr("Linear velocity magnitude:")), 1, 0);
  labelLayout->addWidget(new QLabel(tr("Angular velocity:")), 2, 0);
  labelLayout->addWidget(new QLabel(tr("Angular velocity magnitude:")), 3, 0);

  mLinearVelocityLabels.resize(4);
  mAngularVelocityLabels.resize(4);
  for (int i = 0; i < 4; ++i) {
    mLinearVelocityLabels[i] = new QLabel(this);
    mAngularVelocityLabels[i] = new QLabel(this);
    mLinearVelocityLabels[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    mAngularVelocityLabels[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    if (i < 3) {
      labelLayout->addWidget(mLinearVelocityLabels[i], 0, i + 1, Qt::AlignVCenter | Qt::AlignLeft);
      labelLayout->addWidget(mAngularVelocityLabels[i], 2, i + 1, Qt::AlignVCenter | Qt::AlignLeft);
    } else {
      labelLayout->addWidget(mLinearVelocityLabels[i], 1, 1, Qt::AlignVCenter | Qt::AlignLeft);
      labelLayout->addWidget(mAngularVelocityLabels[i], 3, 1, Qt::AlignVCenter | Qt::AlignLeft);
    }
  }
  vBoxLayout->addLayout(labelLayout);
}

WbVelocityViewer::~WbVelocityViewer() {
  mSolid = NULL;
}

void WbVelocityViewer::clean() {
  if (mSolid)
    disconnect(mSolid, &WbSolid::destroyed, this, &WbVelocityViewer::clean);
  mSolid = NULL;
}

void WbVelocityViewer::stopUpdating() {
  if (mSolid)
    disconnect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbVelocityViewer::requestUpdate);
}

void WbVelocityViewer::show(WbSolid *solid) {
  if (mSolid)
    disconnect(mSolid, &WbSolid::destroyed, this, &WbVelocityViewer::clean);

  mSolid = solid;

  updateRelativeToComboBox();

  if (mSolid) {
    connect(mSolid, &WbSolid::destroyed, this, &WbVelocityViewer::clean, Qt::UniqueConnection);

    if (mIsSelected)
      connect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbVelocityViewer::requestUpdate,
              Qt::UniqueConnection);
  }
}

void WbVelocityViewer::requestUpdate() {
  if (WbGuiRefreshOracle::instance()->canRefreshNow())
    update();
}

void WbVelocityViewer::update() {
  if (mIsSelected && mSolid) {
    WbSolid *solid = mSolid;
    if (mRelativeToComboBox->currentIndex() == 0)
      solid = NULL;
    else {
      for (int i = 0; i < mRelativeToComboBox->currentIndex(); ++i)
        solid = solid->upperSolid();
    }
    WbVector3 linearVelocity = mSolid->relativeLinearVelocity(solid);
    WbVector3 angularVelocity = mSolid->relativeAngularVelocity(solid);
    for (int i = 0; i < 3; ++i) {
      mLinearVelocityLabels[i]->setText(WbPrecision::doubleToString(linearVelocity[i], WbPrecision::GUI_MEDIUM));
      mAngularVelocityLabels[i]->setText(WbPrecision::doubleToString(angularVelocity[i], WbPrecision::GUI_MEDIUM));
    }
    mLinearVelocityLabels[3]->setText(WbPrecision::doubleToString(linearVelocity.length(), WbPrecision::GUI_MEDIUM));
    mAngularVelocityLabels[3]->setText(WbPrecision::doubleToString(angularVelocity.length(), WbPrecision::GUI_MEDIUM));
    return;
  }

  for (int i = 0; i < mLinearVelocityLabels.size(); ++i)
    mLinearVelocityLabels[i]->clear();
  for (int i = 0; i < mAngularVelocityLabels.size(); ++i)
    mAngularVelocityLabels[i]->clear();
}

void WbVelocityViewer::updateRelativeTo(int index) {
  update();
}

void WbVelocityViewer::setSelected(bool selected) {
  mIsSelected = selected;
  triggerPhysicsUpdates();
}

void WbVelocityViewer::triggerPhysicsUpdates() {
  if (mSolid == NULL)
    return;

  if (mIsSelected) {
    connect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbVelocityViewer::requestUpdate,
            Qt::UniqueConnection);
    update();
  } else
    disconnect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbVelocityViewer::requestUpdate);
}

void WbVelocityViewer::updateRelativeToComboBox() {
  mRelativeToComboBox->clear();
  if (mSolid) {
    mRelativeToComboBox->insertItem(0, tr("Absolute"));
    int i = 0;
    WbSolid *solid = WbNodeUtilities::findUpperSolid(mSolid);
    while (solid) {
      ++i;
      if (solid->nodeModelName() == solid->fullName())
        mRelativeToComboBox->insertItem(i, tr("Relative to %1 (depth level %2)").arg(solid->fullName()).arg(i));
      else
        mRelativeToComboBox->insertItem(i, tr("Relative to %1").arg(solid->fullName()));
      solid = WbNodeUtilities::findUpperSolid(solid);
    }
  }
}
