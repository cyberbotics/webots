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

#include "WbPositionViewer.hpp"

#include "WbGuiRefreshOracle.hpp"
#include "WbSolid.hpp"
#include "WbTransform.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

WbPositionViewer::WbPositionViewer(QWidget *parent) :
  QWidget(parent),
  mTransform(NULL),
  mIsSelected(false),
  mRelativeToComboBox(new QComboBox(this)) {
  QVBoxLayout *vBoxLayout = new QVBoxLayout(this);

  // Relative to combo box
  mRelativeToComboBox->setMinimumHeight(mRelativeToComboBox->sizeHint().height());
  mRelativeToComboBox->setToolTip(tr("Select relatively to which solid the position should be measured"));
  vBoxLayout->addWidget(mRelativeToComboBox);
  connect(mRelativeToComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this,
          &WbPositionViewer::updateRelativeTo);

  // Labels
  QGridLayout *labelLayout = new QGridLayout();
  labelLayout->addWidget(new QLabel(tr("Position:")), 0, 0);
  labelLayout->addWidget(new QLabel(tr("Rotation:")), 1, 0);

  mPositionLabels.resize(3);
  for (int i = 0; i < mPositionLabels.size(); ++i) {
    mPositionLabels[i] = new QLabel(this);
    mPositionLabels[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    labelLayout->addWidget(mPositionLabels[i], 0, i + 1, Qt::AlignVCenter | Qt::AlignLeft);
  }
  mRotationLabels.resize(4);
  for (int i = 0; i < mRotationLabels.size(); ++i) {
    mRotationLabels[i] = new QLabel(this);
    mRotationLabels[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    labelLayout->addWidget(mRotationLabels[i], 1, i + 1, Qt::AlignVCenter | Qt::AlignLeft);
  }
  vBoxLayout->addLayout(labelLayout);
}

WbPositionViewer::~WbPositionViewer() {
  mTransform = NULL;
}

void WbPositionViewer::clean() {
  if (mTransform)
    disconnect(mTransform, &WbTransform::destroyed, this, &WbPositionViewer::clean);
  mTransform = NULL;
}

void WbPositionViewer::stopUpdating() {
  if (mTransform) {
    disconnect(mTransform->translationFieldValue(), &WbSFVector3::changed, this, &WbPositionViewer::update);
    disconnect(mTransform->rotationFieldValue(), &WbSFRotation::changed, this, &WbPositionViewer::update);
    disconnect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbPositionViewer::requestUpdate);
  }
}

void WbPositionViewer::show(WbTransform *transform) {
  if (mTransform)
    disconnect(mTransform, &WbTransform::destroyed, this, &WbPositionViewer::clean);

  mTransform = transform;

  updateRelativeToComboBox();

  if (mTransform) {
    connect(mTransform, &WbTransform::destroyed, this, &WbPositionViewer::clean, Qt::UniqueConnection);

    if (mIsSelected) {
      connect(mTransform->translationFieldValue(), &WbSFVector3::changed, this, &WbPositionViewer::update,
              Qt::UniqueConnection);
      connect(mTransform->rotationFieldValue(), &WbSFRotation::changed, this, &WbPositionViewer::update, Qt::UniqueConnection);
      connect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbPositionViewer::requestUpdate,
              Qt::UniqueConnection);
    }
  }
}

void WbPositionViewer::requestUpdate() {
  if (WbGuiRefreshOracle::instance()->canRefreshNow())
    update();
}

void WbPositionViewer::update() {
  if (mIsSelected && mTransform) {
    WbVector3 position(0, 0, 0);
    WbRotation rotation(0, 0, 0, 0);
    if (mRelativeToComboBox->currentIndex() == 0) {
      position = mTransform->position();
      rotation = WbRotation(mTransform->rotationMatrix());
      rotation.normalize();
    } else {
      WbTransform *transform = mTransform;
      for (int i = 0; i < mRelativeToComboBox->currentIndex(); ++i)
        transform = transform->upperTransform();
      position = mTransform->position() - transform->position();
      position = position * WbMatrix3(transform->rotation().toQuaternion());
      WbRotation currentRotation = WbRotation(mTransform->rotationMatrix());
      WbRotation referenceRotation = WbRotation(transform->rotationMatrix());
      currentRotation.normalize();
      referenceRotation.normalize();
      if (currentRotation == referenceRotation)  // if there is no orientation difference, return 0 0 1 0
        rotation = WbRotation(0, 0, 1, 0);
      else
        rotation = WbRotation(currentRotation.toQuaternion() * referenceRotation.toQuaternion().conjugated());
    }
    for (int i = 0; i < mPositionLabels.size(); ++i)
      mPositionLabels[i]->setText(WbPrecision::doubleToString(position[i], WbPrecision::GUI_MEDIUM));
    for (int i = 0; i < mRotationLabels.size(); ++i)
      mRotationLabels[i]->setText(WbPrecision::doubleToString(rotation[i], WbPrecision::GUI_MEDIUM));
    return;
  }

  for (int i = 0; i < mPositionLabels.size(); ++i)
    mPositionLabels[i]->clear();
  for (int i = 0; i < mRotationLabels.size(); ++i)
    mRotationLabels[i]->clear();
}

void WbPositionViewer::updateRelativeTo(int index) {
  update();
}

void WbPositionViewer::setSelected(bool selected) {
  mIsSelected = selected;
  triggerPhysicsUpdates();
}

void WbPositionViewer::triggerPhysicsUpdates() {
  if (mTransform == NULL)
    return;

  if (mIsSelected) {
    connect(mTransform->translationFieldValue(), &WbSFVector3::changed, this, &WbPositionViewer::update, Qt::UniqueConnection);
    connect(mTransform->rotationFieldValue(), &WbSFRotation::changed, this, &WbPositionViewer::update, Qt::UniqueConnection);
    connect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbPositionViewer::requestUpdate,
            Qt::UniqueConnection);
    update();
  } else {
    disconnect(mTransform->translationFieldValue(), &WbSFVector3::changed, this, &WbPositionViewer::update);
    disconnect(mTransform->rotationFieldValue(), &WbSFRotation::changed, this, &WbPositionViewer::update);
    disconnect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbPositionViewer::requestUpdate);
  }
}

void WbPositionViewer::updateRelativeToComboBox() {
  mRelativeToComboBox->clear();
  if (mTransform) {
    mRelativeToComboBox->insertItem(0, tr("Absolute"));
    int i = 0;
    WbTransform *transform = mTransform->upperTransform();
    while (transform) {
      ++i;
      if (transform->nodeModelName() == transform->fullName())
        mRelativeToComboBox->insertItem(i, tr("Relative to %1 (depth level %2)").arg(transform->fullName()).arg(i));
      else
        mRelativeToComboBox->insertItem(i, tr("Relative to %1").arg(transform->fullName()));
      transform = transform->upperTransform();
    }
  }
}
