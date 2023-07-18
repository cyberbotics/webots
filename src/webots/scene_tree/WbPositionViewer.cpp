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

#include "WbPositionViewer.hpp"

#include "WbGuiRefreshOracle.hpp"
#include "WbPose.hpp"
#include "WbSolid.hpp"
#include "WbTransform.hpp"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

WbPositionViewer::WbPositionViewer(QWidget *parent) :
  QWidget(parent),
  mPose(NULL),
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
  mScaleTitleLabel = new QLabel(this);
  QGridLayout *labelLayout = new QGridLayout();
  labelLayout->addWidget(new QLabel(tr("Position:")), 0, 0);
  labelLayout->addWidget(new QLabel(tr("Rotation:")), 1, 0);
  labelLayout->addWidget(mScaleTitleLabel, 2, 0);

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
  mScaleLabels.resize(3);
  for (int i = 0; i < mScaleLabels.size(); ++i) {
    mScaleLabels[i] = new QLabel(this);
    mScaleLabels[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    labelLayout->addWidget(mScaleLabels[i], 2, i + 1, Qt::AlignVCenter | Qt::AlignLeft);
  }
  vBoxLayout->addLayout(labelLayout);
}

WbPositionViewer::~WbPositionViewer() {
  mPose = NULL;
}

void WbPositionViewer::clean() {
  if (mPose)
    disconnect(mPose, &WbPose::destroyed, this, &WbPositionViewer::clean);
  mPose = NULL;
}

void WbPositionViewer::stopUpdating() {
  if (mPose) {
    disconnect(mPose->translationFieldValue(), &WbSFVector3::changed, this, &WbPositionViewer::update);
    disconnect(mPose->rotationFieldValue(), &WbSFRotation::changed, this, &WbPositionViewer::update);
    disconnect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbPositionViewer::requestUpdate);
  }
}

void WbPositionViewer::show(WbPose *pose) {
  if (mPose)
    disconnect(mPose, &WbPose::destroyed, this, &WbPositionViewer::clean);

  mPose = pose;

  updateRelativeToComboBox();

  if (mPose) {
    connect(mPose, &WbPose::destroyed, this, &WbPositionViewer::clean, Qt::UniqueConnection);

    if (mIsSelected) {
      connect(mPose->translationFieldValue(), &WbSFVector3::changed, this, &WbPositionViewer::update, Qt::UniqueConnection);
      connect(mPose->rotationFieldValue(), &WbSFRotation::changed, this, &WbPositionViewer::update, Qt::UniqueConnection);
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
  if (mIsSelected && mPose) {
    WbVector3 position(mPose->position());
    WbVector3 scale;
    const WbTransform *transform = dynamic_cast<const WbTransform *>(mPose);
    if (transform)
      scale = transform->absoluteScale();
    else {
      transform = mPose->upperTransform();
      scale = transform ? transform->absoluteScale() : WbVector3(1.0, 1.0, 1.0);
    }

    WbRotation rotation;
    if (mRelativeToComboBox->currentIndex() == 0)
      rotation.fromMatrix3(mPose->rotationMatrix());
    else {
      const WbPose *pose = mPose;
      WbQuaternion q;
      for (int i = 0; i < mRelativeToComboBox->currentIndex(); ++i) {
        assert(pose);
        q = pose->relativeQuaternion() * q;
        pose = pose->upperPose();
      }
      // compute relative rotation
      q.normalize();
      rotation.fromQuaternion(q);

      // compute relative scale
      WbVector3 otherAbsoluteScale;
      transform = dynamic_cast<const WbTransform *>(pose);
      if (transform)
        otherAbsoluteScale = transform->absoluteScale();
      else {
        transform = pose->upperTransform();
        otherAbsoluteScale = transform ? transform->absoluteScale() : WbVector3(1.0, 1.0, 1.0);
      }
      scale /= otherAbsoluteScale;

      // compute relative translation
      position = pose->rotationMatrix().transposed() * ((position - pose->position()));
      position /= otherAbsoluteScale;
    }

    rotation.normalize();
    if (rotation.almostEquals(WbRotation(), 0.000001))
      rotation = WbRotation();

    for (int i = 0; i < mPositionLabels.size(); ++i)
      mPositionLabels[i]->setText(WbPrecision::doubleToString(position[i], WbPrecision::GUI_MEDIUM));
    for (int i = 0; i < mRotationLabels.size(); ++i)
      mRotationLabels[i]->setText(WbPrecision::doubleToString(rotation[i], WbPrecision::GUI_MEDIUM));
    if (!scale.almostEquals(WbVector3(1, 1, 1))) {
      mScaleTitleLabel->setText(tr("Scale:"));
      for (int i = 0; i < mScaleLabels.size(); ++i)
        mScaleLabels[i]->setText(WbPrecision::doubleToString(scale[i], WbPrecision::GUI_MEDIUM));
    } else {
      mScaleTitleLabel->clear();
      for (int i = 0; i < mScaleLabels.size(); ++i)
        mScaleLabels[i]->clear();
    }
    return;
  }

  for (int i = 0; i < mPositionLabels.size(); ++i)
    mPositionLabels[i]->clear();
  for (int i = 0; i < mRotationLabels.size(); ++i)
    mRotationLabels[i]->clear();
  for (int i = 0; i < mScaleLabels.size(); ++i)
    mScaleLabels[i]->clear();
}

void WbPositionViewer::updateRelativeTo(int index) {
  update();
}

void WbPositionViewer::setSelected(bool selected) {
  mIsSelected = selected;
  triggerPhysicsUpdates();
}

void WbPositionViewer::triggerPhysicsUpdates() {
  if (mPose == NULL)
    return;

  if (mIsSelected) {
    connect(mPose->translationFieldValue(), &WbSFVector3::changed, this, &WbPositionViewer::update, Qt::UniqueConnection);
    connect(mPose->rotationFieldValue(), &WbSFRotation::changed, this, &WbPositionViewer::update, Qt::UniqueConnection);
    connect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbPositionViewer::requestUpdate,
            Qt::UniqueConnection);
    update();
  } else {
    disconnect(mPose->translationFieldValue(), &WbSFVector3::changed, this, &WbPositionViewer::update);
    disconnect(mPose->rotationFieldValue(), &WbSFRotation::changed, this, &WbPositionViewer::update);
    disconnect(WbGuiRefreshOracle::instance(), &WbGuiRefreshOracle::canRefreshUpdated, this, &WbPositionViewer::requestUpdate);
  }
}

void WbPositionViewer::updateRelativeToComboBox() {
  mRelativeToComboBox->clear();
  if (mPose) {
    mRelativeToComboBox->insertItem(0, tr("Absolute"));
    int i = 0;
    const WbPose *pose = mPose->upperPose();
    while (pose) {
      ++i;
      if (pose->nodeModelName() == pose->fullName())
        mRelativeToComboBox->insertItem(i, tr("Relative to %1 (depth level %2)").arg(pose->fullName()).arg(i));
      else
        mRelativeToComboBox->insertItem(i, tr("Relative to %1").arg(pose->fullName()));
      pose = pose->upperPose();
    }
  }
}
