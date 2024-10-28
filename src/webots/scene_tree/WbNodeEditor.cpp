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

#include "WbNodeEditor.hpp"

#include "WbActionManager.hpp"
#include "WbBaseNode.hpp"
#include "WbField.hpp"
#include "WbFieldLineEdit.hpp"
#include "WbGeometry.hpp"
#include "WbGroup.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbMessageBox.hpp"
#include "WbNode.hpp"
#include "WbNodeModel.hpp"
#include "WbNodeUtilities.hpp"
#include "WbProtoManager.hpp"
#include "WbSFNode.hpp"
#include "WbSelection.hpp"
#include "WbToken.hpp"
#include "WbTransform.hpp"
#include "WbViewpoint.hpp"
#include "WbVrmlNodeUtilities.hpp"
#include "WbWorld.hpp"
#include "WbWorldInfo.hpp"

#include <QtCore/QDir>
#include <QtCore/QTimer>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStackedWidget>

WbNodeEditor *WbNodeEditor::cInstance = nullptr;

WbNodeEditor *WbNodeEditor::instance(QWidget *parent) {
  if (!cInstance)
    cInstance = new WbNodeEditor(parent);
  return cInstance;
}

WbNodeEditor::WbNodeEditor(QWidget *parent) :
  WbValueEditor(parent),
  mNode(NULL),
  mDefEdit(new WbFieldLineEdit(this)),
  mUseCount(new QLabel(this)),
  mPrintUrl(new QPushButton("Print EXTERNPROTO", this)),
  mNbTriangles(new QLabel(this)),
  mStackedWidget(new QStackedWidget(this)),
  mMessageBox(false),
  mShowResizeHandlesLabel(new QLabel(tr("3D tools:"), this)),
  mShowResizeHandlesCheckBox(new QCheckBox(tr("show resize handles"), this)) {
  mShowResizeHandlesCheckBox->setChecked(false);
  QWidget *nodePane = new QWidget(this);
  nodePane->setObjectName("NodeEditorBackground");
  QGridLayout *const layout = new QGridLayout(nodePane);
  layout->addWidget(new QLabel("DEF:", this), 0, 0);
  layout->addWidget(mDefEdit, 0, 1);
  layout->addWidget(mUseCount, 1, 1);
  layout->addWidget(mPrintUrl, 3, 1);
  layout->addWidget(mNbTriangles, 4, 1);

  layout->addWidget(mShowResizeHandlesLabel, 5, 0);
  layout->addWidget(mShowResizeHandlesCheckBox, 5, 1);

  // setup layout size policy in order to put all the widgets top - left
  // vertically
  QWidget *vStretch = new QWidget(this);
  layout->addWidget(vStretch, 5, 0);
  layout->setRowStretch(5, 1);
  // horizontally
  QWidget *hStretch = new QWidget(this);
  layout->addWidget(hStretch, 0, 2);
  layout->setColumnStretch(2, 1);

  // Main layout
  mStackedWidget->addWidget(nodePane);
  mStackedWidget->addWidget(new QWidget(this));  // empty pane
  mLayout->addWidget(mStackedWidget, 1, 1);

  connect(mDefEdit, &WbFieldLineEdit::returnPressed, this, &WbNodeEditor::apply);
  connect(mDefEdit, &WbFieldLineEdit::focusLeft, this, &WbNodeEditor::apply);
  connect(mPrintUrl, &QPushButton::pressed, this, &WbNodeEditor::printUrl);
  connect(mShowResizeHandlesCheckBox, &QAbstractButton::toggled, WbSelection::instance(),
          &WbSelection::showResizeManipulatorFromSceneTree, Qt::UniqueConnection);
  state = WbSimulationState::instance();
}

void WbNodeEditor::tryConnectToWorld() {
  world = WbWorld::instance();
  connect(world, &WbWorld::checkDefDiff, this, &WbNodeEditor::resetDefNamesToInitial);
  connect(this, &WbNodeEditor::resetModifiedFromSceneTree, world, &WbWorld::resetModifiedFromSceneTree);
  mInitialCurrentDefMap.clear();
}

void WbNodeEditor::printUrl() {
  if (!mNode->isProtoInstance())
    return;

  WbLog::info(tr("EXTERNPROTO \"%1\"").arg(WbProtoManager::instance()->externProtoUrl(mNode, true)));
}

void WbNodeEditor::recursiveBlockSignals(bool block) {
  blockSignals(block);
  mStackedWidget->blockSignals(block);
  mDefEdit->blockSignals(block);
}

void WbNodeEditor::edit(bool copyOriginalValue) {
  bool sfnodeChanged = false;
  if (singleValue()) {
    WbNode *newNode = static_cast<WbSFNode *>(singleValue())->value();
    sfnodeChanged = mNode != newNode;
    mNode = newNode;
  }

  if (copyOriginalValue || sfnodeChanged) {
    if (multipleValue())
      mNode = static_cast<WbMFNode *>(multipleValue())->item(index());

    WbBaseNode *const baseNode = dynamic_cast<WbBaseNode *>(mNode);
    if (baseNode) {
      const bool handlesAvailable = baseNode->hasResizeManipulator();

      mShowResizeHandlesLabel->setVisible(handlesAvailable);
      mShowResizeHandlesCheckBox->setVisible(handlesAvailable);
      mShowResizeHandlesCheckBox->setEnabled(true);

      if (WbNodeUtilities::isNodeOrAncestorLocked(baseNode))
        mShowResizeHandlesCheckBox->setEnabled(false);

      if (handlesAvailable) {
        const WbGeometry *g = dynamic_cast<const WbGeometry *>(baseNode);
        if (g)
          mShowResizeHandlesCheckBox->setChecked(g->isResizeManipulatorAttached());
      }
    }

    if (mNode && !mInitialCurrentDefMap.contains(mNode))
      mInitialCurrentDefMap[mNode] = QPair<QString, QString>(mNode->defName(), QString());
  }

  update();
}

void WbNodeEditor::cleanValue() {
  mNode = NULL;
  WbValueEditor::cleanValue();
  mShowResizeHandlesCheckBox->blockSignals(true);
  mShowResizeHandlesCheckBox->setChecked(false);
  mShowResizeHandlesCheckBox->blockSignals(false);
}

void WbNodeEditor::stopEditing() {
  if (!mNode)
    // value destroyed
    return;

  mShowResizeHandlesCheckBox->blockSignals(true);
  mShowResizeHandlesCheckBox->setChecked(false);
  mShowResizeHandlesCheckBox->blockSignals(false);
}

void WbNodeEditor::update() {
  if (mNode && !mNode->isUseNode()) {
    mStackedWidget->setCurrentIndex(DEF_PANE);
    mDefEdit->setText(mNode->defName());
    if (mNode->defName().isEmpty())
      mUseCount->clear();
    else
      mUseCount->setText(tr("USE count: %1").arg(mNode->useCount()));  // TODO: is this the final implementation?

    if (mNode->isProtoInstance()) {
      mPrintUrl->setVisible(true);
      mPrintUrl->setToolTip(WbProtoManager::instance()->externProtoUrl(mNode, true));
    } else
      mPrintUrl->setVisible(false);
  } else
    mStackedWidget->setCurrentIndex(EMPTY_PANE);

  const WbGeometry *node = dynamic_cast<WbGeometry *>(mNode);
  if (node && !node->isUseNode()) {
    const int maxTriangleNumberToCastShadows = node->maxIndexNumberToCastShadows() / 3;
    int triangleCount = node->triangleCount();
    if (triangleCount > maxTriangleNumberToCastShadows)
      mNbTriangles->setText(tr("Triangle count: %1 (no shadow)").arg(triangleCount));
    else
      mNbTriangles->setText(tr("Triangle count: %1").arg(triangleCount));
  } else
    mNbTriangles->clear();
}

void WbNodeEditor::resetFocus() {
  mDefEdit->clearFocus();
}

void WbNodeEditor::apply() {
  if (!mNode || mStackedWidget->currentIndex() == EMPTY_PANE)
    return;

  // message box popup makes lineEdit lose its focus
  if (mMessageBox)
    return;

  QString newDef = mDefEdit->text();
  const QString &previousDef = mNode->defName();

  mInitialCurrentDefMap[mNode].second = newDef;

  bool hasStarted = state->hasStarted();
  if (!hasStarted)
    this->compareInitialCurrentDef();

  if (newDef == previousDef)
    return;

  // block duplicated WbNodeEditor::apply call triggered by focusOutEvent
  mDefEdit->blockSignals(true);

  if (newDef.isEmpty() && mNode->useCount() > 0) {
    WbMessageBox::warning(tr("This DEF cannot be cleared because some USE nodes depend on it."), this);
    mDefEdit->setText(previousDef);  // restore
    mDefEdit->blockSignals(false);
    return;
  }

  bool dictionaryUpdateRequest = false;
  if (!newDef.isEmpty()) {
    // check if the new DEF name is not already used by subsequent USE nodes
    bool defOverlap = false;
    bool useOverlap = false;
    dictionaryUpdateRequest =
      WbVrmlNodeUtilities::hasASubsequentUseOrDefNode(mNode, newDef, previousDef, useOverlap, defOverlap);
    if (dictionaryUpdateRequest) {
      mMessageBox = true;
      QString message;
      if (defOverlap && useOverlap) {
        message = tr("This DEF string is already used by subsequent USE and DEF nodes. "
                     "Applying this change will modify all the USE nodes referring to previous node with same DEF name "
                     "and USE nodes referring to the selected node. \n"
                     "Do you want to continue?");
      } else if (defOverlap) {
        message = tr("This DEF string is already used by subsequent DEF nodes. "
                     "Applying this change will turn USE nodes of the selected node into copies of subsequent DEF node.\n"
                     "Do you want to continue?");
      } else {
        message = tr("This DEF string is already referred to by subsequent USE nodes. "
                     "Applying this change will turn them into copies of the selected node.\n"
                     "Do you want to continue?");
      }

      mMessageBox = false;

      if (WbMessageBox::question(message, this, tr("DEF name change")) == QMessageBox::Cancel) {
        mDefEdit->setText(previousDef);
        mDefEdit->blockSignals(false);
        return;
      }
    }
  }

  mDefEdit->blockSignals(false);

  // apply
  WbToken::makeValidIdentifier(newDef);
  mPreviousValue->setString(previousDef);
  mNewValue->setString(newDef);
  WbValueEditor::apply();

  update();

  if (dictionaryUpdateRequest)
    emit dictionaryUpdateRequested();
}

void WbNodeEditor::compareInitialCurrentDef() {
  if (!mInitialCurrentDefMap.isEmpty()) {
    bool foundDifference = false;
    // Iterate through the QMap
    for (auto it = mInitialCurrentDefMap.constBegin(); it != mInitialCurrentDefMap.constEnd(); ++it) {
      const QString &initialDef = it.value().first;   // First QString (initial)
      const QString &currentDef = it.value().second;  // Second QString (current)

      // Compare the two QStrings
      if (initialDef != currentDef) {
        foundDifference = true;  // Mark that a difference is found
        break;
      }
    }
    if (foundDifference)
      emit defNameChanged(true);  // Emit true if any difference is found
    else {
      emit resetModifiedFromSceneTree();
      emit defNameChanged(false);  // Emit false if no differences were found
    }
  } else
    emit defNameChanged(false);  // If all QStrings are the same, return false
}

void WbNodeEditor::resetDefNamesToInitial() {
  // Check if the map is empty
  if (mInitialCurrentDefMap.isEmpty()) {
    emit defNameChanged(false);
    return;
  }

  // Iterate through the map and reset each node's DEF name to its initial value
  for (auto it = mInitialCurrentDefMap.begin(); it != mInitialCurrentDefMap.end(); ++it) {
    WbNode *node = it.key();
    const QString &initialDef = it.value().first;  // Access initial DEF name

    // Only reset if node exists and the current DEF differs from the initial one
    if (node && node->defName() != initialDef)
      node->setDefName(initialDef);  // Set the DEF name back to the initial one
  }

  update();

  emit defNameChanged(false);
  emit resetModifiedFromSceneTree();
}

void WbNodeEditor::switchInitialCurrentDef() {
  // Check if the map is empty
  if (mInitialCurrentDefMap.isEmpty()) {
    emit defNameChanged(false);
    return;
  }

  // Iterate through the map and switch the initial DEF to the current one
  for (auto it = mInitialCurrentDefMap.begin(); it != mInitialCurrentDefMap.end(); ++it) {
    const WbNode *node = it.key();

    // Switch the initial DEF to the current DEF
    if (node) {
      QString &initialDef = it.value().first;       // Reference to initial DEF name
      const QString &currentDef = node->defName();  // Get the current DEF name of the node
      initialDef = currentDef;                      // Update the initial DEF with the current one
    }
  }

  update();

  emit defNameChanged(false);
}
