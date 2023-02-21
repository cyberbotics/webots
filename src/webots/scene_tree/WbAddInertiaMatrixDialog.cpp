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

#include "WbAddInertiaMatrixDialog.hpp"
#include "WbField.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbNode.hpp"
#include "WbNodeModel.hpp"
#include "WbNodeUtilities.hpp"
#include "WbProject.hpp"
#include "WbProtoModel.hpp"
#include "WbSFNode.hpp"
#include "WbStandardPaths.hpp"
#include "WbTreeItem.hpp"

#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QTreeWidgetItem>

#include <cassert>

enum { IDENTITY_MATRIX = 10001, BOUNDING_OBJECT_BASED = 10002 };

WbAddInertiaMatrixDialog::WbAddInertiaMatrixDialog(bool validBoudingObject, QWidget *parent) :
  QDialog(parent),
  mValidBoundingObject(validBoudingObject) {
  setWindowTitle(tr("Add inertia matrix"));

  mTree = new QTreeWidget(this);
  mTree->setHeaderHidden(true);
  mTree->setSelectionMode(QAbstractItemView::SingleSelection);

  mInfoText = new QPlainTextEdit(this);
  mInfoText->setReadOnly(true);

  mGroupBox = new QGroupBox(this);
  mGroupBox->setObjectName("dialogInfoGroupBox");
  mGroupBox->setFlat(false);

  QPushButton *cancelButton = new QPushButton(tr("Cancel"), this);
  mAddButton = new QPushButton(tr("Add"), this);
  connect(cancelButton, &QPushButton::pressed, this, &WbAddInertiaMatrixDialog::reject);
  connect(mAddButton, &QPushButton::pressed, this, &WbAddInertiaMatrixDialog::accept);

  QHBoxLayout *mainLayout = new QHBoxLayout(this);
  QVBoxLayout *rightPaneLayout = new QVBoxLayout();
  QVBoxLayout *groupBoxLayout = new QVBoxLayout();

  // groupBoxLayout->addWidget(mPixmapLabel, 0, Qt::AlignHCenter);
  groupBoxLayout->addWidget(mInfoText);
  mGroupBox->setLayout(groupBoxLayout);

  QDialogButtonBox *buttonBox = new QDialogButtonBox(this);
  buttonBox->addButton(mAddButton, QDialogButtonBox::AcceptRole);
  buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);

  rightPaneLayout->addWidget(mGroupBox);
  rightPaneLayout->addWidget(buttonBox);

  mainLayout->addWidget(mTree);
  mainLayout->addLayout(rightPaneLayout);

  // Populates the tree with an item for the default inertia matrix and an item for the bounding object based intertia matrix
  buildTree();

  setMinimumSize(500, 390);

  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &WbAddInertiaMatrixDialog::updateItemInfo);
}

int WbAddInertiaMatrixDialog::inertiaMatrixType() const {
  QTreeWidgetItem *selectedItem = mTree->selectedItems().at(0);
  const int index = mTree->indexOfTopLevelItem(selectedItem);
  switch (index) {
    case 0:
      return IDENTITY_MATRIX;
    case 1:
      return BOUNDING_OBJECT_BASED;
    default:
      return IDENTITY_MATRIX;
  }
}

void WbAddInertiaMatrixDialog::updateItemInfo() {
  if (mTree->selectedItems().size() != 1)
    return;

  const QTreeWidgetItem *selectedItem = mTree->selectedItems().at(0);
  const QString &selectedInertiaMatrixText = selectedItem->text(0);

  mGroupBox->setTitle(selectedInertiaMatrixText);
  switch (selectedItem->type()) {
    case IDENTITY_MATRIX:
      static const QString strIdentity = tr("Insert the identity matrix \n [ 1 1 1, 0 0 0]");
      static const QString strZero = tr("If no center of mass is currently specified, a zero 3D vector will be inserted.");
      static const QString strOne =
        tr("The density will be set as -1 and if the mass is negative or zero, it will be set as 1.");
      static const QString textId = strIdentity + "\n\n" + strZero + "\n\n" + strOne;
      mInfoText->setPlainText(textId);
      break;
    case BOUNDING_OBJECT_BASED:
      static const QString strInertia =
        tr("The inertia matrix is computed using the solid bounding object and the frame obtained by translating "
           "solid's frame to bounding object's center of mass.");
      static const QString strCoM = tr("The center of mass will be set as the bounding object center of mass.");
      static const QString strMass =
        tr("If the density is currently specified, it will be set as -1 and the mass will be set as the bounding object mass.");
      static const QString textBo = strInertia + "\n\n" + strCoM + "\n\n" + strMass;
      mInfoText->setPlainText(textBo);
      break;
    default:
      // no information
      mInfoText->setPlainText(tr("No info available."));
      break;
  }

  mAddButton->setEnabled(!selectedItem->icon(0).isNull());
}

void WbAddInertiaMatrixDialog::buildTree() {
  // Two top tree items
  QTreeWidgetItem *defaultInertiaMatrixItem = new QTreeWidgetItem(QStringList(tr("Identity matrix")), IDENTITY_MATRIX);
  defaultInertiaMatrixItem->setIcon(0, QIcon("enabledIcons:field.png"));
  mTree->addTopLevelItem(defaultInertiaMatrixItem);

  QTreeWidgetItem *boundingObjectBasedItem = NULL;
  if (mValidBoundingObject) {
    boundingObjectBasedItem = new QTreeWidgetItem(QStringList(tr("Bounding object based")), BOUNDING_OBJECT_BASED);
    boundingObjectBasedItem->setIcon(0, QIcon("enabledIcons:field.png"));
    mTree->addTopLevelItem(boundingObjectBasedItem);
  }

  mTree->setCurrentItem(defaultInertiaMatrixItem);
  updateItemInfo();
}
