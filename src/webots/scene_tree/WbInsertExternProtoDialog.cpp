// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbInsertExternProtoDialog.hpp"

#include <WbDownloader.hpp>
#include "WbLog.hpp"
#include "WbNetwork.hpp"
#include "WbPreferences.hpp"
#include "WbProtoManager.hpp"
#include "WbUrl.hpp"

#include <QtCore/QRegularExpression>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QTreeWidgetItem>
#include <QtWidgets/QVBoxLayout>

WbInsertExternProtoDialog::WbInsertExternProtoDialog(QWidget *parent) : mRetrievalTriggered(false) {
  QVBoxLayout *const layout = new QVBoxLayout(this);

  QFont font;
  font.fromString(WbPreferences::instance()->value("Editor/font").toString());
  mSearchBar = new QLineEdit(this);
  mSearchBar->setFont(font);
  mSearchBar->setClearButtonEnabled(true);

  mTree = new QTreeWidget();

  // define buttons
  mCancelButton = new QPushButton(tr("Cancel"), this);
  mCancelButton->setFocusPolicy(Qt::ClickFocus);
  mInsertButton = new QPushButton(tr("Insert"), this);
  mInsertButton->setFocusPolicy(Qt::ClickFocus);
  connect(mCancelButton, &QPushButton::pressed, this, &WbInsertExternProtoDialog::reject);
  connect(mInsertButton, &QPushButton::pressed, this, &WbInsertExternProtoDialog::accept);

  QDialogButtonBox *const buttonBox = new QDialogButtonBox(this);
  buttonBox->addButton(mCancelButton, QDialogButtonBox::RejectRole);
  buttonBox->addButton(mInsertButton, QDialogButtonBox::AcceptRole);
  // define layout
  layout->addWidget(mSearchBar);
  layout->addWidget(mTree);
  layout->addWidget(buttonBox);

  connect(mSearchBar, &QLineEdit::textChanged, this, &WbInsertExternProtoDialog::updateProtoTree);
  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &WbInsertExternProtoDialog::updateSelection);

  updateProtoTree();
}

WbInsertExternProtoDialog::~WbInsertExternProtoDialog() {
}

void WbInsertExternProtoDialog::updateProtoTree() {
  mTree->clear();
  mTree->setHeaderHidden(true);

  QTreeWidgetItem *const projectProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (Current Project)"), WbProtoManager::PROTO_PROJECT);
  QTreeWidgetItem *const extraProtosItem =
    new QTreeWidgetItem(QStringList(tr("PROTO nodes (Extra Projects)")), WbProtoManager::PROTO_EXTRA);
  QTreeWidgetItem *const webotsProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (Webots Projects)"), WbProtoManager::PROTO_WEBOTS);

  const QRegularExpression regexp(
    QRegularExpression::wildcardToRegularExpression(mSearchBar->text(), QRegularExpression::UnanchoredWildcardConversion),
    QRegularExpression::CaseInsensitiveOption);

  const int categories[3] = {WbProtoManager::PROTO_PROJECT, WbProtoManager::PROTO_EXTRA, WbProtoManager::PROTO_WEBOTS};
  QTreeWidgetItem *const items[3] = {projectProtosItem, extraProtosItem, webotsProtosItem};
  for (int i = 0; i < 3; ++i) {
    WbProtoManager::instance()->generateProtoInfoMap(categories[i], true);
    QMapIterator<QString, WbProtoInfo *> it(WbProtoManager::instance()->protoInfoMap(categories[i]));
    while (it.hasNext()) {
      const QString &protoName = it.next().key();
      if (protoName.contains(regexp))
        items[i]->addChild(new QTreeWidgetItem(items[i], QStringList(protoName)));
    }
  }

  if (projectProtosItem->childCount() > 0)
    mTree->addTopLevelItem(projectProtosItem);
  if (extraProtosItem->childCount() > 0)
    mTree->addTopLevelItem(extraProtosItem);
  if (webotsProtosItem->childCount() > 0)
    mTree->addTopLevelItem(webotsProtosItem);

  if (mSearchBar->text().length() > 0)
    mTree->expandAll();
}

void WbInsertExternProtoDialog::accept() {
  if (mTree->selectedItems().size() == 0)
    return;

  // When declaring an EXTERNPROTO, the associated node and all the sub-proto it depends on are downloaded. Since a-priori is
  // unknown which among them is already available, it must be assumed that none is and therefore this function is called twice,
  // the second time by the retriever, and only then the dialog can be accepted

  if (!mRetrievalTriggered) {  // TODO: this needs to be done only for web proto, can be simplified somewhat?
    const QTreeWidgetItem *topLevel = mTree->selectedItems().at(0);
    while (topLevel->parent())
      topLevel = topLevel->parent();

    mProto = mTree->selectedItems().at(0)->text(0);
    mPath = WbProtoManager::instance()->protoUrl(topLevel->type(), mProto);

    printf("selected '%s' for insertion (path: %s)\n", mProto.toUtf8().constData(), mPath.toUtf8().constData());

    connect(WbProtoManager::instance(), &WbProtoManager::retrievalCompleted, this, &WbInsertExternProtoDialog::accept);
    mRetrievalTriggered = true;  // the second time the accept function is called, no retrieval should occur
    WbProtoManager::instance()->retrieveExternProto(mPath);  // note: already takes care of declaring it
    return;
  }

  // this point should only be reached after the retrieval and therefore from this point the PROTO must be available locally
  if (WbUrl::isWeb(mPath) && !WbNetwork::instance()->isCached(mPath)) {
    WbLog::error(tr("Retrieval of PROTO '%1' was unsuccessful, the asset should be cached but it is not.").arg(mProto));
    QDialog::reject();
  }

  // the addition must be declared as EXTERNPROTO so that it is added to the world file when saving
  WbProtoManager::instance()->declareExternProto(mProto, mPath, true);

  QDialog::accept();
}

void WbInsertExternProtoDialog::updateSelection() {
  if (mTree->selectedItems().size() == 0)
    return;

  const QTreeWidgetItem *const selectedItem = mTree->selectedItems().at(0);
  const QTreeWidgetItem *topLevel = selectedItem;
  while (topLevel->parent())
    topLevel = topLevel->parent();

  if (selectedItem->childCount() > 0 || topLevel == selectedItem) {
    mInsertButton->setEnabled(false);  // selected a category or folder
    return;
  }

  mInsertButton->setEnabled(true);
}
