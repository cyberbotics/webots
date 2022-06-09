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

enum { PROTO_PROJECT = 10001, PROTO_EXTRA = 10002, PROTO_WEBOTS = 10003 };  // TODO: should be moved to WbProtoManager?

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

  QTreeWidgetItem *const projectProtosItem = new QTreeWidgetItem(QStringList("PROTO nodes (Current Project)"), PROTO_PROJECT);
  QTreeWidgetItem *const extraProtosItem = new QTreeWidgetItem(QStringList(tr("PROTO nodes (Extra Projects)")), PROTO_EXTRA);
  QTreeWidgetItem *const webotsProtosItem = new QTreeWidgetItem(QStringList("PROTO nodes (Webots Projects)"), PROTO_WEBOTS);

  const QRegularExpression regexp(
    QRegularExpression::wildcardToRegularExpression(mSearchBar->text(), QRegularExpression::UnanchoredWildcardConversion),
    QRegularExpression::CaseInsensitiveOption);

  // list of all available protos in the current project
  foreach (const QString &protoName, WbProtoManager::instance()->nameList(WbProtoManager::PROTO_PROJECT)) {
    if (protoName.contains(regexp))
      projectProtosItem->addChild(new QTreeWidgetItem(projectProtosItem, QStringList(protoName)));
  }
  // list of all available protos in the current project
  foreach (const QString &protoName, WbProtoManager::instance()->nameList(WbProtoManager::PROTO_EXTRA)) {
    if (protoName.contains(regexp))
      extraProtosItem->addChild(new QTreeWidgetItem(extraProtosItem, QStringList(protoName)));
  }
  // list of all available protos among the webots ones
  foreach (const QString &protoName, WbProtoManager::instance()->nameList(WbProtoManager::PROTO_WEBOTS)) {
    if (protoName.contains(regexp))
      webotsProtosItem->addChild(new QTreeWidgetItem(webotsProtosItem, QStringList(protoName)));
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
  // when inserting a PROTO, it's necessary to ensure it is cached (both it and all the sub-proto it depends on). This may not
  // typically be the case hence we are forced to assume nothing is available (the root proto might be available, but not
  // necessarily all its subs, or vice-versa), then trigger the cascaded download (which will download the sub-proto only if
  // necessary) and only when the retriever gives the go ahead the dialog's accept function can actually be executed entirely.
  // In short, two passes are unavoidable for any inserted proto.

  if (mTree->selectedItems().size() == 0)
    return;

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
    return;  // TODO: or reject?
  }

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
