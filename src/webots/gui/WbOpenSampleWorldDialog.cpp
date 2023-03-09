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

#include "WbOpenSampleWorldDialog.hpp"

#include "WbLog.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QDir>
#include <QtGui/QStandardItemModel>
#include <QtWidgets/QtWidgets>

#include <cassert>

namespace {

  // derived QStandardItem to be able to keep a qfileinfo and be able to
  // easily detect these kind of objects thanks to dynamic_cast
  class FileDataItem : public QStandardItem {
  public:
    FileDataItem(const QString &text, const QFileInfo &fi) : QStandardItem(text), mFileInfo(fi) {}
    virtual ~FileDataItem() {}

    const QFileInfo &fileInfo() const { return mFileInfo; }

  private:
    QFileInfo mFileInfo;
  };
  static const QStringList skipDirs = (QStringList() << "protos"
                                                     << "controllers"
                                                     << "plugins");
  // function telling if a given directory contains (recursively) at least one
  // file finishing with ".wbt", located inside a "worlds" directory and matching regex
  // stopping search at "protos", "controllers", "protos", "plugins", etc. directories
  bool directoryContainsWbtFile(QDir dir, const QString &wildcard) {
    if (dir.dirName() == "worlds") {
      dir.setFilter(QDir::Files);
      dir.setNameFilters(QStringList("*.wbt"));
      QStringList fileList = dir.entryList();
      const QDir projectsDir = QDir(WbStandardPaths::projectsPath());
      const QRegularExpression regexp = QRegularExpression(
        QRegularExpression::wildcardToRegularExpression(wildcard, QRegularExpression::UnanchoredWildcardConversion),
        QRegularExpression::CaseInsensitiveOption);
      foreach (const QString &fileName, fileList) {
        if (projectsDir.relativeFilePath(dir.path() + fileName).contains(regexp))
          return true;
      }
      return false;
    }
    dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
    QFileInfoList fileInfos = dir.entryInfoList();
    foreach (const QFileInfo &fileInfo, fileInfos)
      if (fileInfo.fileName() == "worlds")
        return directoryContainsWbtFile(QDir(fileInfo.filePath()), wildcard);
    // from here, no "worlds" folder was found, so we need to explore deeper all folders
    foreach (const QFileInfo &fileInfo, fileInfos)
      if (skipDirs.contains(fileInfo.fileName()))
        return false;
      else if (directoryContainsWbtFile(QDir(fileInfo.filePath()), wildcard))
        return true;
    return false;
  }

  // creating the QStandardItem children given a QDir
  // - append only files finishing with ".wbt", located inside a "worlds" directory and matching regex
  // - skip recursively empty directories
  // - skip "worlds" layers (but passing through them anyway)
  // - stop search at "protos", "controllers", "plugins" and other similar folders
  void populateTreeModel(int &itemCounter, QStandardItem *parent, QDir dir, const QString &wildcard) {
    if (dir.dirName() == "worlds") {
      dir.setFilter(QDir::Files | QDir::NoDotAndDotDot | QDir::NoSymLinks);
      dir.setNameFilters(QStringList("*.wbt"));
      QFileInfoList fileInfos = dir.entryInfoList();
      const QDir projectsDir = QDir(WbStandardPaths::projectsPath());
      const QRegularExpression re = QRegularExpression(
        QRegularExpression::wildcardToRegularExpression(wildcard, QRegularExpression::UnanchoredWildcardConversion),
        QRegularExpression::CaseInsensitiveOption);
      foreach (const QFileInfo &fileInfo, fileInfos) {
        if (projectsDir.relativeFilePath(fileInfo.filePath()).contains(re)) {
          FileDataItem *item = new FileDataItem(fileInfo.fileName(), fileInfo);
          parent->appendRow(item);
          ++itemCounter;
        }
      }
      return;
    }
    dir.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
    QStringList dirList = dir.entryList();
    if (dirList.contains("worlds"))
      dirList = QStringList("worlds");
    else {
      foreach (const QString &skippedDir, skipDirs) {
        if (dirList.contains(skippedDir))
          return;
      }
    }
    foreach (const QString &dirName, dirList) {
      QDir childDir(QString("%1/%2").arg(dir.absolutePath()).arg(dirName));
      if (directoryContainsWbtFile(childDir, wildcard)) {
        if (dirName == "worlds")  // skip this directory layer
          populateTreeModel(itemCounter, parent, childDir, wildcard);
        else {
          QStandardItem *item = new QStandardItem(dirName);
          parent->appendRow(item);
          populateTreeModel(itemCounter, item, childDir, wildcard);
        }
      }
    }
  }
};  // namespace

WbOpenSampleWorldDialog::WbOpenSampleWorldDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle(tr("Open Sample World"));

  mFindLineEdit = new QLineEdit(this);
  mFindLineEdit->setClearButtonEnabled(true);
  connect(mFindLineEdit, &QLineEdit::textChanged, this, &WbOpenSampleWorldDialog::updateTree);

  QHBoxLayout *filterLayout = new QHBoxLayout;
  filterLayout->addStretch();
  QLabel *findLabel = new QLabel(tr("Find :"), this);
  filterLayout->addWidget(findLabel);
  filterLayout->addWidget(mFindLineEdit);

  QString toolTip(tr("Filter world names. "
                     "Only the world names containing the given string are displayed in the tree below. "
                     "Regular expressions can be used."));
  findLabel->setToolTip(toolTip);
  mFindLineEdit->setToolTip(toolTip);

  mModel = new QStandardItemModel(this);
  mTreeView = new QTreeView(this);
  mTreeView->setModel(mModel);
  mTreeView->setEditTriggers(QAbstractItemView::NoEditTriggers);
  mTreeView->setHeaderHidden(true);
  mTreeView->setFocus();
  connect(mTreeView, &QTreeView::doubleClicked, this, &WbOpenSampleWorldDialog::handleDoubleClick);
  connect(mTreeView->selectionModel(), &QItemSelectionModel::selectionChanged, this,
          &WbOpenSampleWorldDialog::handleSelectedItemChanged);

  mButtonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this);
  mButtonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
  connect(mButtonBox, &QDialogButtonBox::accepted, this, &WbOpenSampleWorldDialog::accept);
  connect(mButtonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
  foreach (QAbstractButton *const button, mButtonBox->buttons())
    button->setFocusPolicy(Qt::ClickFocus);

  QHBoxLayout *buttonLayout = new QHBoxLayout;
  buttonLayout->addStretch();
  buttonLayout->addWidget(mButtonBox);

  QGridLayout *mainLayout = new QGridLayout(this);
  mainLayout->addLayout(filterLayout, 0, 0);
  mainLayout->addWidget(mTreeView, 1, 0);
  mainLayout->addLayout(buttonLayout, 2, 0);

  updateTree();
}

WbOpenSampleWorldDialog::~WbOpenSampleWorldDialog() {
}

QSize WbOpenSampleWorldDialog::sizeHint() const {
  return QSize(350, 500);
}

void WbOpenSampleWorldDialog::accept() {
  QModelIndexList list = mTreeView->selectionModel()->selectedIndexes();
  if (list.size() == 1) {
    const QStandardItem *clickedItem = mModel->itemFromIndex(list[0]);
    const FileDataItem *clickedFileItem = dynamic_cast<const FileDataItem *>(clickedItem);
    if (clickedFileItem) {
      mSelectedWorld = clickedFileItem->fileInfo().absoluteFilePath();
      QDialog::accept();
    } else
      assert(0);  // this should never occur if the ok button is correctly disabled
  } else
    assert(0);  // this should never occur if the ok button is correctly disabled
}

void WbOpenSampleWorldDialog::handleDoubleClick(const QModelIndex &index) {
  const QStandardItem *clickedItem = mModel->itemFromIndex(index);
  const FileDataItem *clickedFileItem = dynamic_cast<const FileDataItem *>(clickedItem);
  if (clickedFileItem) {
    mSelectedWorld = clickedFileItem->fileInfo().absoluteFilePath();
    QDialog::accept();
  }
}

void WbOpenSampleWorldDialog::handleSelectedItemChanged(const QItemSelection &selected, const QItemSelection &deselected) {
  bool validSelection = false;
  if (selected.indexes().size() == 1) {
    const QStandardItem *clickedItem = mModel->itemFromIndex(selected.indexes().at(0));
    validSelection = dynamic_cast<const FileDataItem *>(clickedItem) != NULL;
  }
  mButtonBox->button(QDialogButtonBox::Ok)->setEnabled(validSelection);
}

void WbOpenSampleWorldDialog::updateTree(const QString &reg) {
  mModel->clear();
  mButtonBox->button(QDialogButtonBox::Ok)->setEnabled(false);

  QStandardItem *parentItem = mModel->invisibleRootItem();

  int itemCounter = 0;
  populateTreeModel(itemCounter, parentItem, QDir(WbStandardPaths::projectsPath()), mFindLineEdit->text());

  if (itemCounter < 10)
    mTreeView->expandAll();
  else
    mTreeView->collapseAll();
}
