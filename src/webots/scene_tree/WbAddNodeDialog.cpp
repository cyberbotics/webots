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

#include "WbAddNodeDialog.hpp"

#include "WbBaseNode.hpp"
#include "WbDesktopServices.hpp"
#include "WbDictionary.hpp"
#include "WbField.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbNode.hpp"
#include "WbNodeModel.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPreferences.hpp"
#include "WbProject.hpp"
#include "WbProtoCachedInfo.hpp"
#include "WbProtoList.hpp"
#include "WbProtoModel.hpp"
#include "WbSFNode.hpp"
#include "WbSimulationState.hpp"
#include "WbStandardPaths.hpp"

#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QTreeWidgetItem>

#include <cassert>

enum { NEW = 10001, USE = 10002, PROTO_WEBOTS = 10003, PROTO_EXTRA = 10004, PROTO_PROJECT = 10005 };

WbAddNodeDialog::WbAddNodeDialog(WbNode *currentNode, WbField *field, int index, QWidget *parent) :
  QDialog(parent),
  mCurrentNode(currentNode),
  mField(field),
  mIndex(index),
  mUsesItem(NULL),
  mNewNodeType(UNKNOWN),
  mDefNodeIndex(-1),
  mActionType(CREATE),
  mIsFolderItemSelected(true),
  mIsAddingLocalProtos(false),
  mIsAddingExtraProtos(false) {
  assert(mCurrentNode && mField);

  // check if top node is a robot node
  const WbNode *const topNode =
    field ? WbNodeUtilities::findTopNode(mCurrentNode) : WbNodeUtilities::findTopNode(mCurrentNode->parentNode());
  mHasRobotTopNode = topNode ? WbNodeUtilities::isRobotTypeName(topNode->nodeModelName()) : false;

  setWindowTitle(tr("Add a node"));

  mTree = new QTreeWidget(this);
  mTree->setHeaderHidden(true);
  mTree->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(mTree, &QTreeWidget::doubleClicked, this, &WbAddNodeDialog::checkAndAddSelectedItem);

  QFont font;
  font.fromString(WbPreferences::instance()->value("Editor/font").toString());

  mFindLineEdit = new QLineEdit(this);
  mFindLineEdit->setFont(font);
  mFindLineEdit->setClearButtonEnabled(true);
  connect(mFindLineEdit, &QLineEdit::textChanged, this, &WbAddNodeDialog::buildTree);

  QHBoxLayout *filterLayout = new QHBoxLayout;
  filterLayout->addStretch();
  QLabel *findLabel = new QLabel(tr("Find:"), this);
  filterLayout->addWidget(findLabel);
  filterLayout->addWidget(mFindLineEdit);

  QString toolTip(tr("Filter node names. "
                     "Only the node names containing the given string are displayed in the tree below. "
                     "Regular expressions can be used."));
  findLabel->setToolTip(toolTip);
  mFindLineEdit->setToolTip(toolTip);

  mInfoText = new QPlainTextEdit(this);
  mInfoText->setFont(font);
  mInfoText->setReadOnly(true);
  mInfoText->setFocusPolicy(Qt::ClickFocus);

  mNodeInfoGroupBox = new QGroupBox(this);
  mNodeInfoGroupBox->setObjectName("dialogInfoGroupBox");
  mNodeInfoGroupBox->setFont(font);
  mNodeInfoGroupBox->setFlat(false);

  mPixmapLabel = new QLabel(this);
  mPixmapLabel->setObjectName("nodePixmapLabel");
  mPixmapLabel->setMinimumSize(128, 128);
  mPixmapLabel->setMaximumSize(mPixmapLabel->minimumSize());

  mDocumentationLabel = new QLabel(this);
  mDocumentationLabel->setObjectName("documentationLabel");
  connect(mDocumentationLabel, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  mDocumentationLabel->setWordWrap(true);
  font.setItalic(true);
  mDocumentationLabel->setFont(font);

  mLicenseLabel = new QLabel(this);
  mLicenseLabel->setObjectName("licenseLabel");
  connect(mLicenseLabel, &QLabel::linkActivated, &WbDesktopServices::openUrl);
  mLicenseLabel->setWordWrap(true);
  font.setItalic(true);
  mLicenseLabel->setFont(font);

  QPushButton *const cancelButton = new QPushButton(tr("Cancel"), this);
  cancelButton->setFocusPolicy(Qt::ClickFocus);
  mAddButton = new QPushButton(tr("Add"), this);
  mAddButton->setFocusPolicy(Qt::ClickFocus);
  connect(cancelButton, &QPushButton::pressed, this, &WbAddNodeDialog::reject);
  connect(mAddButton, &QPushButton::pressed, this, &WbAddNodeDialog::accept);

  QHBoxLayout *const mainLayout = new QHBoxLayout(this);
  QVBoxLayout *const rightPaneLayout = new QVBoxLayout();
  QVBoxLayout *const nodeInfoLayout = new QVBoxLayout();

  nodeInfoLayout->addWidget(mPixmapLabel, 0, Qt::AlignHCenter);
  nodeInfoLayout->addWidget(mInfoText);
  nodeInfoLayout->addWidget(mDocumentationLabel);
  nodeInfoLayout->addWidget(mLicenseLabel);
  mNodeInfoGroupBox->setLayout(nodeInfoLayout);

  QDialogButtonBox *const buttonBox = new QDialogButtonBox(this);
  buttonBox->addButton(mAddButton, QDialogButtonBox::AcceptRole);

  QPushButton *const importButton = new QPushButton(tr("Import..."), this);
  importButton->setEnabled(mField->isMultiple());
  importButton->setFocusPolicy(Qt::ClickFocus);
  connect(importButton, &QPushButton::pressed, this, &WbAddNodeDialog::import);
  buttonBox->addButton(importButton, QDialogButtonBox::AcceptRole);
  buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
  buttonBox->setFocusPolicy(Qt::ClickFocus);

  QHBoxLayout *buttonLayout = new QHBoxLayout();
  buttonLayout->addWidget(buttonBox);

  if (WbSimulationState::instance()->hasStarted()) {
    QPixmap pixmap("coreIcons:warning.png");
    QLabel *pixmapLabel = new QLabel(this);
    pixmapLabel->setPixmap(pixmap.scaledToHeight(20, Qt::SmoothTransformation));
    pixmapLabel->setToolTip(tr("The simulation has run!"));
    buttonLayout->addWidget(pixmapLabel);
  }

  rightPaneLayout->addLayout(filterLayout);
  rightPaneLayout->addWidget(mNodeInfoGroupBox);
  rightPaneLayout->addLayout(buttonLayout);

  mainLayout->addWidget(mTree);
  mainLayout->addLayout(rightPaneLayout);

  // populate the tree with suitable nodes
  buildTree();

  setMinimumSize(800, 500);

  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &WbAddNodeDialog::updateItemInfo);
}

WbAddNodeDialog::~WbAddNodeDialog() {
}

QString WbAddNodeDialog::modelName() const {
  QString modelName(mTree->selectedItems().at(0)->text(MODEL_NAME));
  if (mNewNodeType == PROTO || mNewNodeType == USE) {
    // return only proto/use name without model name
    return modelName.split(QRegExp("\\W+"))[0];
  }
  return modelName;
}

QString WbAddNodeDialog::protoFilePath() const {
  if (mNewNodeType != PROTO)
    return QString();

  return mTree->selectedItems().at(0)->text(FILE_NAME);
}

WbNode *WbAddNodeDialog::defNode() const {
  assert(mDefNodeIndex >= 0);
  return mDefNodes[mDefNodeIndex];
}

void WbAddNodeDialog::updateItemInfo() {
  if (mTree->selectedItems().size() != 1)
    return;

  const QTreeWidgetItem *const selectedItem = mTree->selectedItems().at(0);
  const QTreeWidgetItem *topLevel = selectedItem;
  while (topLevel->parent())
    topLevel = topLevel->parent();

  const QString selectedNode(selectedItem->text(MODEL_NAME));
  mNodeInfoGroupBox->setTitle(selectedNode);
  bool validForInsertionInBoundingObject = true;
  mLicenseLabel->hide();
  mDocumentationLabel->hide();
  if (selectedItem->childCount() > 0 || topLevel == selectedItem) {
    // a folder is selected
    mIsFolderItemSelected = true;
    mPixmapLabel->hide();

    switch (topLevel->type()) {
      case NEW:
        mInfoText->setPlainText(tr("This folder lists all Webots base nodes that are suitable to insert at (or below) the "
                                   "currently selected Scene Tree line."));
        break;
      case USE:
        mInfoText->setPlainText(
          tr("This folder lists all suitable node that were defined (using DEF) above the current line of the Scene Tree."));
        break;
      case PROTO_EXTRA:
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes from the extra projects located in: '%1'.")
                                  .arg(WbPreferences::instance()->value("General/extraProjectsPath").toString()));
        break;
      case PROTO_PROJECT:
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes from the local 'protos' directory: '%1'.")
                                  .arg(WbProject::current()->protosPath()));
        break;
      case PROTO_WEBOTS:
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes provided by Webots."));
        break;
      default:
        // no information
        mInfoText->setPlainText(tr("No info available."));
        break;
    }

  } else {
    // a node is selected
    mIsFolderItemSelected = false;

    // check if USE node
    switch (topLevel->type()) {
      case USE: {
        mNewNodeType = USE;
        QString boi(selectedItem->text(BOUNDING_OBJECT_INFO));
        validForInsertionInBoundingObject = boi.isEmpty();
        showNodeInfo(selectedItem->text(FILE_NAME), USE, boi);
        mDefNodeIndex = mUsesItem->indexOfChild(const_cast<QTreeWidgetItem *>(selectedItem));
        assert(mDefNodeIndex < mDefNodes.size() && mDefNodeIndex >= 0);
        break;
      }
      case PROTO_WEBOTS:
      case PROTO_EXTRA:
      case PROTO_PROJECT:
        mDefNodeIndex = -1;
        mNewNodeType = PROTO;
        showNodeInfo(selectedItem->text(FILE_NAME), PROTO);
        break;
      default:
        mDefNodeIndex = -1;
        mNewNodeType = BASIC;
        showNodeInfo(selectedNode, BASIC);
        break;
    }
  }

  mAddButton->setEnabled(!selectedItem->icon(0).isNull() && validForInsertionInBoundingObject);
}

void WbAddNodeDialog::showNodeInfo(const QString &nodeFileName, NodeType nodeType, const QString &boundingObjectInfo) {
  QString info, license, documentation;
  QString pixmapPath;

  const QFileInfo fileInfo(nodeFileName);
  const QString modelName = fileInfo.baseName();
  if (nodeType != PROTO && WbNodeModel::isBaseModelName(modelName)) {
    WbNodeModel *nodeModel = WbNodeModel::findModel(modelName);
    assert(nodeModel);
    info = nodeModel->info();

    // set icon path
    pixmapPath = "icons:" + fileInfo.baseName() + ".png";

  } else {
    WbProtoCachedInfo *protoCachedInfo = new WbProtoCachedInfo(nodeFileName);
    bool success = protoCachedInfo->load();
    if (success) {
      info = protoCachedInfo->info();
      documentation = protoCachedInfo->documentationUrl();
      license = protoCachedInfo->license().simplified();
      const QString licenseUrl = protoCachedInfo->licenseUrl().simplified();
      if (!licenseUrl.isEmpty())
        license += tr(" <a style='color: #5DADE2;' href='%1'>More information.</a>").arg(licenseUrl);
    }
    delete protoCachedInfo;

    // set documentation url
    if (!documentation.isEmpty()) {
      mDocumentationLabel->show();
      mDocumentationLabel->setText(tr("Documentation: <a style='color: #5DADE2;' href='%1'>%1</a>").arg(documentation));
    }

    // set license
    if (!license.isEmpty()) {
      mLicenseLabel->show();
      mLicenseLabel->setText(tr("License: ") + license);
    }

    // set icon path
    const QString iconPath(fileInfo.absolutePath() + "/icons");
    if (QDir(iconPath).exists()) {
#ifdef _WIN32
      QStringList filenames = QDir(iconPath).entryList();
      foreach (const QString filename, filenames) {
        QString strippedFileName = filename.mid(0, filename.indexOf("."));
        if (strippedFileName.compare(fileInfo.baseName(), Qt::CaseInsensitive) == 0 &&
            strippedFileName.compare(fileInfo.baseName(), Qt::CaseSensitive) != 0) {
          WbLog::warning(tr("The icon file '%1' does not exactly match the PROTO name. Expected '%2.png'")
                           .arg(filename)
                           .arg(fileInfo.baseName()));
          break;
        }
      }
#endif
      pixmapPath = iconPath + "/" + fileInfo.baseName() + ".png";
    }
  }

  mPixmapLabel->hide();
  if (!pixmapPath.isEmpty()) {
    QPixmap pixmap(pixmapPath);
    if (!pixmap.isNull()) {
      if (pixmap.size() != QSize(128, 128)) {
        WbLog::warning(tr("The \"%1\" icon should have a dimension of 128x128 pixels.").arg(pixmapPath));
        pixmap = pixmap.scaled(128, 128);
      }
      mPixmapLabel->show();
      mPixmapLabel->setPixmap(pixmap);
    }
  }

  mInfoText->clear();

  if (!boundingObjectInfo.isEmpty())
    mInfoText->appendHtml(tr("<font color=\"red\">WARNING: this node contains a Geometry with non-positive dimensions and "
                             "hence cannot be inserted in a bounding object.</font><br/>"));

  if (info.isEmpty())
    mInfoText->setPlainText(tr("No info available."));
  else {
    // replace carriage returns with spaces where appropriate:
    // "\n\n" => "\n\n": two consecutive carriage returns are preserved (new paragraph)
    // "\n-"  => "\n-": a carriage return followed by a "-" are preserved (bullet list)
    // "\n"   => " ": a single carriage return is transformed into a space (comment line wrap)
    for (int i = 0; i < info.length(); i++) {
      if (info[i] == '\n') {
        if (i < (info.length() - 1)) {
          if (info[i + 1] == '\n' || info[i + 1] == '-') {
            i++;
            continue;
          }
        }
        info[i] = ' ';
      }
    }
    mInfoText->appendPlainText(info.trimmed());
  }
  mInfoText->moveCursor(QTextCursor::Start);
}

bool WbAddNodeDialog::doFieldRestrictionsAllowNode(const QString &nodeName) const {
  foreach (const WbVariant variant, mField->acceptedValues()) {
    const WbNode *node = variant.toNode();
    assert(node);
    if (node->modelName() == nodeName)
      return true;
  }
  return false;
}

void WbAddNodeDialog::buildTree() {
  mTree->clear();
  mUsesItem = NULL;
  mDefNodes.clear();
  mUniqueLocalProtoNames.clear();
  mUniqueExtraProtoNames.clear();

  // basic tree items
  QTreeWidgetItem *const nodesItem = new QTreeWidgetItem(QStringList(tr("Base nodes")), NEW);
  QTreeWidgetItem *const wprotosItem = new QTreeWidgetItem(QStringList("PROTO nodes (Webots Projects)"), PROTO_WEBOTS);

  QStringList basicNodes;
  mUsesItem = new QTreeWidgetItem(QStringList("USE"), USE);
  QTreeWidgetItem *lprotosItem = new QTreeWidgetItem(QStringList(tr("PROTO nodes (Current Project)")), PROTO_PROJECT);
  QTreeWidgetItem *aprotosItem = WbPreferences::instance()->value("General/extraProjectsPath").toString().isEmpty() ?
                                   NULL :
                                   new QTreeWidgetItem(QStringList(tr("PROTO nodes (Extra Projects)")), PROTO_EXTRA);
  basicNodes = WbNodeModel::baseModelNames();

  QTreeWidgetItem *item = NULL;

  // add valid basic nodes
  const WbNode::NodeUse nodeUse = static_cast<WbBaseNode *>(mCurrentNode)->nodeUse();
  foreach (const QString &basicNodeName, basicNodes) {
    QFileInfo fileInfo(basicNodeName);
    QString errorMessage;
    if (fileInfo.baseName().contains(QRegExp(mFindLineEdit->text(), Qt::CaseInsensitive, QRegExp::Wildcard)) &&
        WbNodeUtilities::isAllowedToInsert(mField, fileInfo.baseName(), mCurrentNode, errorMessage, nodeUse, QString(),
                                           QStringList(fileInfo.baseName()))) {
      item = new QTreeWidgetItem(nodesItem, QStringList(fileInfo.baseName()));
      item->setIcon(0, QIcon("enabledIcons:node.png"));
      nodesItem->addChild(item);
    }
  }

  // add USE nodes that are suitable for insertion
  if (mUsesItem) {
    static const QString INVALID_FOR_INSERTION_IN_BOUNDING_OBJECT("N");

    const WbField *const actualField =
      (mField->isParameter() && !mField->alias().isEmpty()) ? mField->internalFields().at(0) : mField;
    bool boInfo = actualField->name() == "boundingObject";
    if (!boInfo)
      boInfo = nodeUse & WbNode::BOUNDING_OBJECT_USE;

    // populates the DEF-USE dictionary with suitable definitions located above mCurrentNode
    mDefNodes = WbDictionary::instance()->computeDefForInsertion(mCurrentNode, mField, mIndex, true);
    foreach (const WbNode *const defNode, mDefNodes) {
      const QString &currentDefName = defNode->defName();
      const QString &currentModelName = defNode->modelName();
      const QString &currentFullDefName = currentDefName + " (" + currentModelName + ")";
      if (!currentFullDefName.contains(QRegExp(mFindLineEdit->text(), Qt::CaseInsensitive, QRegExp::Wildcard)))
        continue;
      if (mField->hasRestrictedValues() &&
          (!doFieldRestrictionsAllowNode(currentModelName) && !doFieldRestrictionsAllowNode(defNode->nodeModelName())))
        continue;
      QString nodeFilePath(currentModelName);
      if (!WbNodeModel::isBaseModelName(currentModelName))
        nodeFilePath = WbProtoList::current()->findModelPath(currentModelName);

      QStringList strl(QStringList() << currentFullDefName << nodeFilePath);

      if (boInfo && !(dynamic_cast<const WbBaseNode *const>(defNode))->isSuitableForInsertionInBoundingObject())
        strl << INVALID_FOR_INSERTION_IN_BOUNDING_OBJECT;

      QTreeWidgetItem *const child = new QTreeWidgetItem(mUsesItem, strl);

      child->setIcon(0, QIcon("enabledIcons:node.png"));
    }
  }

  mUniqueLocalProtoNames.clear();
  mUniqueExtraProtoNames.clear();

  // add project PROTO
  if (lprotosItem) {
    mIsAddingLocalProtos = true;
    addProtosFromDirectory(lprotosItem, WbProject::current()->path() + "/protos/", mFindLineEdit->text(),
                           QDir(WbProject::current()->path() + "/protos/"));
    mIsAddingLocalProtos = false;
  }

  // add extra PROTO
  if (aprotosItem) {
    mIsAddingExtraProtos = true;
    const QString &extraProjectsPath = WbPreferences::instance()->value("General/extraProjectsPath").toString();
    addProtosFromDirectory(aprotosItem, extraProjectsPath, mFindLineEdit->text(), QDir(extraProjectsPath));
    mIsAddingExtraProtos = false;
  }

  // add Webots PROTO
  int nWProtosNodes = 0;
  nWProtosNodes = addProtosFromDirectory(wprotosItem, WbStandardPaths::projectsPath(), mFindLineEdit->text(),
                                         QDir(WbStandardPaths::projectsPath()));
  mTree->addTopLevelItem(nodesItem);
  if (mUsesItem)
    mTree->addTopLevelItem(mUsesItem);
  if (lprotosItem)
    mTree->addTopLevelItem(lprotosItem);
  if (aprotosItem)
    mTree->addTopLevelItem(aprotosItem);
  mTree->addTopLevelItem(wprotosItem);

  // initial selection
  const int nBasicNodes = nodesItem->childCount();
  const int nUseNodes = mUsesItem ? mUsesItem->childCount() : 0;
  const int nLProtosNodes = lprotosItem ? lprotosItem->childCount() : 0;
  const int nAProtosNodes = aprotosItem ? aprotosItem->childCount() : 0;

  // if everything can fit in the tree height then show all
  if (nBasicNodes + nUseNodes + nLProtosNodes + nAProtosNodes + nWProtosNodes < 20)
    mTree->expandAll();

  // if no USE nor PROTO items
  if (nBasicNodes && !nUseNodes && !nLProtosNodes && !nAProtosNodes && !nWProtosNodes)
    // then select first basic node
    mTree->setCurrentItem(nodesItem->child(0));
  else
    mTree->setCurrentItem(nodesItem);

  updateItemInfo();
}

int WbAddNodeDialog::addProtosFromDirectory(QTreeWidgetItem *parentItem, const QString &dirPath, const QString &regex,
                                            const QDir &rootDirectory, bool recurse, bool inProtos) {
  QDir dir(dirPath);
  if (!dir.exists() || !dir.isReadable()) {
    // no protos node
    return 0;
  }
  int nAddedNodes = 0;
  int nNodes = 0;
  if (dirPath.endsWith("/protos/"))
    inProtos = true;
  if (inProtos) {
    QStringList filter("*.proto");
    // search in folder
    const QStringList &protoFiles = dir.entryList(filter, QDir::Files, QDir::Name);
    nAddedNodes += addProtos(parentItem, protoFiles, dir.absolutePath(), regex, rootDirectory);
  }
  // search in subfolders
  QTreeWidgetItem *newFolderItem;
  QStringList list(dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot));
  const int size = list.size();
  if (recurse)
    for (int i = 0; i < size; ++i)
      if (list[i] == "controllers" || list[i] == "worlds" || list[i] == "plugins" || list[i] == "protos") {
        recurse = false;
        break;
      }
  for (int i = 0; i < size; ++i) {
    if (inProtos && (list[i] == "textures" || list[i] == "icons"))
      continue;
    if (list[i] == "worlds")
      continue;
    if (list[i] == "controllers")
      continue;
    if (list[i] == "plugins")
      continue;
    if (list[i] == "protos")
      // do not add 'protos' item to the tree
      newFolderItem = parentItem;
    else {
      newFolderItem = new QTreeWidgetItem(QStringList(list[i]));
      parentItem->addChild(newFolderItem);
    }
    if (list[i] == "protos" || inProtos || recurse)
      nNodes = addProtosFromDirectory(newFolderItem, dir.absolutePath() + "/" + list[i] + "/", regex, rootDirectory, recurse,
                                      inProtos);
    else
      nNodes = 0;
    if (nNodes == 0) {
      if (parentItem != newFolderItem) {
        parentItem->removeChild(newFolderItem);
        delete newFolderItem;
      }
    } else
      nAddedNodes += nNodes;
  }
  return nAddedNodes;
}

int WbAddNodeDialog::addProtos(QTreeWidgetItem *parentItem, const QStringList &protoList, const QString &dirPath,
                               const QString &regex, const QDir &rootDirectory) {
  QTreeWidgetItem *item;
  int nAddedNodes = 0;

  const WbNode::NodeUse nodeUse = static_cast<WbBaseNode *>(mCurrentNode)->nodeUse();
  foreach (const QString protoFile, protoList) {
    const QString protoFilePath(dirPath + "/" + protoFile);
    WbProtoCachedInfo *protoCachedInfo = new WbProtoCachedInfo(protoFilePath);
    bool success = protoCachedInfo->load();
    if (!success || protoCachedInfo->isOutOfDate()) {
      // (re)compute if not valid
      delete protoCachedInfo;
      protoCachedInfo = WbProtoCachedInfo::computeInfo(protoFilePath);
      if (!protoCachedInfo)
        // ignore invalid PROTO file
        continue;
    }

    if (protoCachedInfo->baseType() == "UNKNOWN")
      continue;

    // don't display PROTOs which contain a "hidden" or a "deprecated" tag
    QStringList tags = protoCachedInfo->tags();
    if (tags.contains("deprecated", Qt::CaseInsensitive) || tags.contains("hidden", Qt::CaseInsensitive))
      continue;

    // don't display PROTO nodes which have been filtered-out by the user's "filter" widget.
    if (!rootDirectory.relativeFilePath(protoFilePath).contains(QRegExp(regex, Qt::CaseInsensitive, QRegExp::Wildcard)) &&
        !protoCachedInfo->baseType().contains(QRegExp(regex, Qt::CaseInsensitive, QRegExp::Wildcard)))
      continue;

    // don't display non-Robot PROTO nodes containing devices (e.g. Kinect) about to be inserted outside a robot.
    if (!mHasRobotTopNode && !WbNodeUtilities::isRobotTypeName(protoCachedInfo->baseType()) &&
        protoCachedInfo->needsRobotAncestor())
      continue;

    QString errorMessage;
    if (!WbNodeUtilities::isAllowedToInsert(mField, protoCachedInfo->baseType(), mCurrentNode, errorMessage, nodeUse,
                                            protoCachedInfo->slotType(),
                                            QStringList() << protoCachedInfo->baseType() << protoFile.chopped(6)))
      continue;

    const QFileInfo fileInfo(protoFile);
    item = new QTreeWidgetItem(
      QStringList(QStringList() << fileInfo.baseName() + " (" + protoCachedInfo->baseType() + ")" << protoFilePath));
    item->setIcon(0, QIcon("enabledIcons:proto.png"));
    parentItem->addChild(item);
    ++nAddedNodes;

    if (mUniqueLocalProtoNames.contains(fileInfo.baseName())) {
      // disable item if PROTO model with same name already exists in local project
      item->setDisabled(true);
      item->setToolTip(
        0,
        tr("Node not available because another project PROTO model with the same name already exists in your local project."));
    } else if (mUniqueExtraProtoNames.contains(fileInfo.baseName())) {
      // disable item if PROTO model with same name already exists in extra projects
      item->setDisabled(true);
      item->setToolTip(0, tr("Node not available because another project PROTO model with the same name already exists in your "
                             "extra projects."));
    } else if (mIsAddingLocalProtos)
      mUniqueLocalProtoNames.append(fileInfo.baseName());
    else if (mIsAddingExtraProtos)
      mUniqueExtraProtoNames.append(fileInfo.baseName());
  }

  return nAddedNodes;
}

void WbAddNodeDialog::import() {
  static QString lastFolder = QDir::homePath();
  QString fileName = QFileDialog::getOpenFileName(this, tr("Import Webots Object"), lastFolder, tr("Files (*.wbo *.WBO)"));
  if (fileName.isEmpty())
    return;

  lastFolder = QFileInfo(fileName).absolutePath();
  mActionType = IMPORT;
  mImportFileName = fileName;
  accept();
}

void WbAddNodeDialog::checkAndAddSelectedItem() {
  if (mIsFolderItemSelected)
    return;

  accept();
}
