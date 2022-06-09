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

#include "WbAddNodeDialog.hpp"

#include <WbDownloader.hpp>
#include <WbNetwork.hpp>
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
#include "WbProtoManager.hpp"
#include "WbProtoModel.hpp"
#include "WbSFNode.hpp"
#include "WbSimulationState.hpp"
#include "WbStandardPaths.hpp"
#include "WbUrl.hpp"

#include <QtCore/QRegularExpression>
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

enum { NEW = 20001, USE = 20002 };

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
  mRetrievalTriggered(false) {
  assert(mCurrentNode && mField);

  mIconDownloaders.clear();
  // check if top node is a robot node
  const WbNode *const topNode =
    field ? WbNodeUtilities::findTopNode(mCurrentNode) : WbNodeUtilities::findTopNode(mCurrentNode->parentNode());
  mHasRobotTopNode = topNode ? WbNodeUtilities::isRobotTypeName(topNode->nodeModelName()) : false;

  setWindowTitle(tr("Add a node"));

  mTree = new QTreeWidget(this);
  mTree->setHeaderHidden(true);
  mTree->setSelectionMode(QAbstractItemView::SingleSelection);
  mTree->setSortingEnabled(true);
  mTree->sortByColumn(0, Qt::AscendingOrder);
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

  mExportProtoButton = new QPushButton(tr("Export"), this);
  mExportProtoButton->setEnabled(true);
  mExportProtoButton->setVisible(false);
  mExportProtoButton->setFocusPolicy(Qt::ClickFocus);
  connect(mExportProtoButton, &QPushButton::pressed, this, &WbAddNodeDialog::exportProto);
  buttonBox->addButton(mExportProtoButton, QDialogButtonBox::AcceptRole);
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

void WbAddNodeDialog::downloadIcon(const QString &url) {
  WbDownloader *downloader = new WbDownloader(this);
  mIconDownloaders.push_back(downloader);
  connect(downloader, &WbDownloader::complete, this, &WbAddNodeDialog::iconUpdate);

  downloader->download(QUrl(url));
}

void WbAddNodeDialog::iconUpdate() {
  WbDownloader *source = dynamic_cast<WbDownloader *>(sender());
  if (source && !source->error().isEmpty()) {
    WbLog::error(source->error());  // failure downloading or file does not exist (404)
    return;
  }

  // set the image
  QString pixmapPath = WbNetwork::instance()->get(source->url().toString());
  QPixmap pixmap(pixmapPath);
  if (!pixmap.isNull()) {
    if (pixmap.size() != QSize(128, 128)) {
      WbLog::warning(tr("The \"%1\" icon should have a dimension of 128x128 pixels.").arg(pixmapPath));
      pixmap = pixmap.scaled(128, 128);
    }
    mPixmapLabel->show();
    mPixmapLabel->setPixmap(pixmap);
  }

  // purge completed downloaders
  for (int i = mIconDownloaders.size() - 1; i >= 0; --i) {
    if (mIconDownloaders[i] && !mIconDownloaders[i]->hasFinished())
      mIconDownloaders.remove(i);
  }
}

QString WbAddNodeDialog::modelName() const {
  QString modelName(mTree->selectedItems().at(0)->text(MODEL_NAME));
  if (mNewNodeType == PROTO || mNewNodeType == USE) {
    // return only proto/use name without model name
    return modelName.split(QRegularExpression("\\W+"))[0];
  }
  return modelName;
}

QString WbAddNodeDialog::protoFilePath() const {
  if (mNewNodeType != PROTO)
    return QString();

  QString path = mTree->selectedItems().at(0)->text(FILE_NAME);
  path = WbUrl::generateExternProtoPath(path, protoFileExternPath());
  if (WbUrl::isWeb(path) && WbNetwork::instance()->isCached(path))
    path = WbNetwork::instance()->get(path);

  printf("INSERTING: %s\n", path.toUtf8().constData());
  return path;
}

QString WbAddNodeDialog::protoFileExternPath() const {
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
    mExportProtoButton->setVisible(false);
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
      case WbProtoManager::PROTO_WORLD: {
        const QString worldFile = "ASD";  // TODO
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes from the world file: '%1'.").arg(worldFile));
        break;
      }
      case WbProtoManager::PROTO_PROJECT:
        mInfoText->setPlainText(tr("This folder lists all suitable PROTO nodes from the local 'protos' directory: '%1'.")
                                  .arg(WbProject::current()->protosPath()));
        break;
      case WbProtoManager::PROTO_EXTRA: {
        QString title("This folder lists all suitable PROTO nodes from the preferences Extra project path and "
                      "the 'WEBOTS_EXTRA_PROJECT_PATH' environment variable:\n");
        foreach (const WbProject *project, *WbProject::extraProjects())
          title.append(QString("- " + project->path() + "\n"));
        mInfoText->setPlainText(title);
      } break;
      case WbProtoManager::PROTO_WEBOTS:
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
        showNodeInfo(selectedItem->text(FILE_NAME), USE, 0, boi);
        mDefNodeIndex = mUsesItem->indexOfChild(const_cast<QTreeWidgetItem *>(selectedItem));
        assert(mDefNodeIndex < mDefNodes.size() && mDefNodeIndex >= 0);
        mExportProtoButton->setVisible(false);
        break;
      }
      case WbProtoManager::PROTO_WORLD:
      case WbProtoManager::PROTO_PROJECT:
      case WbProtoManager::PROTO_EXTRA:
      case WbProtoManager::PROTO_WEBOTS:
        mDefNodeIndex = -1;
        mNewNodeType = PROTO;
        showNodeInfo(selectedItem->text(FILE_NAME), PROTO, topLevel->type());
        mExportProtoButton->setVisible(true);
        break;
      default:
        mDefNodeIndex = -1;
        mNewNodeType = BASIC;
        showNodeInfo(selectedNode, BASIC, BASIC);
        mExportProtoButton->setVisible(false);
        break;
    }
  }

  mAddButton->setEnabled(!selectedItem->icon(0).isNull() && validForInsertionInBoundingObject);
}

void WbAddNodeDialog::showNodeInfo(const QString &nodeFileName, NodeType nodeType, int variant,
                                   const QString &boundingObjectInfo) {  // TODO: can variant be better?
  QString description;
  QString pixmapPath;

  QString path = nodeFileName;
  if (path.startsWith(WbNetwork::instance()->cacheDirectory()))
    path = WbNetwork::instance()->getUrlFromEphemeralCache(nodeFileName);

  const QFileInfo fileInfo(path);
  const QString modelName = fileInfo.baseName();
  if (nodeType != PROTO && WbNodeModel::isBaseModelName(modelName)) {
    WbNodeModel *nodeModel = WbNodeModel::findModel(modelName);
    assert(nodeModel);
    description = nodeModel->info();

    // set icon path
    pixmapPath = "icons:" + fileInfo.baseName() + ".png";
  } else {
    // the node is a PROTO
    QMap<QString, WbProtoInfo *> list;
    if (variant == WbProtoManager::PROTO_WEBOTS)
      list = WbProtoManager::instance()->webotsProtoList();
    else
      list = WbProtoManager::instance()->protoInfoMap(variant);

    if (!list.contains(modelName)) {
      WbLog::error(tr("'%1' is not a known proto in category '%2'.\n").arg(modelName).arg(variant));
      return;
    }

    WbProtoInfo *info = list.value(modelName);
    assert(info);

    // set documentation url
    if (!info->documentationUrl().isEmpty()) {
      mDocumentationLabel->show();
      mDocumentationLabel->setText(
        tr("Documentation: <a style='color: #5DADE2;' href='%1'>%1</a>").arg(info->documentationUrl()));
    }

    // set license
    if (!info->license().isEmpty()) {
      QString license =
        info->license() + tr(" <a style='color: #5DADE2;' href='%1'>More information.</a>").arg(info->licenseUrl());
      mLicenseLabel->show();
      mLicenseLabel->setText(tr("License: ") + license);
    }

    mInfoText->clear();

    if (!boundingObjectInfo.isEmpty())
      mInfoText->appendHtml(tr("<font color=\"red\">WARNING: this node contains a Geometry with non-positive dimensions and "
                               "hence cannot be inserted in a bounding object.</font><br/>"));

    description = info->description();
    if (description.isEmpty())
      mInfoText->setPlainText(tr("No info available."));
    else {
      // replace carriage returns with spaces where appropriate:
      // "\n\n" => "\n\n": two consecutive carriage returns are preserved (new paragraph)
      // "\n-"  => "\n-": a carriage return followed by a "-" are preserved (bullet list)
      // "\n"   => " ": a single carriage return is transformed into a space (comment line wrap)
      for (int i = 0; i < description.length(); i++) {
        if (description[i] == '\n') {
          if (i < (description.length() - 1)) {
            if (description[i + 1] == '\n' || description[i + 1] == '-') {
              i++;
              continue;
            }
          }
          description[i] = ' ';
        }
      }
      mInfoText->appendPlainText(description.trimmed());
    }
    mInfoText->moveCursor(QTextCursor::Start);

    pixmapPath = QString("%1icons/%2.png").arg(QUrl(path).adjusted(QUrl::RemoveFilename).toString()).arg(modelName);
    printf("ICON WILL BE AT: %s\n", pixmapPath.toUtf8().constData());
  }

  mPixmapLabel->hide();
  if (!pixmapPath.isEmpty()) {
    if (WbUrl::isWeb(pixmapPath)) {
      if (WbNetwork::instance()->isCached(pixmapPath))
        pixmapPath = WbNetwork::instance()->get(pixmapPath);
      else {
        downloadIcon(pixmapPath);
        return;
      }
    } else if (WbUrl::isLocalUrl(pixmapPath))
      pixmapPath = QDir::cleanPath(WbStandardPaths::webotsHomePath() + pixmapPath.mid(9));

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

  QTreeWidgetItem *const nodesItem = new QTreeWidgetItem(QStringList(tr("Base nodes")), NEW);
  QTreeWidgetItem *const worldFileProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (Current World File)"), WbProtoManager::PROTO_WORLD);
  QTreeWidgetItem *const projectProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (Current Project)"), WbProtoManager::PROTO_PROJECT);
  QTreeWidgetItem *const extraProtosItem =
    new QTreeWidgetItem(QStringList(tr("PROTO nodes (Extra Projects)")), WbProtoManager::PROTO_EXTRA);
  QTreeWidgetItem *const webotsProtosItem =
    new QTreeWidgetItem(QStringList("PROTO nodes (Webots Projects)"), WbProtoManager::PROTO_WEBOTS);
  mUsesItem = new QTreeWidgetItem(QStringList("USE"), USE);

  const QStringList basicNodes = WbNodeModel::baseModelNames();
  QTreeWidgetItem *item = NULL;

  const QRegularExpression regexp(
    QRegularExpression::wildcardToRegularExpression(mFindLineEdit->text(), QRegularExpression::UnanchoredWildcardConversion),
    QRegularExpression::CaseInsensitiveOption);

  // add valid basic nodes
  const WbNode::NodeUse nodeUse = static_cast<WbBaseNode *>(mCurrentNode)->nodeUse();
  foreach (const QString &basicNodeName, basicNodes) {
    QFileInfo fileInfo(basicNodeName);
    QString errorMessage;
    if (fileInfo.baseName().contains(regexp) &&
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
      if (!currentFullDefName.contains(regexp))
        continue;
      if (mField->hasRestrictedValues() &&
          (!doFieldRestrictionsAllowNode(currentModelName) && !doFieldRestrictionsAllowNode(defNode->nodeModelName())))
        continue;
      QString nodeFilePath(currentModelName);
      if (!WbNodeModel::isBaseModelName(currentModelName))
        nodeFilePath = WbProtoManager::instance()->findModelPath(currentModelName);

      QStringList strl(QStringList() << currentFullDefName << nodeFilePath);

      if (boInfo && !(dynamic_cast<const WbBaseNode *const>(defNode))->isSuitableForInsertionInBoundingObject())
        strl << INVALID_FOR_INSERTION_IN_BOUNDING_OBJECT;

      QTreeWidgetItem *const child = new QTreeWidgetItem(mUsesItem, strl);

      child->setIcon(0, QIcon("enabledIcons:node.png"));
    }
  }

  // when filtering, don't regenerate WbProtoInfo
  const bool regenerate = qobject_cast<QLineEdit *>(sender()) ? false : true;

  // add World PROTO (i.e. referenced as EXTERNPROTO by the world file)
  int nWorldFileProtosNodes = 0;
  nWorldFileProtosNodes = addProtosFromProtoList(worldFileProtosItem, WbProtoManager::PROTO_WORLD, regexp, regenerate);
  // add Current Project PROTO (all PROTO locally available in the project location)
  int nProjectProtosNodes = 0;
  nProjectProtosNodes = addProtosFromProtoList(projectProtosItem, WbProtoManager::PROTO_PROJECT, regexp, regenerate);
  // add Extra PROTO (all PROTO available in the extra location)
  int nExtraProtosNodes = 0;
  nExtraProtosNodes = addProtosFromProtoList(extraProtosItem, WbProtoManager::PROTO_EXTRA, regexp, regenerate);
  // add Webots PROTO
  int nWebotsProtosNodes = 0;
  nWebotsProtosNodes = addProtosFromProtoList(webotsProtosItem, WbProtoManager::PROTO_WEBOTS, regexp, false);

  mTree->addTopLevelItem(nodesItem);
  mTree->addTopLevelItem(worldFileProtosItem);
  mTree->addTopLevelItem(projectProtosItem);
  mTree->addTopLevelItem(extraProtosItem);
  mTree->addTopLevelItem(webotsProtosItem);
  mTree->addTopLevelItem(mUsesItem);

  // initial selection
  const int nBasicNodes = nodesItem->childCount();
  const int nUseNodes = mUsesItem ? mUsesItem->childCount() : 0;

  // if everything can fit in the tree height then show all
  if (nBasicNodes + nUseNodes + nWorldFileProtosNodes + nProjectProtosNodes + nExtraProtosNodes + nWebotsProtosNodes < 20)
    mTree->expandAll();

  // if no USE nor PROTO items
  if (nBasicNodes && !nUseNodes && !nWorldFileProtosNodes && !nProjectProtosNodes && !nExtraProtosNodes && !nWebotsProtosNodes)
    // then select first basic node
    mTree->setCurrentItem(nodesItem->child(0));
  else
    mTree->setCurrentItem(nodesItem);

  updateItemInfo();
}

int WbAddNodeDialog::addProtosFromProtoList(QTreeWidgetItem *parentItem, int type, const QRegularExpression &regexp,
                                            bool regenerate) {
  int nAddedNodes = 0;
  const QRegularExpression re("(https://raw.githubusercontent.com/cyberbotics/webots/[a-zA-Z0-9\\-\\_\\+]+/)");
  const WbNode::NodeUse nodeUse = static_cast<WbBaseNode *>(mCurrentNode)->nodeUse();

  WbProtoManager::instance()->generateProtoInfoList(type, regenerate);
  const bool flattenHierarchy = type != WbProtoManager::PROTO_WEBOTS;

  QMapIterator<QString, WbProtoInfo *> it(WbProtoManager::instance()->protoInfoMap(type));
  while (it.hasNext()) {
    it.next();
    WbProtoInfo *info = it.value();

    // don't display PROTOs which contain a "hidden" or a "deprecated" tag
    const QStringList tags = info->tags();
    if (tags.contains("deprecated", Qt::CaseInsensitive) || tags.contains("hidden", Qt::CaseInsensitive))
      continue;

    // don't display PROTO nodes which have been filtered-out by the user's "filter" widget.
    const QString baseType = info->baseType();
    QString path = info->url();
    const QString cleanPath = path.replace("webots://", "").replace(re, "").replace(WbStandardPaths::webotsHomePath(), "");
    if (!cleanPath.contains(regexp) && !baseType.contains(regexp))
      continue;

    // don't display non-Robot PROTO nodes containing devices (e.g. Kinect) about to be inserted outside a robot.
    if (!mHasRobotTopNode && !WbNodeUtilities::isRobotTypeName(baseType) && info->needsRobotAncestor())
      continue;

    QString errorMessage;
    const QString nodeName = it.key();
    if (!WbNodeUtilities::isAllowedToInsert(mField, baseType, mCurrentNode, errorMessage, nodeUse, info->slotType(),
                                            QStringList() << baseType << nodeName))
      continue;

    // populate tree
    if (flattenHierarchy) {
      QTreeWidgetItem *item =
        new QTreeWidgetItem(QStringList() << QString("%1 (%2)").arg(nodeName).arg(baseType) << info->url());
      parentItem->addChild(item);
      item->setIcon(0, QIcon("enabledIcons:proto.png"));
      ++nAddedNodes;
    } else {
      QStringList categories = cleanPath.split('/', Qt::SkipEmptyParts);
      QTreeWidgetItem *parent = parentItem;
      foreach (QString folder, categories) {
        if (folder == "projects" || folder == "protos")
          continue;

        const bool isProto = folder.endsWith(".proto");
        bool exists = false;
        int i;
        for (i = 0; i < parent->childCount(); ++i) {
          if (parent->child(i)->text(0) == folder) {
            exists = true;
            break;
          }
        }

        QTreeWidgetItem *subFolder;
        if (exists)
          subFolder = parent->child(i);
        else {
          const QString name = isProto ? QString("%1 (%2)").arg(nodeName).arg(baseType) : folder;
          subFolder = new QTreeWidgetItem(QStringList() << name << info->url());
        }

        if (isProto) {
          subFolder->setIcon(0, QIcon("enabledIcons:proto.png"));
          ++nAddedNodes;
        }
        parent->addChild(subFolder);
        parent = subFolder;
      }
    }
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

void WbAddNodeDialog::accept() {
  if (mNewNodeType != PROTO || mActionType != CREATE) {
    QDialog::accept();
    return;
  }

  // when inserting a PROTO, it's necessary to ensure it is cached (both it and all the sub-proto it depends on). This may not
  // typically be the case hence we are forced to assume nothing is available (the root proto might be available, but not
  // necessarily all its subs, or vice-versa), then trigger the cascaded download (which will download the sub-proto only if
  // necessary) and only when the retriever gives the go ahead the dialog's accept function can actually be executed entirely.
  // In short, two passes are unavoidable for any inserted proto.

  QString path = mTree->selectedItems().at(0)->text(FILE_NAME);
  if (!mRetrievalTriggered) {
    connect(WbProtoManager::instance(), &WbProtoManager::retrievalCompleted, this, &WbAddNodeDialog::accept);
    mRetrievalTriggered = true;  // the second time the accept function is called, no retrieval should occur
    WbProtoManager::instance()->retrieveExternProto(path);
    return;
  }

  WbProtoManager::instance()->declareExternProto(QUrl(path).fileName(), path, true);

  QDialog::accept();
}

void WbAddNodeDialog::exportProto() {
  WbProtoManager::instance()->exportProto(mTree->selectedItems().at(0)->text(FILE_NAME));  // TODO: tidy up when bugs are fixed

  mActionType = EXPORT_PROTO;
  // accept();  // TODO: this will trigger download since was overloaded, same on import button
}